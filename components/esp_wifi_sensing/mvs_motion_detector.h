#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct MvsMotionResult {
  bool active{false};
  float variance{0.0f};
  float threshold{0.0f};
  float baseline{0.0f};
  float change_rate{0.0f};
};

// MVS (Motion Variance Signature) detector.
// Original detector inspired by the general principle used by
// Espressif-style CSI motion detection: compare short-term CSI activity
// against a learned quiet reference, smooth the temporal change, and require
// persistence. This is not a copy of Espressif's implementation.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t value) { this->window_samples_ = value; }
  void set_threshold_multiplier(float value) { this->sigma_multiplier_ = value; }
  void set_enter_hits(uint8_t value) { this->enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { this->exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { this->hold_time_ms_ = value; }

  MvsMotionResult update(uint32_t metric, uint32_t now_ms) {
    const float x = static_cast<float>(metric);
    if (this->raw_count_ < kRawWindow) {
      this->raw_[this->raw_count_++] = x;
    } else {
      for (uint16_t i = 1; i < kRawWindow; ++i) this->raw_[i - 1] = this->raw_[i];
      this->raw_[kRawWindow - 1] = x;
    }

    const uint16_t n = std::min<uint16_t>(this->raw_count_, this->window_samples_);
    if (n < 16) return this->result_();

    const uint16_t start = this->raw_count_ - n;
    float mean = 0.0f;
    for (uint16_t i = start; i < this->raw_count_; ++i) mean += this->raw_[i];
    mean /= static_cast<float>(n);

    float variance = 0.0f;
    for (uint16_t i = start; i < this->raw_count_; ++i) {
      const float d = this->raw_[i] - mean;
      variance += d * d;
    }
    variance /= static_cast<float>(n);
    this->variance_ = variance;

    // Calibration and decisions are both strictly 1 Hz.
    if (this->last_1hz_ms_ != 0 && static_cast<uint32_t>(now_ms - this->last_1hz_ms_) < 1000)
      return this->result_();
    this->last_1hz_ms_ = now_ms;

    // Diagnostic only: rate of change of the variance-derived activity.
    // It is also used as temporal evidence for entry below; it is NOT a
    // standalone threshold.
    const float activity = std::sqrt(std::fmax(0.0f, variance));
    if (this->have_previous_activity_)
      this->change_rate_ = activity - this->previous_activity_;
    else
      this->change_rate_ = 0.0f;
    this->previous_activity_ = activity;
    this->have_previous_activity_ = true;

    if (!this->calibrated_) {
      if (this->calibration_start_ms_ == 0) this->calibration_start_ms_ = now_ms;
      if (this->calibration_count_ < kCalibrationSamples)
        this->calibration_[this->calibration_count_++] = variance;

      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples) {
        this->finish_calibration_();
      }
      return this->result_();
    }

    this->threshold_ = this->baseline_ + this->variance_stddev_ * this->sigma_multiplier_;

    const float quiet_activity = std::sqrt(std::fmax(1.0f, this->baseline_));
    const float ratio = activity / quiet_activity;
    const float change = std::fmax(0.0f, ratio - 1.0f);
    // Faster response than the previous 80/20 smoothing. The variance itself
    // is already windowed, so excessive second-stage smoothing was suppressing
    // real transitions.
    this->change_score_ = 0.60f * this->change_score_ + 0.40f * change;

    if (!this->active_ && !this->rearm_required_ && change < kQuietChange)
      this->baseline_ = 0.997f * this->baseline_ + 0.003f * variance;

    if (!this->active_) {
      if (this->rearm_required_) {
        if (change < kQuietChange) {
          if (++this->quiet_seconds_ >= kRearmSeconds) {
            this->rearm_required_ = false;
            this->quiet_seconds_ = 0;
          }
        } else {
          this->quiet_seconds_ = 0;
        }
        return this->result_();
      }

      // Entry is now a temporal signature rather than accumulated amplitude
      // alone. A meaningful rise in activity is the start of an event. Once
      // the signal is elevated, either a sustained elevated level or a
      // subsequent falling edge completes the signature. Change rate is used
      // as evidence, never as an absolute movement threshold.
      const bool elevated = this->change_score_ >= kModerateChange;
      const bool strong = this->change_score_ >= kStrongChange;
      const bool rising = this->change_rate_ >= kRiseRate;
      const bool falling = this->change_rate_ <= -kFallRate;

      if (rising && elevated)
        this->signature_score_ += 2;
      else if (strong)
        this->signature_score_ += 1;
      else if (falling && this->signature_score_ > 0)
        this->signature_score_ += 1;
      else
        this->signature_score_ = std::max(0, this->signature_score_ - 1);

      if (elevated)
        this->elevated_seconds_++;
      else
        this->elevated_seconds_ = 0;

      if (++this->enter_window_seconds_ >= kEvidenceWindowSeconds) {
        const bool sustained = this->elevated_seconds_ >= kMinimumElevatedSeconds;
        if (this->signature_score_ >= kRequiredSignatureScore ||
            (sustained && this->signature_score_ >= kSustainedSignatureScore)) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
        }
        this->signature_score_ = 0;
        this->enter_window_seconds_ = 0;
        this->elevated_seconds_ = 0;
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      if (this->change_score_ < kExitChange)
        ++this->exit_seconds_;
      else
        this->exit_seconds_ = 0;

      if (this->exit_seconds_ >= kExitSeconds) {
        this->active_ = false;
        this->rearm_required_ = true;
        this->exit_seconds_ = 0;
        this->signature_score_ = 0;
        this->enter_window_seconds_ = 0;
        this->elevated_seconds_ = 0;
      }
    }

    return this->result_();
  }

 private:
  void finish_calibration_() {
    // Robust calibration: discard the upper 10% of samples first, then use
    // the median of the remaining quiet distribution as the noise reference.
    std::sort(this->calibration_, this->calibration_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);
    const uint16_t median_index = usable / 2;
    this->baseline_ = std::fmax(1.0f, this->calibration_[median_index]);

    float deviations[kCalibrationSamples];
    for (uint16_t i = 0; i < usable; ++i)
      deviations[i] = std::fabs(this->calibration_[i] - this->baseline_);
    std::sort(deviations, deviations + usable);
    const float mad = deviations[median_index];
    this->variance_stddev_ = std::fmax(1.0f, 1.4826f * mad);

    this->calibrated_ = true;
    this->threshold_ = this->baseline_ + this->variance_stddev_ * this->sigma_multiplier_;
  }

  MvsMotionResult result_() const {
    MvsMotionResult r;
    r.active = this->active_;
    r.variance = this->variance_;
    r.threshold = this->threshold_;
    r.baseline = this->baseline_;
    r.change_rate = this->change_rate_;
    return r;
  }

  static constexpr uint16_t kRawWindow = 64;
  static constexpr uint16_t kCalibrationSamples = 300;
  static constexpr uint16_t kMinimumCalibrationSamples = 240;
  static constexpr uint32_t kCalibrationMs = 300000;
  static constexpr uint16_t kRearmSeconds = 10;
  static constexpr uint16_t kExitSeconds = 5;
  static constexpr uint16_t kEvidenceWindowSeconds = 5;
  static constexpr uint16_t kMinimumElevatedSeconds = 2;
  static constexpr int kRequiredSignatureScore = 3;
  static constexpr int kSustainedSignatureScore = 4;
  static constexpr float kStrongChange = 0.75f;
  static constexpr float kModerateChange = 0.40f;
  static constexpr float kQuietChange = 0.15f;
  static constexpr float kExitChange = 0.20f;
  static constexpr float kRiseRate = 12.0f;
  static constexpr float kFallRate = 12.0f;

  float raw_[kRawWindow]{};
  uint16_t raw_count_{0};
  uint16_t window_samples_{32};

  float calibration_[kCalibrationSamples]{};
  uint16_t calibration_count_{0};
  uint32_t calibration_start_ms_{0};
  uint32_t last_1hz_ms_{0};

  float sigma_multiplier_{2.0f};
  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  int signature_score_{0};
  uint16_t enter_window_seconds_{0};
  uint16_t elevated_seconds_{0};
  uint16_t exit_seconds_{0};
  uint16_t quiet_seconds_{0};

  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  bool active_{false};
  bool calibrated_{false};
  bool rearm_required_{false};
  bool have_previous_activity_{false};

  float baseline_{0.0f};
  float variance_stddev_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
  float change_score_{0.0f};
  float previous_activity_{0.0f};
  float change_rate_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
