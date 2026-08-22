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
};

// MVS (Motion Variance Signature) detector.
//
// Original detector inspired by the general principle used by Espressif-style
// CSI motion detection: compare short-term CSI activity against a learned
// quiet reference, smooth the temporal change, and require persistence.
// This is not a copy of Espressif's implementation.
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

    // Work in the amplitude domain (sqrt variance), because ratios in this
    // domain are much less dominated by rare squared outliers than raw
    // variance crossings.
    const float activity = std::sqrt(std::fmax(0.0f, variance));
    const float quiet_activity = std::sqrt(std::fmax(1.0f, this->baseline_));
    const float ratio = activity / quiet_activity;

    // Slow envelope of CHANGE rather than an absolute level detector.
    const float change = std::fmax(0.0f, ratio - 1.0f);
    this->change_score_ = 0.80f * this->change_score_ + 0.20f * change;

    // Adapt the quiet reference only while the signal is clearly quiet.
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

      // Temporal evidence: moderate change contributes one point, strong
      // change contributes two. Three points are required inside five seconds.
      if (this->change_score_ >= kStrongChange)
        this->enter_score_ += 2;
      else if (this->change_score_ >= kModerateChange)
        this->enter_score_ += 1;
      else
        this->enter_score_ = std::max(0, this->enter_score_ - 1);

      if (++this->enter_window_seconds_ >= kEvidenceWindowSeconds) {
        if (this->enter_score_ >= kRequiredEnterScore) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
        }
        this->enter_score_ = 0;
        this->enter_window_seconds_ = 0;
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      // Hold is presentation-only. OFF requires the smoothed signal to return
      // to the quiet region for several real seconds.
      if (this->change_score_ < kExitChange)
        ++this->exit_seconds_;
      else
        this->exit_seconds_ = 0;

      if (this->exit_seconds_ >= kExitSeconds) {
        this->active_ = false;
        this->rearm_required_ = true;
        this->exit_seconds_ = 0;
        this->enter_score_ = 0;
        this->enter_window_seconds_ = 0;
      }
    }

    return this->result_();
  }

 private:
  void finish_calibration_() {
    std::sort(this->calibration_, this->calibration_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);

    float sum = 0.0f;
    for (uint16_t i = 0; i < usable; ++i) sum += this->calibration_[i];
    this->baseline_ = std::fmax(1.0f, sum / static_cast<float>(usable));

    float sq = 0.0f;
    for (uint16_t i = 0; i < usable; ++i) {
      const float d = this->calibration_[i] - this->baseline_;
      sq += d * d;
    }
    this->variance_stddev_ = std::sqrt(sq / static_cast<float>(std::max<uint16_t>(1, usable - 1)));
    this->calibrated_ = true;
    this->threshold_ = this->baseline_ + this->variance_stddev_ * this->sigma_multiplier_;
  }

  MvsMotionResult result_() const {
    MvsMotionResult r;
    r.active = this->active_;
    r.variance = this->variance_;
    r.threshold = this->threshold_;
    r.baseline = this->baseline_;
    return r;
  }

  static constexpr uint16_t kRawWindow = 64;
  static constexpr uint16_t kCalibrationSamples = 300;
  static constexpr uint16_t kMinimumCalibrationSamples = 240;
  static constexpr uint32_t kCalibrationMs = 300000;
  static constexpr uint16_t kRearmSeconds = 10;
  static constexpr uint16_t kExitSeconds = 5;
  static constexpr uint16_t kEvidenceWindowSeconds = 5;
  static constexpr int kRequiredEnterScore = 3;
  static constexpr float kStrongChange = 0.75f;
  static constexpr float kModerateChange = 0.40f;
  static constexpr float kQuietChange = 0.15f;
  static constexpr float kExitChange = 0.20f;

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
  int enter_score_{0};
  uint16_t enter_window_seconds_{0};
  uint16_t exit_seconds_{0};
  uint16_t quiet_seconds_{0};

  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  bool active_{false};
  bool calibrated_{false};
  bool rearm_required_{false};

  float baseline_{0.0f};
  float variance_stddev_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
  float change_score_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
