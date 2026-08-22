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

// Temporal CSI motion detector. The detector deliberately keeps the existing
// MVS/variance signal as a diagnostic, but the motion decision is now based on
// the SHAPE of the signal: energy above the quiet floor, persistence, and a
// recovery period. A 60 s hold is presentation-only and never feeds detection.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t value) { this->window_samples_ = value; }
  void set_threshold_multiplier(float value) { this->sigma_multiplier_ = value; }
  void set_enter_hits(uint8_t value) { this->enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { this->exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { this->hold_time_ms_ = value; }

  MvsMotionResult update(uint32_t metric, uint32_t now_ms) {
    const float x = static_cast<float>(metric);
    if (this->count_ < kMaxWindow) this->window_[this->count_++] = x;
    else {
      for (uint16_t i = 1; i < kMaxWindow; ++i) this->window_[i - 1] = this->window_[i];
      this->window_[kMaxWindow - 1] = x;
    }

    const uint16_t n = std::min<uint16_t>(this->count_, this->window_samples_);
    if (n < 8) return this->result_();

    float mean = 0.0f;
    const uint16_t start = this->count_ - n;
    for (uint16_t i = start; i < this->count_; ++i) mean += this->window_[i];
    mean /= static_cast<float>(n);

    float variance = 0.0f;
    for (uint16_t i = start; i < this->count_; ++i) {
      const float d = this->window_[i] - mean;
      variance += d * d;
    }
    variance /= static_cast<float>(n);
    this->variance_ = variance;

    // Five-minute, 1 Hz quiet calibration. Only the lower 90% is used so an
    // occasional event during startup cannot become the noise floor.
    if (!this->calibrated_) {
      if (this->calibration_start_ms_ == 0) this->calibration_start_ms_ = now_ms;
      if (this->last_calibration_sample_ms_ == 0 ||
          static_cast<uint32_t>(now_ms - this->last_calibration_sample_ms_) >= 1000) {
        this->last_calibration_sample_ms_ = now_ms;
        if (this->calibration_count_ < kMaxCalibrationSamples)
          this->calibration_values_[this->calibration_count_++] = variance;
      }
      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationDurationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples) {
        this->finish_calibration_();
      }
      return this->result_();
    }

    // Dynamic quiet floor. Threshold is diagnostic and remains visible in HA.
    this->threshold_ = this->threshold_for_baseline_();
    const float enter_level = this->threshold_;
    const float exit_level = this->baseline_ + this->variance_stddev_ * 0.45f;

    // Compress excess energy into a dimensionless score. This is what the
    // state machine uses instead of a single absolute variance crossing.
    const float excess = std::fmax(0.0f, variance - enter_level);
    const float score = excess / std::fmax(1.0f, enter_level);

    if (!this->active_) {
      if (this->rearm_required_) {
        if (variance <= exit_level) {
          if (this->quiet_seconds_ < kRearmSeconds) ++this->quiet_seconds_;
          if (this->quiet_seconds_ >= kRearmSeconds) {
            this->rearm_required_ = false;
            this->quiet_seconds_ = 0;
          }
        } else {
          this->quiet_seconds_ = 0;
        }
        return this->result_();
      }

      // A single spike is not movement. Require a sustained energy pattern,
      // and accumulate strength instead of counting raw CSI callbacks.
      if (score >= kStrongScore) {
        if (this->strong_seconds_ < kMaxStrongSeconds) ++this->strong_seconds_;
      } else if (score >= kWeakScore) {
        if (this->strong_seconds_ > 0) --this->strong_seconds_;
      } else {
        this->strong_seconds_ = 0;
      }

      if (this->strong_seconds_ >= kRequiredStrongSeconds) {
        this->active_ = true;
        this->hold_until_ms_ = now_ms + this->hold_time_ms_;
        this->strong_seconds_ = 0;
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      // Once the presentation hold expires, OFF requires several quiet
      // seconds. The signal itself, not the 60 s timer, decides recovery.
      if (variance <= exit_level) {
        if (this->exit_quiet_seconds_ < kExitQuietSeconds) ++this->exit_quiet_seconds_;
        if (this->exit_quiet_seconds_ >= kExitQuietSeconds) {
          this->active_ = false;
          this->rearm_required_ = true;
          this->exit_quiet_seconds_ = 0;
          this->strong_seconds_ = 0;
        }
      } else {
        this->exit_quiet_seconds_ = 0;
      }
    }

    return this->result_();
  }

 private:
  void finish_calibration_() {
    std::sort(this->calibration_values_,
              this->calibration_values_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);

    float sum = 0.0f;
    for (uint16_t i = 0; i < usable; ++i) sum += this->calibration_values_[i];
    this->baseline_ = sum / static_cast<float>(usable);

    float sq = 0.0f;
    for (uint16_t i = 0; i < usable; ++i) {
      const float d = this->calibration_values_[i] - this->baseline_;
      sq += d * d;
    }
    this->variance_stddev_ = std::sqrt(sq / static_cast<float>(std::max<uint16_t>(1, usable - 1)));

    const uint16_t p99_index = static_cast<uint16_t>((usable - 1) * 0.99f);
    this->quiet_p99_ = this->calibration_values_[p99_index];
    this->calibrated_ = true;
    this->threshold_ = this->threshold_for_baseline_();
  }

  float threshold_for_baseline_() const {
    return std::fmax(this->baseline_ + this->variance_stddev_ * this->sigma_multiplier_,
                     this->quiet_p99_);
  }

  MvsMotionResult result_() const {
    MvsMotionResult r;
    r.active = this->active_;
    r.variance = this->variance_;
    r.threshold = this->threshold_;
    r.baseline = this->baseline_;
    return r;
  }

  static constexpr uint16_t kMaxWindow = 64;
  static constexpr uint16_t kMaxCalibrationSamples = 360;
  static constexpr uint16_t kMinimumCalibrationSamples = 240;
  static constexpr uint32_t kCalibrationDurationMs = 300000;
  static constexpr uint16_t kRearmSeconds = 10;
  static constexpr uint16_t kExitQuietSeconds = 5;
  static constexpr uint16_t kRequiredStrongSeconds = 3;
  static constexpr uint16_t kMaxStrongSeconds = 6;
  static constexpr float kStrongScore = 0.50f;
  static constexpr float kWeakScore = 0.20f;

  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};

  float calibration_values_[kMaxCalibrationSamples]{};
  uint16_t calibration_count_{0};
  uint32_t calibration_start_ms_{0};
  uint32_t last_calibration_sample_ms_{0};

  float sigma_multiplier_{2.0f};
  float baseline_alpha_{0.005f};
  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  uint8_t strong_seconds_{0};
  uint8_t exit_quiet_seconds_{0};
  uint8_t quiet_seconds_{0};

  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  bool active_{false};
  bool calibrated_{false};
  bool rearm_required_{false};

  float quiet_p99_{0.0f};
  float variance_stddev_{0.0f};
  float baseline_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
