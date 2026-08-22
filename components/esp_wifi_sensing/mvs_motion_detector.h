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

// Windowed variance detector using a statistically calibrated quiet baseline.
// Calibration is deliberately long and sampled at 1 Hz so the startup
// threshold represents the real environmental noise floor rather than a few
// initial CSI windows.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t value) { this->window_samples_ = value; }
  void set_threshold_multiplier(float value) { this->sigma_multiplier_ = value; }
  void set_enter_hits(uint8_t value) { this->enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { this->exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { this->hold_time_ms_ = value; }

  MvsMotionResult update(uint32_t metric, uint32_t now_ms) {
    const float x = static_cast<float>(metric);
    if (this->count_ < kMaxWindow) {
      this->window_[this->count_++] = x;
    } else {
      for (uint16_t i = 1; i < kMaxWindow; i++) this->window_[i - 1] = this->window_[i];
      this->window_[kMaxWindow - 1] = x;
    }

    const uint16_t n = this->count_ < this->window_samples_ ? this->count_ : this->window_samples_;
    if (n < 8) return this->result_();

    float mean = 0.0f;
    const uint16_t start = this->count_ - n;
    for (uint16_t i = start; i < this->count_; i++) mean += this->window_[i];
    mean /= static_cast<float>(n);

    float variance = 0.0f;
    for (uint16_t i = start; i < this->count_; i++) {
      const float d = this->window_[i] - mean;
      variance += d * d;
    }
    variance /= static_cast<float>(n);
    this->variance_ = variance;

    // Calibration is time based, not update-count based. The detector may be
    // fed much faster than the 1 Hz diagnostic sensors, so counting calls
    // would make the calibration finish almost immediately.
    if (!this->calibrated_) {
      if (this->last_calibration_sample_ms_ == 0 ||
          static_cast<uint32_t>(now_ms - this->last_calibration_sample_ms_) >= kCalibrationSampleMs) {
        this->last_calibration_sample_ms_ = now_ms;
        if (this->calibration_count_ < kMaxCalibrationSamples) {
          this->calibration_values_[this->calibration_count_++] = variance;
        }
      }

      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationDurationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples) {
        this->finish_calibration_();
      }
      return this->result_();
    }

    const float enter_threshold = this->threshold_for_baseline_();
    // Hysteresis: exit is deliberately below the entry level.
    const float exit_threshold = this->baseline_ + this->variance_stddev_ *
        std::fmax(0.5f, this->sigma_multiplier_ * 0.60f);
    this->threshold_ = enter_threshold;

    if (!this->active_) {
      if (variance <= enter_threshold) {
        // Slow adaptation only from quiet observations.
        this->baseline_ += this->baseline_alpha_ * (variance - this->baseline_);
        this->enter_count_ = 0;
      } else {
        this->enter_count_++;
        if (this->enter_count_ >= this->enter_hits_) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
          this->enter_count_ = 0;
        }
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      if (variance <= exit_threshold) {
        this->exit_count_++;
        if (this->exit_count_ >= this->exit_hits_) {
          this->active_ = false;
          this->exit_count_ = 0;
          this->enter_count_ = 0;
          this->count_ = 0;
          this->variance_ = 0.0f;
        }
      } else {
        this->exit_count_ = 0;
      }
    }

    return this->result_();
  }

 private:
  void finish_calibration_() {
    // Ignore the highest 10% of calibration windows. This prevents an
    // occasional movement during startup from becoming part of the normal
    // noise floor and pushing the threshold upwards.
    std::sort(this->calibration_values_,
              this->calibration_values_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);

    float sum = 0.0f;
    for (uint16_t i = 0; i < usable; i++) sum += this->calibration_values_[i];
    this->baseline_ = sum / static_cast<float>(usable);

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < usable; i++) {
      const float d = this->calibration_values_[i] - this->baseline_;
      sum_sq += d * d;
    }
    this->variance_stddev_ = std::sqrt(sum_sq / static_cast<float>(usable > 1 ? usable - 1 : 1));
    this->calibrated_ = true;
    this->threshold_ = this->threshold_for_baseline_();
  }

  float threshold_for_baseline_() const {
    return std::fmax(this->baseline_ + this->variance_stddev_ * this->sigma_multiplier_, 1.0f);
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
  static constexpr uint16_t kMinimumCalibrationSamples = 120;
  static constexpr uint32_t kCalibrationDurationMs = 300000;  // 5 minutes
  static constexpr uint32_t kCalibrationSampleMs = 1000;      // 1 Hz

  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};

  float calibration_values_[kMaxCalibrationSamples]{};
  uint16_t calibration_count_{0};
  uint32_t calibration_start_ms_{0};
  uint32_t last_calibration_sample_ms_{0};

  // Kept under the existing YAML name mvs_threshold_multiplier, but it is a
  // sigma multiplier: threshold = baseline + sigma * multiplier.
  float sigma_multiplier_{2.0f};
  float baseline_alpha_{0.01f};

  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  uint8_t enter_count_{0};
  uint8_t exit_count_{0};

  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  bool active_{false};
  bool calibrated_{false};

  float variance_stddev_{0.0f};
  float baseline_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
