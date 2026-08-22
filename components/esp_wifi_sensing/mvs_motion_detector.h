#pragma once

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
// The threshold is derived from the distribution of quiet-window variance,
// rather than simply multiplying the mean by a fixed factor.
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

    // Build the quiet distribution from complete windows. We need both the
    // mean and spread: mean*2 was too permissive for this CSI noise floor.
    if (!this->calibrated_) {
      this->calibration_count_++;
      const float delta = variance - this->calibration_mean_;
      this->calibration_mean_ += delta / static_cast<float>(this->calibration_count_);
      const float delta2 = variance - this->calibration_mean_;
      this->calibration_m2_ += delta * delta2;

      if (this->calibration_count_ >= kCalibrationWindows) {
        this->baseline_ = this->calibration_mean_;
        this->variance_stddev_ = std::sqrt(
            this->calibration_count_ > 1
                ? this->calibration_m2_ / static_cast<float>(this->calibration_count_ - 1)
                : 0.0f);
        this->calibrated_ = true;
        this->threshold_ = this->threshold_for_baseline_();
      }
      return this->result_();
    }

    const float enter_threshold = this->threshold_for_baseline_();
    // Hysteresis: leaving requires the variance to return well inside the
    // quiet region, not merely dip one sample below the entry threshold.
    const float exit_threshold = this->baseline_ + this->variance_stddev_ *
        std::fmax(0.5f, this->sigma_multiplier_ * 0.60f);
    this->threshold_ = enter_threshold;

    if (!this->active_) {
      if (variance <= enter_threshold) {
        // Learn only quiet observations, and do so slowly. Keep the measured
        // spread stable while adapting the mean noise floor.
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
  static constexpr uint8_t kCalibrationWindows = 16;

  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};

  // Kept under the existing YAML name mvs_threshold_multiplier, but now it
  // has the statistically useful meaning of a sigma multiplier.
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
  uint16_t calibration_count_{0};

  float calibration_mean_{0.0f};
  float calibration_m2_{0.0f};
  float variance_stddev_{0.0f};
  float baseline_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
