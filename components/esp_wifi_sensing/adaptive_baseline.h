#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

class AdaptiveBaseline {
 public:
  void set_sigma_multiplier(float multiplier) { this->sigma_multiplier_ = multiplier; }
  void set_baseline_rise_time(uint32_t time_ms) { this->baseline_rise_time_ms_ = time_ms; }
  void set_baseline_fall_time(uint32_t time_ms) { this->baseline_fall_time_ms_ = time_ms; }
  void set_learning_delay(uint32_t delay_ms) { this->learning_delay_ms_ = delay_ms; }

  void reset() {
    this->learning_allowed_after_ms_ = 0;
    this->initialized_ = false;
    this->motion_was_active_ = false;
    this->baseline_mean_ = 0.0f;
    this->baseline_variance_ = 0.0f;
    this->baseline_stddev_ = 0.0f;
    this->adaptive_threshold_ = 0.0f;
  }

  void update(float value, bool motion_active, uint32_t now_ms, uint32_t interval_ms) {
    if (motion_active) {
      this->motion_was_active_ = true;
      this->learning_allowed_after_ms_ = now_ms + this->learning_delay_ms_;
      return;
    }

    if (this->motion_was_active_) {
      this->motion_was_active_ = false;
      this->learning_allowed_after_ms_ = now_ms + this->learning_delay_ms_;
    }

    if (now_ms < this->learning_allowed_after_ms_) {
      return;
    }

    if (!this->initialized_) {
      this->baseline_mean_ = value;
      this->baseline_variance_ = 0.0f;
      this->baseline_stddev_ = 0.0f;
      this->adaptive_threshold_ = value;
      this->initialized_ = true;
      return;
    }

    const uint32_t time_constant_ms = value > this->baseline_mean_ ?
        this->baseline_rise_time_ms_ : this->baseline_fall_time_ms_;
    const float alpha = this->ema_alpha_(interval_ms, time_constant_ms);
    const float delta = value - this->baseline_mean_;

    this->baseline_mean_ += alpha * delta;
    const float variance_sample = delta * delta;
    this->baseline_variance_ += alpha * (variance_sample - this->baseline_variance_);
    if (this->baseline_variance_ < 0.0f) {
      this->baseline_variance_ = 0.0f;
    }
    this->baseline_stddev_ = std::sqrt(this->baseline_variance_);
    this->adaptive_threshold_ = this->baseline_mean_ + this->sigma_multiplier_ * this->baseline_stddev_;
  }

  float baseline_mean() const { return this->baseline_mean_; }
  float baseline_stddev() const { return this->baseline_stddev_; }
  float adaptive_threshold() const { return this->adaptive_threshold_; }
  bool initialized() const { return this->initialized_; }

 protected:
  float ema_alpha_(uint32_t interval_ms, uint32_t time_constant_ms) const {
    if (time_constant_ms == 0) {
      return 1.0f;
    }
    return 1.0f - std::exp(-static_cast<float>(interval_ms) / static_cast<float>(time_constant_ms));
  }

  float sigma_multiplier_{4.0f};
  uint32_t baseline_rise_time_ms_{1800000};
  uint32_t baseline_fall_time_ms_{1800000};
  uint32_t learning_delay_ms_{60000};
  uint32_t learning_allowed_after_ms_{0};
  bool initialized_{false};
  bool motion_was_active_{false};
  float baseline_mean_{0.0f};
  float baseline_variance_{0.0f};
  float baseline_stddev_{0.0f};
  float adaptive_threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
