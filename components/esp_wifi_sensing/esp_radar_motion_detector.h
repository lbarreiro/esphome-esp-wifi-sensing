#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct EspRadarMotionResult {
  bool active{false};
  float smooth{0.0f};
  float jitter{0.0f};
  float enter_level{0.0f};
};

class EspRadarMotionDetector {
 public:
  void set_sensitivity(float value) { this->sensitivity_ = value; }
  void set_active_jitter_min(float value) { this->active_jitter_min_ = value; }
  void set_active_filter_ms(uint32_t value) { this->active_filter_ms_ = value; }

  EspRadarMotionResult update(uint32_t metric, uint32_t now_ms) {
    const float x = static_cast<float>(metric);

    if (!this->initialized_) {
      this->initialized_ = true;
      this->smooth_ = x;
      this->previous_smooth_ = x;
      this->last_update_ms_ = now_ms;
      return this->result_();
    }

    const uint32_t dt = now_ms - this->last_update_ms_;
    this->last_update_ms_ = now_ms;

    // Fast exponential smoothing. Higher sensitivity means less smoothing.
    const float alpha = 0.15f + (this->sensitivity_ * 0.45f);
    this->smooth_ += alpha * (x - this->smooth_);

    this->jitter_ = std::fabs(this->smooth_ - this->previous_smooth_);
    this->previous_smooth_ = this->smooth_;

    // Track the quiet-state level only while inactive. This prevents active
    // movement from immediately moving the entry threshold upwards.
    if (!this->active_) {
      const float quiet_alpha = dt >= 1000 ? 0.08f : 0.03f;
      this->quiet_level_ += quiet_alpha * (this->jitter_ - this->quiet_level_);
    }

    const float scale = 1.0f + this->sensitivity_ * 2.0f;
    const float enter_level = this->active_jitter_min_ * scale + this->quiet_level_ * 3.0f;
    const float exit_level = enter_level * 0.55f;

    if (!this->active_) {
      if (this->jitter_ >= enter_level && this->jitter_ >= this->active_jitter_min_) {
        this->active_accumulated_ms_ += dt;
        if (this->active_accumulated_ms_ >= this->active_filter_ms_) {
          this->active_ = true;
          this->inactive_accumulated_ms_ = 0;
        }
      } else {
        this->active_accumulated_ms_ = 0;
      }
    } else {
      if (this->jitter_ <= exit_level) {
        this->inactive_accumulated_ms_ += dt;
        if (this->inactive_accumulated_ms_ >= this->active_filter_ms_) {
          this->active_ = false;
          this->active_accumulated_ms_ = 0;
        }
      } else {
        this->inactive_accumulated_ms_ = 0;
      }
    }

    this->last_enter_level_ = enter_level;
    return this->result_();
  }

 protected:
  EspRadarMotionResult result_() const {
    EspRadarMotionResult result;
    result.active = this->active_;
    result.smooth = this->smooth_;
    result.jitter = this->jitter_;
    result.enter_level = this->last_enter_level_;
    return result;
  }

  bool initialized_{false};
  bool active_{false};
  float sensitivity_{0.5f};
  float active_jitter_min_{0.05f};
  uint32_t active_filter_ms_{500};
  uint32_t last_update_ms_{0};
  uint32_t active_accumulated_ms_{0};
  uint32_t inactive_accumulated_ms_{0};
  float smooth_{0.0f};
  float previous_smooth_{0.0f};
  float jitter_{0.0f};
  float quiet_level_{0.0f};
  float last_enter_level_{0.05f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
