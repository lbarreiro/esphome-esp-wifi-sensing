#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct EspRadarMotionResult {
  bool active{false};
  float jitter{0.0f};
  float smooth{0.0f};
  float enter_level{0.0f};
  float exit_level{0.0f};
};

// Espressif-inspired single-channel motion FSM.
// Uses temporal jitter, smoothing, separate enter/exit levels (hysteresis),
// and an active confirmation filter. It is not a copy of Espressif's
// internal multi-channel implementation.
class EspRadarMotionDetector {
 public:
  void set_sensitivity(float value) {
    if (value < 0.05f) value = 0.05f;
    if (value > 1.0f) value = 1.0f;
    this->sensitivity_ = value;
  }

  void set_active_jitter_min(float value) {
    if (value < 0.0f) value = 0.0f;
    this->active_jitter_min_ = value;
  }

  void set_active_filter_ms(uint32_t value) { this->active_filter_ms_ = value; }

  EspRadarMotionResult update(uint32_t metric, uint32_t now_ms) {
    if (!this->initialized_) {
      this->initialized_ = true;
      this->previous_metric_ = metric;
      this->last_update_ms_ = now_ms;
      return this->result_();
    }

    const uint32_t dt = now_ms - this->last_update_ms_;
    this->last_update_ms_ = now_ms;

    const float raw_jitter = std::fabs(static_cast<float>(metric) - static_cast<float>(this->previous_metric_));
    this->previous_metric_ = metric;

    const float scale = std::max(1.0f, std::fabs(static_cast<float>(metric)));
    this->jitter_ = raw_jitter / scale;

    const float alpha = dt == 0 ? 1.0f : std::min(1.0f, static_cast<float>(dt) / 250.0f);
    this->smooth_ += alpha * (this->jitter_ - this->smooth_);

    // Higher sensitivity means a lower level required to enter ACTIVE.
    const float base = std::max(this->active_jitter_min_, 0.01f);
    const float sensitivity_factor = 1.50f - this->sensitivity_;
    this->enter_level_ = base * sensitivity_factor;
    this->exit_level_ = this->enter_level_ * 0.55f;

    const bool candidate = this->smooth_ >= this->enter_level_ &&
                           this->jitter_ >= this->active_jitter_min_;

    if (candidate) {
      if (!this->candidate_active_) {
        this->candidate_active_ = true;
        this->candidate_since_ms_ = now_ms;
      }
      if (!this->active_ && now_ms - this->candidate_since_ms_ >= this->active_filter_ms_) {
        this->active_ = true;
      }
    } else {
      this->candidate_active_ = false;
      if (this->active_ && this->smooth_ <= this->exit_level_) {
        this->active_ = false;
      }
    }

    return this->result_();
  }

 private:
  EspRadarMotionResult result_() const {
    EspRadarMotionResult result{};
    result.active = this->active_;
    result.jitter = this->jitter_;
    result.smooth = this->smooth_;
    result.enter_level = this->enter_level_;
    result.exit_level = this->exit_level_;
    return result;
  }

  bool initialized_{false};
  uint32_t previous_metric_{0};
  uint32_t last_update_ms_{0};
  uint32_t candidate_since_ms_{0};
  uint32_t active_filter_ms_{500};
  float sensitivity_{0.5f};
  float active_jitter_min_{0.05f};
  float jitter_{0.0f};
  float smooth_{0.0f};
  float enter_level_{0.0f};
  float exit_level_{0.0f};
  bool candidate_active_{false};
  bool active_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
