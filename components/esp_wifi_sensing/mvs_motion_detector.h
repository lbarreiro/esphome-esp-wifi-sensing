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

// A windowed variance detector inspired by the temporal-statistics idea used
// by ESPectre MVS, but operating on this component's own CSI metric stream.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t value) { this->window_samples_ = value; }
  void set_threshold_multiplier(float value) { this->threshold_multiplier_ = value; }
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

    if (!this->active_) {
      // Slow baseline learning: movement raises the instantaneous variance,
      // but does not immediately redefine the quiet-state baseline.
      const float alpha = this->baseline_initialized_ ? 0.02f : 0.25f;
      this->baseline_ += alpha * (variance - this->baseline_);
      this->baseline_initialized_ = true;
    }

    const float floor = this->baseline_ + 1.0f;
    const float threshold = std::fmax(floor, this->baseline_ * this->threshold_multiplier_);
    this->threshold_ = threshold;

    if (!this->active_) {
      if (variance >= threshold) {
        this->enter_count_++;
        this->exit_count_ = 0;
        if (this->enter_count_ >= this->enter_hits_) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
          this->enter_count_ = 0;
        }
      } else {
        this->enter_count_ = 0;
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      if (variance < threshold * 0.70f) {
        this->exit_count_++;
        if (this->exit_count_ >= this->exit_hits_) {
          this->active_ = false;
          this->exit_count_ = 0;
        }
      } else {
        this->exit_count_ = 0;
      }
    }
    return this->result_();
  }

 private:
  MvsMotionResult result_() const {
    MvsMotionResult r;
    r.active = this->active_;
    r.variance = this->variance_;
    r.threshold = this->threshold_;
    r.baseline = this->baseline_;
    return r;
  }

  static constexpr uint16_t kMaxWindow = 64;
  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};
  float threshold_multiplier_{2.0f};
  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  uint8_t enter_count_{0};
  uint8_t exit_count_{0};
  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};
  bool active_{false};
  bool baseline_initialized_{false};
  float baseline_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
