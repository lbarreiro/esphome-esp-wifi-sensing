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

// Windowed variance detector inspired by the temporal-statistics approach of
// ESPectre MVS, but implemented independently for this CSI metric stream.
// The quiet baseline is learned only from quiet windows and a completed
// motion window is discarded before the detector can re-arm.
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

    if (!this->baseline_initialized_) {
      this->baseline_ = variance;
      this->baseline_initialized_ = true;
      this->threshold_ = this->threshold_for_baseline_();
      return this->result_();
    }

    const float enter_threshold = this->threshold_for_baseline_();
    const float exit_threshold = enter_threshold * 0.70f;
    this->threshold_ = enter_threshold;

    if (!this->active_) {
      // Learn the quiet environment only when the complete current window is
      // below the entry threshold. Motion cannot raise its own threshold.
      if (variance < enter_threshold) {
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
      if (variance < exit_threshold) {
        this->exit_count_++;
        if (this->exit_count_ >= this->exit_hits_) {
          this->active_ = false;
          this->exit_count_ = 0;
          this->enter_count_ = 0;

          // Critical re-arm fix: do not leave the 32-sample motion window in
          // place after OFF. Otherwise its old high-variance samples survive
          // the 60 s hold and immediately satisfy the entry condition again.
          this->count_ = 0;
          this->variance_ = 0.0f;
          this->threshold_ = this->threshold_for_baseline_();
        }
      } else {
        this->exit_count_ = 0;
      }
    }

    return this->result_();
  }

 private:
  float threshold_for_baseline_() const {
    return std::fmax(this->baseline_ + 1.0f,
                     this->baseline_ * this->threshold_multiplier_);
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
  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};
  float threshold_multiplier_{2.0f};
  float baseline_alpha_{0.02f};
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
