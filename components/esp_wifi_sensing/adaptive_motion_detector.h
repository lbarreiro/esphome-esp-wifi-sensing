#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct AdaptiveMotionDetectorResult {
  bool candidate{false};
  bool persistence_on{false};
  bool motion{false};
  float baseline_mean{0.0f};
  float baseline_stddev{0.0f};
  float adaptive_threshold{0.0f};
};

class AdaptiveMotionDetector {
 public:
  static constexpr uint8_t kPersistenceWindowSize = 10;
  static constexpr float kPersistenceRequirement = 0.70f;

  void set_adaptive_threshold_enabled(bool enabled) { this->adaptive_threshold_enabled_ = enabled; }
  void set_sigma_multiplier(float multiplier) { this->sigma_multiplier_ = multiplier; }
  void set_baseline_rise_time_ms(uint32_t time_ms) { this->baseline_rise_time_ms_ = time_ms; }
  void set_baseline_fall_time_ms(uint32_t time_ms) { this->baseline_fall_time_ms_ = time_ms; }
  void set_learning_delay_ms(uint32_t time_ms) { this->learning_delay_ms_ = time_ms; }
  void set_debounce_samples(uint32_t samples) { this->debounce_samples_ = samples; }
  void set_warmup_time_ms(uint32_t time_ms) { this->warmup_time_ms_ = time_ms; }
  void set_motion_hold_time_ms(uint32_t time_ms) { this->motion_hold_time_ms_ = time_ms; }

  AdaptiveMotionDetectorResult update(uint32_t metric, uint32_t now_ms) {
    if (!this->initialized_) {
      this->initialized_ = true;
      this->started_ms_ = now_ms;
      this->last_update_ms_ = now_ms;
      this->baseline_mean_ = static_cast<float>(metric);
      this->baseline_variance_ = 0.0f;
      this->reset_persistence_();
      return this->result_(false, now_ms);
    }

    const uint32_t elapsed_ms = now_ms - this->last_update_ms_;
    this->last_update_ms_ = now_ms;
    this->update_baseline_(metric, elapsed_ms);

    const bool ready = now_ms - this->started_ms_ >= this->warmup_time_ms_ + this->learning_delay_ms_;
    if (!ready) {
      this->motion_ = false;
      this->debounce_count_ = 0;
      this->last_motion_ms_ = 0;
      this->reset_persistence_();
      return this->result_(false, now_ms);
    }

    const float threshold = this->adaptive_threshold_value_();
    const float difference = std::fabs(static_cast<float>(metric) - this->baseline_mean_);
    const bool candidate = this->adaptive_threshold_enabled_ ? difference > threshold : metric > threshold;
    const bool persistence_on = this->update_persistence_(candidate);

    if (candidate && persistence_on) {
      if (this->debounce_count_ < this->debounce_samples_) {
        this->debounce_count_++;
      }
      if (this->debounce_count_ >= this->debounce_samples_) {
        this->motion_ = true;
        this->last_motion_ms_ = now_ms;
      }
    } else {
      this->debounce_count_ = 0;
      if (this->motion_ && now_ms - this->last_motion_ms_ >= this->motion_hold_time_ms_) {
        this->motion_ = false;
      } else if (!this->motion_) {
        this->motion_ = false;
      }
    }

    return this->result_(candidate, now_ms);
  }

  bool persistence_on() const {
    return this->persistence_valid_samples_ == kPersistenceWindowSize &&
           this->persistence() >= kPersistenceRequirement;
  }

  float persistence() const {
    if (this->persistence_valid_samples_ == 0) {
      return 0.0f;
    }
    return static_cast<float>(this->persistence_elevated_count_) /
           static_cast<float>(this->persistence_valid_samples_);
  }

  float baseline_mean() const { return this->baseline_mean_; }
  float baseline_stddev() const { return std::sqrt(this->baseline_variance_); }
  float adaptive_threshold() const { return this->adaptive_threshold_value_(); }
  bool motion() const { return this->motion_; }
  bool persistence_has_valid_window() const { return this->persistence_valid_samples_ == kPersistenceWindowSize; }

 protected:
  void update_baseline_(uint32_t metric, uint32_t elapsed_ms) {
    const float sample = static_cast<float>(metric);
    const uint32_t time_ms = sample >= this->baseline_mean_ ? this->baseline_rise_time_ms_ :
                                                            this->baseline_fall_time_ms_;
    float alpha = 1.0f;
    if (time_ms > 0) {
      alpha = static_cast<float>(elapsed_ms) / static_cast<float>(time_ms);
      if (alpha > 1.0f) {
        alpha = 1.0f;
      }
    }

    const float previous_mean = this->baseline_mean_;
    this->baseline_mean_ += alpha * (sample - this->baseline_mean_);
    const float deviation = sample - previous_mean;
    const float target_variance = deviation * deviation;
    this->baseline_variance_ += alpha * (target_variance - this->baseline_variance_);
  }

  bool update_persistence_(bool elevated) {
    if (this->persistence_valid_samples_ == kPersistenceWindowSize) {
      if (this->persistence_samples_[this->persistence_next_]) {
        this->persistence_elevated_count_--;
      }
    } else {
      this->persistence_valid_samples_++;
    }

    this->persistence_samples_[this->persistence_next_] = elevated;
    if (elevated) {
      this->persistence_elevated_count_++;
    }
    this->persistence_next_ = (this->persistence_next_ + 1) % kPersistenceWindowSize;
    return this->persistence_on();
  }

  void reset_persistence_() {
    for (bool &sample : this->persistence_samples_) {
      sample = false;
    }
    this->persistence_next_ = 0;
    this->persistence_valid_samples_ = 0;
    this->persistence_elevated_count_ = 0;
  }

  AdaptiveMotionDetectorResult result_(bool candidate, uint32_t now_ms) const {
    AdaptiveMotionDetectorResult result{};
    result.candidate = candidate;
    result.persistence_on = this->persistence_on();
    result.motion = this->motion_;
    result.baseline_mean = this->baseline_mean_;
    result.baseline_stddev = this->baseline_stddev();
    result.adaptive_threshold = this->adaptive_threshold_value_();
    return result;
  }

  float adaptive_threshold_value_() const { return this->baseline_stddev() * this->sigma_multiplier_; }

  bool adaptive_threshold_enabled_{true};
  float sigma_multiplier_{3.0f};
  uint32_t baseline_rise_time_ms_{600000};
  uint32_t baseline_fall_time_ms_{3600000};
  uint32_t learning_delay_ms_{60000};
  uint32_t debounce_samples_{2};
  uint32_t warmup_time_ms_{60000};
  uint32_t motion_hold_time_ms_{60000};

  bool initialized_{false};
  uint32_t started_ms_{0};
  uint32_t last_update_ms_{0};
  float baseline_mean_{0.0f};
  float baseline_variance_{0.0f};
  uint32_t debounce_count_{0};
  bool motion_{false};
  uint32_t last_motion_ms_{0};

  bool persistence_samples_[kPersistenceWindowSize]{};
  uint8_t persistence_next_{0};
  uint8_t persistence_valid_samples_{0};
  uint8_t persistence_elevated_count_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
