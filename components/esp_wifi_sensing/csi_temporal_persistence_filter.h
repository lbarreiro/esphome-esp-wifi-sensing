#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

class CsiTemporalPersistenceFilter {
 public:
  static constexpr uint8_t kWindowSize = 10;
  static constexpr float kPersistenceThreshold = 0.70f;

  bool update_metric(uint32_t metric) {
    if (!this->has_previous_metric_) {
      this->previous_metric_ = metric;
      this->has_previous_metric_ = true;
      return false;
    }

    const uint32_t delta = metric >= this->previous_metric_ ? metric - this->previous_metric_ :
                                                            this->previous_metric_ - metric;
    this->previous_metric_ = metric;
    return this->update_delta(delta);
  }

  bool update_delta(uint32_t delta) {
    const bool elevated = delta != 0;

    if (this->valid_samples_ == kWindowSize) {
      if (this->elevated_samples_[this->next_]) {
        this->elevated_count_--;
      }
    } else {
      this->valid_samples_++;
    }

    this->elevated_samples_[this->next_] = elevated;
    if (elevated) {
      this->elevated_count_++;
    }
    this->next_ = (this->next_ + 1) % kWindowSize;

    return this->is_on();
  }

  bool is_on() const {
    return this->has_valid_window() && this->persistence() >= kPersistenceThreshold;
  }

  bool has_valid_window() const { return this->valid_samples_ == kWindowSize; }

  float persistence() const {
    if (this->valid_samples_ == 0) {
      return 0.0f;
    }
    return static_cast<float>(this->elevated_count_) / static_cast<float>(this->valid_samples_);
  }

  uint8_t elevated_count() const { return this->elevated_count_; }
  uint8_t valid_samples() const { return this->valid_samples_; }

 protected:
  uint32_t previous_metric_{0};
  bool has_previous_metric_{false};
  bool elevated_samples_[kWindowSize]{};
  uint8_t next_{0};
  uint8_t valid_samples_{0};
  uint8_t elevated_count_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
