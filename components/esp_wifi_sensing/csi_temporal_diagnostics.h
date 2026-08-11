#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct CsiTemporalDiagnosticsSample {
  bool has_delta{false};
  uint32_t delta{0};
  float temporal_mean{0.0f};
  float temporal_persistence{0.0f};
};

class CsiTemporalDiagnostics {
 public:
  static constexpr uint8_t kWindowSize = 10;

  CsiTemporalDiagnosticsSample update(uint32_t metric) {
    CsiTemporalDiagnosticsSample sample{};

    if (!this->has_previous_metric_) {
      this->previous_metric_ = metric;
      this->has_previous_metric_ = true;
      return sample;
    }

    const uint32_t delta = metric >= this->previous_metric_ ? metric - this->previous_metric_ :
                                                            this->previous_metric_ - metric;
    this->previous_metric_ = metric;

    if (this->count_ == kWindowSize) {
      this->sum_ -= this->deltas_[this->next_];
    } else {
      this->count_++;
    }

    this->deltas_[this->next_] = delta;
    this->sum_ += delta;
    this->next_ = (this->next_ + 1) % kWindowSize;

    const float temporal_mean = static_cast<float>(this->sum_) / static_cast<float>(this->count_);
    uint8_t persistent_samples = 0;
    for (uint8_t i = 0; i < this->count_; i++) {
      if (static_cast<float>(this->deltas_[i]) > temporal_mean) {
        persistent_samples++;
      }
    }

    sample.has_delta = true;
    sample.delta = delta;
    sample.temporal_mean = temporal_mean;
    sample.temporal_persistence =
        (static_cast<float>(persistent_samples) * 100.0f) / static_cast<float>(this->count_);
    return sample;
  }

  uint8_t count() const { return this->count_; }

 protected:
  uint32_t previous_metric_{0};
  bool has_previous_metric_{false};
  uint32_t deltas_[kWindowSize]{};
  uint8_t next_{0};
  uint8_t count_{0};
  uint32_t sum_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
