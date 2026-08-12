#pragma once

#include <cmath>
#include <cstdint>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class CsiAmplitudeAlgorithm {
 public:
  static constexpr uint16_t kNormalization = 10000;
  static constexpr uint16_t kMaxSubcarriers = 128;

  uint32_t process(const CsiPacket &packet) {
    if (packet.raw_bytes == nullptr || packet.len < 2) {
      return 0;
    }

    const uint16_t subcarriers = static_cast<uint16_t>(packet.len / 2);
    const uint16_t count = subcarriers > kMaxSubcarriers ? kMaxSubcarriers : subcarriers;

    uint32_t amplitude_sum = 0;
    uint16_t amplitudes[kMaxSubcarriers]{};

    for (uint16_t i = 0; i < count; i++) {
      const int32_t real = static_cast<int32_t>(packet.raw_bytes[i * 2]);
      const int32_t imag = static_cast<int32_t>(packet.raw_bytes[i * 2 + 1]);
      const uint32_t magnitude_squared = static_cast<uint32_t>(real * real + imag * imag);
      const uint16_t magnitude = static_cast<uint16_t>(std::sqrt(static_cast<float>(magnitude_squared)));
      amplitudes[i] = magnitude;
      amplitude_sum += magnitude;
    }

    if (amplitude_sum == 0) {
      this->reset();
      return 0;
    }

    uint16_t normalized[kMaxSubcarriers]{};
    for (uint16_t i = 0; i < count; i++) {
      normalized[i] = static_cast<uint16_t>(
          (static_cast<uint32_t>(amplitudes[i]) * kNormalization) / amplitude_sum);
    }

    if (!this->has_previous_) {
      for (uint16_t i = 0; i < count; i++) {
        this->previous_[i] = normalized[i];
      }
      this->previous_count_ = count;
      this->has_previous_ = true;
      return 0;
    }

    const uint16_t compare_count = count < this->previous_count_ ? count : this->previous_count_;
    uint32_t l1 = 0;
    for (uint16_t i = 0; i < compare_count; i++) {
      const uint16_t a = normalized[i];
      const uint16_t b = this->previous_[i];
      l1 += a >= b ? a - b : b - a;
    }

    for (uint16_t i = compare_count; i < count; i++) {
      l1 += normalized[i];
    }
    for (uint16_t i = compare_count; i < this->previous_count_; i++) {
      l1 += this->previous_[i];
    }

    for (uint16_t i = 0; i < count; i++) {
      this->previous_[i] = normalized[i];
    }
    this->previous_count_ = count;

    if (l1 >= 20000) {
      return 10000;
    }
    return l1 / 2;
  }

  void reset() {
    this->has_previous_ = false;
    this->previous_count_ = 0;
  }

 protected:
  uint16_t previous_[kMaxSubcarriers]{};
  uint16_t previous_count_{0};
  bool has_previous_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
