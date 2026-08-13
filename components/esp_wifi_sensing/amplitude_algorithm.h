#pragma once

#include <cmath>
#include <cstdint>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class CsiAmplitudeAlgorithm {
 public:
  // Keep a stable 0..10000 scale without normalizing each packet by its own
  // total amplitude. This preserves common-mode amplitude changes.
  static constexpr uint16_t kNormalization = 10000;
  static constexpr uint16_t kMaxSubcarriers = 128;
  static constexpr uint16_t kMaxMagnitude = 181;  // sqrt(127^2 + 127^2)

  uint32_t process(const CsiPacket &packet) {
    if (packet.raw_bytes == nullptr || packet.len < 2) {
      return 0;
    }

    const uint16_t subcarriers = static_cast<uint16_t>(packet.len / 2);
    const uint16_t count = subcarriers > kMaxSubcarriers ? kMaxSubcarriers : subcarriers;

    uint16_t amplitudes[kMaxSubcarriers]{};
    for (uint16_t i = 0; i < count; i++) {
      const int32_t real = static_cast<int32_t>(packet.raw_bytes[i * 2]);
      const int32_t imag = static_cast<int32_t>(packet.raw_bytes[i * 2 + 1]);
      const uint32_t magnitude_squared = static_cast<uint32_t>(real * real + imag * imag);
      amplitudes[i] = static_cast<uint16_t>(std::sqrt(static_cast<float>(magnitude_squared)));
    }

    if (!this->has_previous_) {
      for (uint16_t i = 0; i < count; i++) {
        this->previous_[i] = amplitudes[i];
      }
      this->previous_count_ = count;
      this->has_previous_ = true;
      return 0;
    }

    const uint16_t compare_count = count < this->previous_count_ ? count : this->previous_count_;
    uint32_t absolute_difference_sum = 0;

    for (uint16_t i = 0; i < compare_count; i++) {
      const uint16_t a = amplitudes[i];
      const uint16_t b = this->previous_[i];
      absolute_difference_sum += a >= b ? a - b : b - a;
    }

    for (uint16_t i = compare_count; i < count; i++) {
      absolute_difference_sum += amplitudes[i];
    }
    for (uint16_t i = compare_count; i < this->previous_count_; i++) {
      absolute_difference_sum += this->previous_[i];
    }

    for (uint16_t i = 0; i < count; i++) {
      this->previous_[i] = amplitudes[i];
    }
    this->previous_count_ = count;

    if (count == 0) {
      return 0;
    }

    // Average absolute amplitude change per subcarrier, mapped to 0..10000
    // using the fixed int8 I/Q magnitude range. No packet-level normalization.
    const uint32_t average_difference = absolute_difference_sum / count;
    const uint32_t metric = (average_difference * kNormalization) / kMaxMagnitude;
    return metric > kNormalization ? kNormalization : metric;
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
