#pragma once

#include <cstdint>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

// Normalized temporal CSI jitter.
// Measures the L1 distance between consecutive CSI vectors and normalizes it
// by their combined absolute energy. The result is scaled to 0..10000.
class JitterAlgorithm {
 public:
  static constexpr uint16_t kMaxCsiBytes = 512;
  static constexpr uint32_t kScale = 10000;

  uint32_t process(const CsiPacket &packet) {
    if (packet.raw_bytes == nullptr || packet.len == 0) {
      this->has_previous_ = false;
      this->previous_len_ = 0;
      return 0;
    }

    const uint16_t len = packet.len > kMaxCsiBytes ? kMaxCsiBytes : packet.len;

    if (!this->has_previous_ || this->previous_len_ != len) {
      for (uint16_t i = 0; i < len; i++) {
        this->previous_bytes_[i] = packet.raw_bytes[i];
      }
      this->previous_len_ = len;
      this->has_previous_ = true;
      return 0;
    }

    uint64_t difference_sum = 0;
    uint64_t energy_sum = 0;

    for (uint16_t i = 0; i < len; i++) {
      const int32_t current = static_cast<int32_t>(packet.raw_bytes[i]);
      const int32_t previous = static_cast<int32_t>(this->previous_bytes_[i]);
      const int32_t difference = current - previous;

      difference_sum += static_cast<uint32_t>(difference < 0 ? -difference : difference);
      energy_sum += static_cast<uint32_t>(current < 0 ? -current : current);
      energy_sum += static_cast<uint32_t>(previous < 0 ? -previous : previous);

      this->previous_bytes_[i] = packet.raw_bytes[i];
    }

    if (energy_sum == 0) {
      return 0;
    }

    return static_cast<uint32_t>((difference_sum * kScale) / energy_sum);
  }

  void reset() {
    this->has_previous_ = false;
    this->previous_len_ = 0;
  }

 protected:
  int8_t previous_bytes_[kMaxCsiBytes]{};
  uint16_t previous_len_{0};
  bool has_previous_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
