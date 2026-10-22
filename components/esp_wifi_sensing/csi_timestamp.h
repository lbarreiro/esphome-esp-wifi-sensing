#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

// CSI callback and ESPHome loop run on different tasks. A snapshot timestamp
// can be a few milliseconds ahead of the loop's 'now'. Do not mistake that
// for a 2^32-ms stall. Legitimate millis() rollover remains supported.
inline uint32_t csi_elapsed_or_zero(uint32_t now, uint32_t timestamp) {
  const uint32_t elapsed = now - timestamp;
  return elapsed <= UINT32_MAX / 2U ? elapsed : 0U;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
