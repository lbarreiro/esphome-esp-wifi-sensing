#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct CsiPacket {
  uint16_t len{0};
  const int8_t *raw_bytes{nullptr};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
