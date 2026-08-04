#pragma once

#include <cstdint>

#include "esp_wifi.h"

namespace esphome {
namespace esp_wifi_sensing {

struct CsiPacket {
  uint16_t len{0};
  const int8_t *raw_bytes{nullptr};
  const wifi_pkt_rx_ctrl_t *rx_ctrl{nullptr};
  bool first_word_invalid{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
