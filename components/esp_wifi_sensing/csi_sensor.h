#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

class CsiSensor {
 public:
  void publish(uint32_t metric, uint16_t len) const {}
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
