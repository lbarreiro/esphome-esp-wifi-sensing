#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

class ThresholdAlgorithm {
 public:
  uint32_t process(uint32_t metric) const { return metric; }
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
