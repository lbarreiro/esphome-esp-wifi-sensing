#include "absolute_sum_algorithm.h"

namespace esphome {
namespace esp_wifi_sensing {

uint32_t AbsoluteSumAlgorithm::process(const ParsedCsiPacket &packet) const {
  uint32_t metric = 0;

  for (size_t i = 0; i < packet.count; i++) {
    metric += packet.subcarriers[i].power;
  }

  return metric;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
