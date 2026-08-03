#pragma once

#include <cstdint>

#include "csi_parser.h"

namespace esphome {
namespace esp_wifi_sensing {

class AbsoluteSumAlgorithm {
 public:
  uint32_t process(const ParsedCsiPacket &packet) const;
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
