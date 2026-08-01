#pragma once

#include "esp_wifi_sensing/models/csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class EmptyFeature {
 public:
  uint32_t extract(const CsiPacket &) const { return 0; }
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
