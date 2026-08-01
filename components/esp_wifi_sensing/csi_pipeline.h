#pragma once

#include <cstdint>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class CsiPipeline {
 public:
  CsiPipeline();

  void process_packet(const CsiPacket &packet);
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
