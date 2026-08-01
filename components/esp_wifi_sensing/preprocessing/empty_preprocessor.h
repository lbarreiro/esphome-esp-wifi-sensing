#pragma once

#include "../models/csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class EmptyPreprocessor {
 public:
  void process(const CsiPacket &input, CsiPacket &output) const {
    output = input;
  }
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
