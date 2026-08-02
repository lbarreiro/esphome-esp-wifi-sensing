#pragma once

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class GainCompensationPreprocessor {
 public:
  void process(const CsiPacket &input, CsiPacket &output) const {
    output = input;
  }
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
