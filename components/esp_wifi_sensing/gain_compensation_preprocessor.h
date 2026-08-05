#pragma once

#include <cstdint>
#include <vector>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class GainCompensationPreprocessor {
 public:
  void set_enabled(bool enabled);
  bool is_enabled() const { return enabled_; }
  bool is_ready() const { return this->ready_; }

  void process(const CsiPacket &input, CsiPacket &output);

 private:
  bool enabled_{false};
  bool ready_{true};
  std::vector<int8_t> compensated_bytes_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
