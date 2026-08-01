#pragma once

#include <cstdint>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

class CsiPipeline {
 public:
  CsiPipeline();

  void process_packet(const CsiPacket &packet);
  uint16_t latest_len() const { return latest_len_; }
  bool has_new_sample() const { return has_new_sample_; }
  void clear_new_sample() { has_new_sample_ = false; }

 private:
  uint16_t latest_len_{0};
  bool has_new_sample_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
