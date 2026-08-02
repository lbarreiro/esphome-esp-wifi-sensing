#pragma once

#include <cstdint>

#include "csi_packet.h"
#include "gain_compensation_preprocessor.h"

namespace esphome {
namespace esp_wifi_sensing {

class CsiPipeline {
 public:
  CsiPipeline();

  void process_packet(const CsiPacket &packet);
  uint16_t latest_len() const { return latest_len_; }
  const CsiPacket &latest_packet() const { return latest_packet_; }
  void set_gain_compensation_enabled(bool enabled) {
    gain_compensation_preprocessor_.set_enabled(enabled);
  }
  bool has_new_sample() const { return has_new_sample_; }
  void clear_new_sample() { has_new_sample_ = false; }

 private:
  GainCompensationPreprocessor gain_compensation_preprocessor_{};
  CsiPacket latest_packet_{};
  uint16_t latest_len_{0};
  bool has_new_sample_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
