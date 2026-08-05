#pragma once

#include <cstdint>

#include "csi_packet.h"
#include "gain_compensation_preprocessor.h"
#include "csi_parser.h"

namespace esphome {
namespace esp_wifi_sensing {

class CsiPipeline {
 public:
  CsiPipeline();

  void process_packet(const CsiPacket &packet);
  uint16_t latest_len() const { return latest_len_; }
  const CsiPacket &latest_packet() const { return latest_packet_; }
  const ParsedCsiPacket &latest_parsed_packet() const { return csi_parser_.latest_packet(); }
  void set_gain_compensation_enabled(bool enabled) {
    gain_compensation_preprocessor_.set_enabled(enabled);
  }
  bool gain_compensation_ready() const { return gain_compensation_preprocessor_.is_ready(); }
  bool consume_gain_compensation_ready_transition() { return gain_compensation_preprocessor_.consume_ready_transition(); }
  bool has_new_sample() const { return has_new_sample_; }
  void clear_new_sample() { has_new_sample_ = false; }

 private:
  GainCompensationPreprocessor gain_compensation_preprocessor_{};
  CsiParser csi_parser_{};
  CsiPacket latest_packet_{};
  uint16_t latest_len_{0};
  bool has_new_sample_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
