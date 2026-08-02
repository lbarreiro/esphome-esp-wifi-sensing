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

  void process(const CsiPacket &input, CsiPacket &output);

 private:
  bool enabled_{false};
  int last_gain_status_{-1};
  uint32_t baseline_sample_count_{0};
  uint32_t compensated_packet_count_{0};
  uint32_t compensation_log_countdown_{0};
  std::vector<int8_t> compensated_bytes_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
