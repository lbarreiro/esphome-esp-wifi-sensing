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
  bool consume_ready_transition() {
    const bool ready_transition = this->ready_transition_;
    this->ready_transition_ = false;
    return ready_transition;
  }

  void process(const CsiPacket &input, CsiPacket &output);

 private:
  bool enabled_{false};
  int last_gain_status_{-1};
  bool ready_transition_reported_{false};
  bool ready_transition_{false};
  std::vector<int8_t> compensated_bytes_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
