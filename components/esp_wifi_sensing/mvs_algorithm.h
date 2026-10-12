#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "csi_parser.h"

namespace esphome {
namespace esp_wifi_sensing {

struct MvsResult {
  bool ready{false};
  bool motion{false};
  float score{0.0f};
  bool observation_added{false};
  size_t observation_count{0};
};

class MvsAlgorithm {
 public:
  void set_threshold(float threshold) { threshold_ = threshold; }
  void set_hold_time(uint32_t hold_ms) { hold_time_ms_ = hold_ms < kMinimumHoldMs ? kMinimumHoldMs : hold_ms; }
  MvsResult process(const ParsedCsiPacket &packet, uint32_t now_ms);
  bool motion_state() const { return motion_state_; }
  size_t observation_count() const { return total_observations_; }

  static constexpr size_t kBins = 48;
  static constexpr size_t kFilterWindow = 8;
  static constexpr uint32_t kUpdateIntervalMs = 100;
  static constexpr uint32_t kMinimumHoldMs = 120000;

 private:
  bool make_frame_(const ParsedCsiPacket &packet, float *frame) const;
  bool make_observation_(const float *frame, uint32_t now_ms, float *observation);
  float temporal_jitter_(const float *observation);
  bool update_filter_(bool above);
  void clear_filter_();

  float accumulator_[kBins]{};
  uint16_t accumulator_count_{0};
  bool timing_started_{false};
  uint32_t last_observation_ms_{0};
  size_t total_observations_{0};

  float previous_profile_[kBins]{};
  bool have_previous_profile_{false};
  float quiet_jitter_{0.02f};
  uint32_t quiet_samples_{0};

  bool filter_[kFilterWindow]{};
  size_t filter_next_{0};
  uint8_t filter_hits_{0};

  bool motion_state_{false};
  uint32_t last_motion_time_{0};
  float threshold_{9.5f};
  uint32_t hold_time_ms_{kMinimumHoldMs};
  float last_score_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
