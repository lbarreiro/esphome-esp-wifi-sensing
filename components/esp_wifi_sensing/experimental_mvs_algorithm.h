#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "csi_parser.h"
#include "mvs_algorithm.h"
#include "csi_learning.h"
#include "mvs3_priority.h"

namespace esphome {
namespace esp_wifi_sensing {

class ExperimentalMvsAlgorithm {
 public:
  void set_threshold(float threshold) { threshold_ = threshold; }
  void set_hold_time(uint32_t hold_ms) { hold_time_ms_ = hold_ms < kMinimumHoldMs ? kMinimumHoldMs : hold_ms; }
  MvsResult process(const ParsedCsiPacket &packet, uint32_t now_ms);
  bool motion_state() const { return motion_state_; }
  uint32_t last_motion_time() const { return last_motion_time_; }
  size_t observation_count() const { return total_observations_; }
  bool priority_active() const { return priority_.active(); }
  float priority_score() const { return priority_.score(); }
  uint32_t priority_confirmations() const { return priority_.confirmations(); }
  size_t priority_bin(size_t i) const { return priority_.top_bin(i); }

  static constexpr size_t kBins = 48;
  static constexpr size_t kMaxFilterWindow = 64;
  void set_filter_window(size_t count) { filter_window_ = count; clear_filter_(); priority_.set_filter_window(count); }
  void set_filter_count(uint8_t count) { filter_count_ = count; clear_filter_(); priority_.set_filter_count(count); }
  void set_update_interval(uint32_t ms) { update_interval_ms_ = ms; }
  void set_quiet_boot_samples(uint32_t count) { quiet_boot_samples_ = count; }
  void set_quiet_alpha(float alpha) { quiet_alpha_ = alpha; }
  void set_quiet_update_limit(float limit) { quiet_update_limit_ = limit; }
  static constexpr uint32_t kMinimumHoldMs = 120000;

  const CsiLearning &learning() const { return learning_; }
 private:
  CsiLearning learning_{};
  Mvs3Priority priority_{};
  float raw_energy_[kBins]{};
  float selected_energy_{0};
  uint32_t last_packet_ms_{0};
  bool packet_seen_{false};
  size_t last_packet_count_{0};

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

  bool filter_[kMaxFilterWindow]{};
  size_t filter_window_{8};
  uint8_t filter_count_{3};
  uint32_t update_interval_ms_{100};
  uint32_t quiet_boot_samples_{20};
  float quiet_alpha_{0.01f};
  float quiet_update_limit_{3.0f};
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

