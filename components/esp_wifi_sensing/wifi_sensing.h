#pragma once

#include <cstdint>
#include <atomic>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include "esp_wifi.h"
#include "ping/ping_sock.h"

#include "espidf_csi_driver.h"
#include "csi_pipeline.h"
#include "csi_packet.h"
#include "absolute_sum_algorithm.h"
#include "variance_algorithm.h"
#include "adaptive_baseline.h"
#include "mvs_algorithm.h"
#include "experimental_mvs_algorithm.h"

namespace esphome {
namespace esp_wifi_sensing {

enum class CsiAlgorithm {
  ABSOLUTE_SUM,
  VARIANCE,
  MVS,
};

class ESPWiFiSensing : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_gain_compensation_enabled(bool enabled);
  void set_algorithm(CsiAlgorithm algorithm) { this->selected_algorithm_ = algorithm; }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *sensor) { this->motion_binary_sensor_ = sensor; }
  void set_motion_threshold(float threshold) { this->motion_threshold_ = threshold; this->mvs_algorithm_.set_threshold(threshold); this->mvs_3_algorithm_.set_threshold(threshold); }
  void set_filter_window(uint32_t count) { this->mvs_algorithm_.set_filter_window(count); this->mvs_3_algorithm_.set_filter_window(count); }
  void set_filter_count(uint32_t count) { this->mvs_algorithm_.set_filter_count(static_cast<uint8_t>(count)); this->mvs_3_algorithm_.set_filter_count(static_cast<uint8_t>(count)); }
  void set_update_interval(uint32_t ms) { this->mvs_algorithm_.set_update_interval(ms); this->mvs_3_algorithm_.set_update_interval(ms); }
  void set_quiet_boot_samples(uint32_t count) { this->mvs_algorithm_.set_quiet_boot_samples(count); this->mvs_3_algorithm_.set_quiet_boot_samples(count); }
  void set_quiet_alpha(float alpha) { this->mvs_algorithm_.set_quiet_alpha(alpha); this->mvs_3_algorithm_.set_quiet_alpha(alpha); }
  void set_quiet_update_limit(float limit) { this->mvs_algorithm_.set_quiet_update_limit(limit); this->mvs_3_algorithm_.set_quiet_update_limit(limit); }
  void set_ping_interval(uint32_t ms) { this->ping_interval_ms_ = ms; }
  void set_motion_debounce(uint32_t debounce) { this->motion_debounce_ = debounce; }
  void set_adaptive_threshold_enabled(bool enabled) { this->adaptive_threshold_enabled_ = enabled; }
  void set_sigma_multiplier(float multiplier) { this->adaptive_baseline_.set_sigma_multiplier(multiplier); this->sigma_multiplier_ = multiplier; }
  void set_baseline_rise_time(uint32_t time_ms) { this->adaptive_baseline_.set_baseline_rise_time(time_ms); this->baseline_rise_time_ms_ = time_ms; }
  void set_baseline_fall_time(uint32_t time_ms) { this->adaptive_baseline_.set_baseline_fall_time(time_ms); this->baseline_fall_time_ms_ = time_ms; }
  void set_learning_delay(uint32_t delay_ms) { this->adaptive_baseline_.set_learning_delay(delay_ms); this->learning_delay_ms_ = delay_ms; }
  void set_warmup_time(uint32_t time_ms) { this->warmup_time_ms_ = time_ms; }
  void set_motion_hold_time(uint32_t time_ms) { this->motion_hold_time_ms_ = time_ms < MvsAlgorithm::kMinimumHoldMs ? MvsAlgorithm::kMinimumHoldMs : time_ms; this->mvs_algorithm_.set_hold_time(this->motion_hold_time_ms_); this->mvs_3_algorithm_.set_hold_time(this->motion_hold_time_ms_); }
  void set_statistics_window(uint32_t time_ms) { this->statistics_window_ms_ = time_ms; }
  void set_statistics_update(uint32_t time_ms) { this->statistics_update_ms_ = time_ms; }

  void set_motion_3_binary_sensor(binary_sensor::BinarySensor *sensor) { motion_3_binary_sensor_ = sensor; }
 protected:
  static void csi_callback_(
      void *ctx,
      wifi_csi_info_t *data
  );

  bool start_csi_();
  bool start_ping_();
  bool warmup_active_(uint32_t now) const;
  bool apply_motion_hold_(bool motion_detected, uint32_t now);
  void add_variation_sample_(uint32_t variation, uint32_t now);
  void prune_variation_samples_(uint32_t now);

  // CSI
  volatile uint32_t csi_packet_count_{0};
  std::atomic<uint32_t> csi_128_packet_count_{0};
  std::atomic<uint32_t> csi_256_packet_count_{0};
  uint32_t csi_length_last_log_ms_{0};

  // Métrica calculada a partir do buffer CSI.
  volatile uint32_t latest_csi_metric_{0};
  volatile uint16_t latest_csi_len_{0};
  volatile bool new_csi_sample_{false};

  // Comparação entre amostras.
  uint32_t previous_csi_metric_{0};
  bool have_previous_sample_{false};

  // Estatísticas para o relatório.
  struct VariationSample {
    uint32_t value{0};
    uint32_t timestamp{0};
  };

  std::vector<VariationSample> variation_window_{};
  uint32_t variation_window_next_{0};
  uint32_t variation_window_count_{0};
  uint32_t variation_sum_{0};
  uint32_t variation_max_{0};
  bool variation_max_dirty_{false};
  uint32_t statistics_window_ms_{5000};
  uint32_t statistics_update_ms_{1000};

  uint32_t last_reported_count_{0};
  uint32_t last_report_time_{0};

  bool csi_started_{false};
  bool ping_started_{false};
  bool gain_compensation_enabled_{false};
  CsiAlgorithm selected_algorithm_{CsiAlgorithm::ABSOLUTE_SUM};

  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};

  bool adaptive_threshold_enabled_{true};
  float sigma_multiplier_{4.0f};
  uint32_t baseline_rise_time_ms_{1800000};
  uint32_t baseline_fall_time_ms_{1800000};
  uint32_t learning_delay_ms_{60000};
  uint32_t warmup_time_ms_{0};
  bool warmup_complete_logged_{false};
  AdaptiveBaseline adaptive_baseline_{};
  float motion_threshold_{9.5f};
  uint32_t motion_debounce_{2};
  uint32_t consecutive_above_threshold_{0};
  bool motion_state_{false};
  uint32_t motion_hold_time_ms_{0};
  uint32_t last_motion_time_{0};

  // Cross-task MVS confirmation snapshot; updated by Wi-Fi CSI callback.
  std::atomic<uint32_t> mvs_last_confirmation_ms_{0};
  std::atomic<bool> mvs_confirmation_seen_{false};
  bool mvs_published_state_{false};
  bool mvs_initial_state_published_{false};
  uint32_t mvs_last_logged_confirmation_ms_{0};
  std::atomic<uint32_t> mvs_last_packet_ms_{0};
  std::atomic<uint32_t> mvs_last_accepted_ms_{0};
  std::atomic<uint32_t> mvs_rejected_packets_{0};
  uint32_t mvs_diagnostic_started_ms_{0};
  std::atomic<uint32_t> mvs_last_observation_ms_{0};
  uint32_t mvs_last_stall_log_ms_{0};
  uint32_t ping_interval_ms_{100};
  esp_ping_handle_t ping_handle_{nullptr};

  binary_sensor::BinarySensor *motion_3_binary_sensor_{nullptr};
  CsiParser csi_3_parser_{};
  ExperimentalMvsAlgorithm mvs_3_algorithm_{};
  std::atomic<uint32_t> mvs_3_last_confirmation_ms_{0};
  std::atomic<bool> mvs_3_confirmation_seen_{false};
  bool mvs_3_published_state_{false};
  bool mvs_3_initial_state_published_{false};
  // Scalar atomic snapshots: loop never reads callback-owned learner memory.
  std::atomic<uint32_t> learn_quiet_{0}, learn_episodes_{0}, learn_rejected_{0}, learn_revisions_{0};
  std::atomic<uint32_t> learn_best_{0}, learn_low_{0};
  std::atomic<uint32_t> learn_best_hits_{0}, learn_low_hits_{0};
  std::atomic<float> learn_best_candidate_{1}, learn_best_weight_{1};
  std::atomic<float> learn_low_candidate_{1}, learn_low_weight_{1};
  std::atomic<uint32_t> learn_discarded_{0}, learn_hold_resets_{0}, learn_gaps_{0};
  std::atomic<uint32_t> learn_good_{0}, learn_count_{0}, learn_observations_{0};
  std::atomic<float> learn_energy_{0}, learn_limit_{0}, learn_score_{0};
  // Compact learner diagnostics, snapshotted by CSI callback (never read
  // learner-owned arrays directly from the ESPHome loop).
  std::atomic<uint32_t> learn_window_{0}, learn_recurrent_{0}, learn_promoted_{0};
  std::atomic<uint32_t> learn_strong_{0}, learn_weak_{0};
  std::atomic<uint32_t> learn_last_over10x_{0}, learn_last_over256x_{0};
  std::atomic<uint32_t> learn_eligible_episodes_{0}, learn_skipped_episodes_{0};
  std::atomic<uint32_t> learn_prequiet_bins_{0}, learn_transition_bins_{0};
  std::atomic<float> learn_mean_candidate_{1.0f};
  // Experimental two-bin 70/30 decision, published only via scalar atomics.
  std::atomic<bool> priority_active_{false};
  std::atomic<float> priority_score_{0};
  std::atomic<uint32_t> priority_confirmations_{0};
  std::atomic<uint32_t> priority_bins_[2]{};
  std::atomic<uint32_t> priority_hits_[2]{};
  std::atomic<float> priority_candidates_[2]{}, priority_weights_[2]{};
  std::atomic<bool> learn_raw_quiet_{false}, learn_held_{false};
  uint32_t learn_last_log_ms_{0};
  EspIdfCsiDriver driver_{};
  CsiPipeline pipeline_{};
  AbsoluteSumAlgorithm absolute_sum_algorithm_{};
  VarianceAlgorithm variance_algorithm_{};
  MvsAlgorithm mvs_algorithm_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
