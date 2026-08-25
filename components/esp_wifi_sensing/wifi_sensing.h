#pragma once

#include <cstdint>
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esp_wifi.h"
#include "ping/ping_sock.h"
#include "espidf_csi_driver.h"
#include "csi_pipeline.h"
#include "csi_packet.h"
#include "threshold_algorithm.h"
#include "variance_algorithm.h"
#include "jitter_algorithm.h"
#include "amplitude_algorithm.h"
#include "adaptive_motion_detector.h"
#include "esp_radar_motion_detector.h"
#include "mvs_motion_detector.h"

namespace esphome { namespace esp_wifi_sensing {

enum class CsiAlgorithm { ABSOLUTE_SUM, VARIANCE, AMPLITUDE, JITTER, ESP_RADAR, MVS };

class ESPWiFiSensing : public Component {
 public:
  void setup() override; void loop() override; void dump_config() override;
  void set_gain_compensation_enabled(bool enabled);
  void set_algorithm(CsiAlgorithm algorithm) { selected_algorithm_ = algorithm; }
  void set_adaptive_threshold_enabled(bool enabled) { motion_detector_.set_adaptive_threshold_enabled(enabled); }
  void set_fixed_threshold(uint32_t threshold) { motion_detector_.set_fixed_threshold(threshold); }
  void set_sigma_multiplier(float multiplier) { motion_detector_.set_sigma_multiplier(multiplier); }
  void set_baseline_rise_time(uint32_t v) { motion_detector_.set_baseline_rise_time_ms(v); }
  void set_baseline_fall_time(uint32_t v) { motion_detector_.set_baseline_fall_time_ms(v); }
  void set_learning_delay(uint32_t v) { motion_detector_.set_learning_delay_ms(v); }
  void set_debounce(uint32_t v) { motion_detector_.set_debounce_samples(v); }
  void set_warmup_time(uint32_t v) { motion_detector_.set_warmup_time_ms(v); }
  void set_motion_hold_time(uint32_t v) { motion_detector_.set_motion_hold_time_ms(v); }
  void set_persistence_samples(uint8_t v) { motion_detector_.set_persistence_samples(v); }
  void set_motion_sensitivity(float v) { esp_radar_detector_.set_sensitivity(v); }
  void set_active_jitter_min(float v) { esp_radar_detector_.set_active_jitter_min(v); }
  void set_active_filter_ms(uint32_t v) { esp_radar_detector_.set_active_filter_ms(v); }
  void set_enter_multiplier(float v) { esp_radar_detector_.set_enter_multiplier(v); }
  void set_mvs_window(uint16_t v) { mvs_detector_.set_window_samples(v); }
  void set_mvs_threshold_multiplier(float v) { mvs_detector_.set_threshold_multiplier(v); }
  void set_mvs_enter_hits(uint8_t v) { mvs_detector_.set_enter_hits(v); }
  void set_mvs_exit_hits(uint8_t v) { mvs_detector_.set_exit_hits(v); }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *v) { motion_binary_sensor_ = v; }
 protected:
  static void csi_callback_(void *ctx, wifi_csi_info_t *data);
  bool start_csi_(); bool start_ping_();
  volatile uint32_t csi_packet_count_{0}, latest_csi_metric_{0};
  volatile uint16_t latest_csi_len_{0}; volatile bool new_csi_sample_{false};
  uint32_t previous_csi_metric_{0}, diagnostic_previous_csi_metric_{0};
  bool have_previous_sample_{false}, diagnostic_have_previous_sample_{false};
  uint32_t diagnostic_last_log_time_{0}, variation_sum_{0}, variation_max_{0}, variation_samples_{0};
  uint32_t last_reported_count_{0}, last_report_time_{0};
  bool csi_started_{false}, ping_started_{false}, gain_compensation_enabled_{false};
  CsiAlgorithm selected_algorithm_{CsiAlgorithm::ABSOLUTE_SUM};
  esp_ping_handle_t ping_handle_{nullptr};
  EspIdfCsiDriver driver_{}; CsiPipeline pipeline_{}; ThresholdAlgorithm algorithm_{};
  VarianceAlgorithm variance_algorithm_{}; CsiAmplitudeAlgorithm amplitude_algorithm_{}; JitterAlgorithm jitter_algorithm_{};
  AdaptiveMotionDetector motion_detector_{}; EspRadarMotionDetector esp_radar_detector_{}; MvsMotionDetector mvs_detector_{};
  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};
};

} }
