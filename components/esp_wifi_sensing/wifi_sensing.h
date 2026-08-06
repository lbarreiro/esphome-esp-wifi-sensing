#pragma once

#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include "esp_wifi.h"
#include "ping/ping_sock.h"

#include "espidf_csi_driver.h"
#include "csi_pipeline.h"
#include "csi_packet.h"
#include "absolute_sum_algorithm.h"
#include "variance_algorithm.h"
#include "adaptive_baseline.h"

namespace esphome {
namespace esp_wifi_sensing {

enum class CsiAlgorithm {
  ABSOLUTE_SUM,
  VARIANCE,
};

class ESPWiFiSensing : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_gain_compensation_enabled(bool enabled);
  void set_algorithm(CsiAlgorithm algorithm) { this->selected_algorithm_ = algorithm; }
  void set_metric_sensor(sensor::Sensor *sensor) { this->metric_sensor_ = sensor; }
  void set_variation_avg_sensor(sensor::Sensor *sensor) { this->variation_avg_sensor_ = sensor; }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *sensor) { this->motion_binary_sensor_ = sensor; }
  void set_motion_threshold(uint32_t threshold) { this->motion_threshold_ = threshold; }
  void set_motion_debounce(uint32_t debounce) { this->motion_debounce_ = debounce; }
  void set_adaptive_threshold_enabled(bool enabled) { this->adaptive_threshold_enabled_ = enabled; }
  void set_sigma_multiplier(float multiplier) { this->adaptive_baseline_.set_sigma_multiplier(multiplier); this->sigma_multiplier_ = multiplier; }
  void set_baseline_rise_time(uint32_t time_ms) { this->adaptive_baseline_.set_baseline_rise_time(time_ms); this->baseline_rise_time_ms_ = time_ms; }
  void set_baseline_fall_time(uint32_t time_ms) { this->adaptive_baseline_.set_baseline_fall_time(time_ms); this->baseline_fall_time_ms_ = time_ms; }
  void set_learning_delay(uint32_t delay_ms) { this->adaptive_baseline_.set_learning_delay(delay_ms); this->learning_delay_ms_ = delay_ms; }
  void set_warmup_time(uint32_t time_ms) { this->warmup_time_ms_ = time_ms; }
  void set_motion_hold_time(uint32_t time_ms) { this->motion_hold_time_ms_ = time_ms; }
  void set_baseline_mean_sensor(sensor::Sensor *sensor) { this->baseline_mean_sensor_ = sensor; }
  void set_baseline_stddev_sensor(sensor::Sensor *sensor) { this->baseline_stddev_sensor_ = sensor; }
  void set_adaptive_threshold_sensor(sensor::Sensor *sensor) { this->adaptive_threshold_sensor_ = sensor; }

 protected:
  static void csi_callback_(
      void *ctx,
      wifi_csi_info_t *data
  );

  bool start_csi_();
  bool start_ping_();
  bool warmup_active_(uint32_t now) const;
  bool apply_motion_hold_(bool motion_detected, uint32_t now);

  // CSI
  volatile uint32_t csi_packet_count_{0};

  // Métrica calculada a partir do buffer CSI.
  volatile uint32_t latest_csi_metric_{0};
  volatile uint16_t latest_csi_len_{0};
  volatile bool new_csi_sample_{false};

  // Comparação entre amostras.
  uint32_t previous_csi_metric_{0};
  bool have_previous_sample_{false};

  // Estatísticas para o relatório.
  uint32_t variation_sum_{0};
  uint32_t variation_max_{0};
  uint32_t variation_samples_{0};

  uint32_t last_reported_count_{0};
  uint32_t last_report_time_{0};

  bool csi_started_{false};
  bool ping_started_{false};
  bool gain_compensation_enabled_{false};
  CsiAlgorithm selected_algorithm_{CsiAlgorithm::ABSOLUTE_SUM};

  sensor::Sensor *metric_sensor_{nullptr};
  sensor::Sensor *variation_avg_sensor_{nullptr};
  sensor::Sensor *baseline_mean_sensor_{nullptr};
  sensor::Sensor *baseline_stddev_sensor_{nullptr};
  sensor::Sensor *adaptive_threshold_sensor_{nullptr};
  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};

  bool adaptive_threshold_enabled_{true};
  float sigma_multiplier_{4.0f};
  uint32_t baseline_rise_time_ms_{1800000};
  uint32_t baseline_fall_time_ms_{1800000};
  uint32_t learning_delay_ms_{60000};
  uint32_t warmup_time_ms_{0};
  bool warmup_complete_logged_{false};
  AdaptiveBaseline adaptive_baseline_{};
  uint32_t motion_threshold_{6000};
  uint32_t motion_debounce_{2};
  uint32_t consecutive_above_threshold_{0};
  bool motion_state_{false};
  uint32_t motion_hold_time_ms_{0};
  uint32_t last_motion_time_{0};

  esp_ping_handle_t ping_handle_{nullptr};

  EspIdfCsiDriver driver_{};
  CsiPipeline pipeline_{};
  AbsoluteSumAlgorithm absolute_sum_algorithm_{};
  VarianceAlgorithm variance_algorithm_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
