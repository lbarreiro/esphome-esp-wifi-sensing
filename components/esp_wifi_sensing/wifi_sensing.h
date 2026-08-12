#pragma once

#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esp_wifi.h"
#include "ping/ping_sock.h"

#include "espidf_csi_driver.h"
#include "csi_pipeline.h"
#include "csi_packet.h"
#include "threshold_algorithm.h"
#include "variance_algorithm.h"
#include "adaptive_motion_detector.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

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
  void set_adaptive_threshold_enabled(bool enabled) { this->motion_detector_.set_adaptive_threshold_enabled(enabled); }
  void set_sigma_multiplier(float multiplier) { this->motion_detector_.set_sigma_multiplier(multiplier); }
  void set_baseline_rise_time(uint32_t time_ms) { this->motion_detector_.set_baseline_rise_time_ms(time_ms); }
  void set_baseline_fall_time(uint32_t time_ms) { this->motion_detector_.set_baseline_fall_time_ms(time_ms); }
  void set_learning_delay(uint32_t time_ms) { this->motion_detector_.set_learning_delay_ms(time_ms); }
  void set_debounce(uint32_t samples) { this->motion_detector_.set_debounce_samples(samples); }
  void set_warmup_time(uint32_t time_ms) { this->motion_detector_.set_warmup_time_ms(time_ms); }
  void set_motion_hold_time(uint32_t time_ms) { this->motion_detector_.set_motion_hold_time_ms(time_ms); }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *sensor) { this->motion_binary_sensor_ = sensor; }
  void set_temporal_persistence_binary_sensor(binary_sensor::BinarySensor *sensor) {
    this->temporal_persistence_binary_sensor_ = sensor;
  }
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

  // CSI
  volatile uint32_t csi_packet_count_{0};

  // Métrica calculada a partir do buffer CSI.
  volatile uint32_t latest_csi_metric_{0};
  volatile uint16_t latest_csi_len_{0};
  volatile bool new_csi_sample_{false};

  // Comparação entre amostras.
  uint32_t previous_csi_metric_{0};
  bool have_previous_sample_{false};

  // Diagnostic per-sample logging state.
  uint32_t diagnostic_previous_csi_metric_{0};
  bool diagnostic_have_previous_sample_{false};

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

  esp_ping_handle_t ping_handle_{nullptr};

  EspIdfCsiDriver driver_{};
  CsiPipeline pipeline_{};
  ThresholdAlgorithm algorithm_{};
  VarianceAlgorithm variance_algorithm_{};
  AdaptiveMotionDetector motion_detector_{};

  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *temporal_persistence_binary_sensor_{nullptr};
  sensor::Sensor *baseline_mean_sensor_{nullptr};
  sensor::Sensor *baseline_stddev_sensor_{nullptr};
  sensor::Sensor *adaptive_threshold_sensor_{nullptr};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
