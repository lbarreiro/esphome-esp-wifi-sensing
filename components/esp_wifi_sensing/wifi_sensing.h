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
#include "csi_temporal_persistence_filter.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

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
  void set_temporal_persistence_binary_sensor(binary_sensor::BinarySensor *sensor) {
    this->temporal_persistence_binary_sensor_ = sensor;
  }

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
  CsiTemporalPersistenceFilter temporal_persistence_filter_{};

  binary_sensor::BinarySensor *temporal_persistence_binary_sensor_{nullptr};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
