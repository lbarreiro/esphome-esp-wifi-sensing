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

namespace esphome {
namespace esp_wifi_sensing {

class ESPWiFiSensing : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

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

  // Estatísticas para o relatório.
  uint32_t variation_sum_{0};
  uint32_t variation_max_{0};
  uint32_t variation_samples_{0};

  uint32_t last_reported_count_{0};
  uint32_t last_report_time_{0};

  bool csi_started_{false};
  bool ping_started_{false};

  esp_ping_handle_t ping_handle_{nullptr};

  EspIdfCsiDriver driver_{};
  CsiPipeline pipeline_{};
  ThresholdAlgorithm algorithm_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
