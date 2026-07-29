#pragma once

#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esp_wifi.h"
#include "ping/ping_sock.h"

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

  volatile uint32_t csi_packet_count_{0};

  uint32_t last_reported_count_{0};
  uint32_t last_report_time_{0};

  bool csi_started_{false};
  bool ping_started_{false};

  esp_ping_handle_t ping_handle_{nullptr};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
