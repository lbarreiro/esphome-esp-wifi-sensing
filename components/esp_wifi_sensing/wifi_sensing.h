#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esp_wifi.h"
#include "esp_wifi_sensing.h"

namespace esphome {
namespace esp_wifi_sensing {

class ESPWiFiSensing : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  // FSM oficial da Espressif
  esp_wifi_sensing_fsm_handle_t fsm_{nullptr};

  // BSSID do router/AP ao qual o ESP32-C6 está ligado
  uint8_t peer_mac_[6]{0};

  // Só arrancamos o sensing depois de o Wi-Fi estar realmente ligado
  bool sensing_started_{false};

  // Arranca o motor esp_wifi_sensing
  bool start_sensing_();

  // Obtém o BSSID do AP atual
  bool get_router_bssid_();

  // Para e liberta o FSM
  void stop_sensing_();
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
