#include "wifi_sensing.h"

#include <cstring>

#include "esp_err.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "TEST 3A - Default FSM config only");
}


void ESPWiFiSensing::loop() {
  // TESTE 3A:
  //
  // 1. Esperamos pelo Wi-Fi.
  // 2. Obtemos o BSSID.
  // 3. Criamos APENAS a estrutura de configuração default.
  //
  // NÃO chamamos esp_wifi_sensing_fsm_create().

  static bool test_attempted = false;

  if (test_attempted) {
    return;
  }

  if (!this->get_router_bssid_()) {
    return;
  }

  test_attempted = true;

  ESP_LOGI(
      TAG,
      "TEST 3A - Router BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
      this->peer_mac_[0],
      this->peer_mac_[1],
      this->peer_mac_[2],
      this->peer_mac_[3],
      this->peer_mac_[4],
      this->peer_mac_[5]
  );

  ESP_LOGI(TAG, "TEST 3A - Creating default FSM config...");

  esp_wifi_sensing_fsm_config_t config =
      DEFAULT_ESP_WIFI_SENSING_FSM_CONFIG();

  // Usamos a variável para evitar que o compilador simplesmente
  // elimine a operação por não ser utilizada.
  volatile size_t config_size = sizeof(config);
  (void) config_size;

  ESP_LOGI(
      TAG,
      "TEST 3A OK - Default FSM config created (%u bytes)",
      static_cast<unsigned>(sizeof(config))
  );

  ESP_LOGI(
      TAG,
      "TEST 3A - esp_wifi_sensing_fsm_create() NOT CALLED"
  );
}


bool ESPWiFiSensing::get_router_bssid_() {
  wifi_ap_record_t ap_info{};

  esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);

  if (err != ESP_OK) {
    ESP_LOGW(
        TAG,
        "esp_wifi_sta_get_ap_info() failed: %s",
        esp_err_to_name(err)
    );
    return false;
  }

  memcpy(
      this->peer_mac_,
      ap_info.bssid,
      6
  );

  return true;
}


bool ESPWiFiSensing::start_sensing_() {
  // Não utilizado no TESTE 3A.
  return false;
}


void ESPWiFiSensing::stop_sensing_() {
  // Nada para libertar no TESTE 3A.
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  Debug stage: TEST 3A");
  ESP_LOGCONFIG(TAG, "  Default FSM config: ENABLED");
  ESP_LOGCONFIG(TAG, "  fsm_create: DISABLED");
  ESP_LOGCONFIG(TAG, "  add_channel: DISABLED");
  ESP_LOGCONFIG(TAG, "  FSM START: DISABLED");
  ESP_LOGCONFIG(TAG, "  router ping: DISABLED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome