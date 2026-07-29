#include "wifi_sensing.h"

#include <cstring>

#include "esp_err.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "TEST 3 - FSM create only");
}


void ESPWiFiSensing::loop() {
  // TESTE 3:
  //
  // 1. Esperamos pelo Wi-Fi.
  // 2. Obtemos o BSSID do router.
  // 3. Criamos APENAS a FSM.
  //
  // NÃO fazemos:
  // - esp_wifi_sensing_fsm_add_channel()
  // - esp_wifi_sensing_fsm_control(START)
  // - esp_wifi_sensing_fsm_ping_router_start()

  static bool test_attempted = false;

  if (test_attempted) {
    return;
  }

  if (!this->get_router_bssid_()) {
    return;
  }

  ESP_LOGI(
      TAG,
      "TEST 3 - Router BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
      this->peer_mac_[0],
      this->peer_mac_[1],
      this->peer_mac_[2],
      this->peer_mac_[3],
      this->peer_mac_[4],
      this->peer_mac_[5]
  );

  // Impede nova tentativa mesmo que a criação falhe.
  test_attempted = true;

  ESP_LOGI(TAG, "TEST 3 - Creating FSM...");

  esp_wifi_sensing_fsm_config_t config =
      DEFAULT_ESP_WIFI_SENSING_FSM_CONFIG();

  esp_err_t err =
      esp_wifi_sensing_fsm_create(
          &config,
          &this->fsm_
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "TEST 3 FAILED - FSM create: %s",
        esp_err_to_name(err)
    );

    this->fsm_ = nullptr;
    return;
  }

  ESP_LOGI(
      TAG,
      "TEST 3 OK - FSM created successfully"
  );

  // IMPORTANTE:
  // Paramamos aqui.
  //
  // Não adicionamos channel.
  // Não arrancamos a FSM.
  // Não iniciamos ping.
}


bool ESPWiFiSensing::get_router_bssid_() {
  wifi_ap_record_t ap_info{};

  esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);

  if (err != ESP_OK) {
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
  // Não utilizado no TESTE 3.
  return false;
}


void ESPWiFiSensing::stop_sensing_() {
  // No TESTE 3 não fazemos cleanup automático.
  //
  // Queremos manter exatamente o estado resultante
  // de esp_wifi_sensing_fsm_create() para observar
  // se o dispositivo permanece estável.
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  Debug stage: TEST 3");
  ESP_LOGCONFIG(TAG, "  Operation: FSM CREATE ONLY");
  ESP_LOGCONFIG(TAG, "  add_channel: DISABLED");
  ESP_LOGCONFIG(TAG, "  FSM START: DISABLED");
  ESP_LOGCONFIG(TAG, "  router ping: DISABLED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
