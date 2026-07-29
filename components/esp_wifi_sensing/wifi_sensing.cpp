#include "wifi_sensing.h"

#include <cstring>

#include "esp_err.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "TEST 3B - Minimal FSM config + create");
}


void ESPWiFiSensing::loop() {
  // TESTE 3B
  //
  // Testamos esp_wifi_sensing_fsm_create() com uma
  // configuração muito mais pequena que a default.
  //
  // NÃO fazemos:
  // - add_channel
  // - FSM START
  // - ping_router_start

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
      "TEST 3B - Router BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
      this->peer_mac_[0],
      this->peer_mac_[1],
      this->peer_mac_[2],
      this->peer_mac_[3],
      this->peer_mac_[4],
      this->peer_mac_[5]
  );

  // Começamos pela configuração oficial para garantir
  // que todos os campos ficam corretamente inicializados.
  esp_wifi_sensing_fsm_config_t config =
      DEFAULT_ESP_WIFI_SENSING_FSM_CONFIG();

  // Defaults oficiais:
  //   max_channel_num  = 16
  //   raw_buf_size     = 20
  //   polling_interval = 20
  //
  // TESTE mínimo:
  config.max_channel_num = 1;
  config.raw_buf_size = 1;
  config.polling_interval = 1000;

  ESP_LOGI(
      TAG,
      "TEST 3B - Config: channels=%u raw_buf=%u polling=%u",
      static_cast<unsigned>(config.max_channel_num),
      static_cast<unsigned>(config.raw_buf_size),
      static_cast<unsigned>(config.polling_interval)
  );

  ESP_LOGI(TAG, "TEST 3B - Calling fsm_create NOW...");

  esp_err_t err =
      esp_wifi_sensing_fsm_create(
          &config,
          &this->fsm_
      );

  // Se o crash estiver dentro de fsm_create(),
  // esta linha nunca aparecerá.
  ESP_LOGI(
      TAG,
      "TEST 3B - fsm_create returned: %s (0x%X)",
      esp_err_to_name(err),
      static_cast<unsigned>(err)
  );

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "TEST 3B FAILED - FSM was not created");
    this->fsm_ = nullptr;
    return;
  }

  ESP_LOGI(TAG, "TEST 3B OK - FSM created successfully");

  // PARAMOS AQUI.
}


bool ESPWiFiSensing::get_router_bssid_() {
  wifi_ap_record_t ap_info{};

  esp_err_t err =
      esp_wifi_sta_get_ap_info(&ap_info);

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
  // Não utilizado no TESTE 3B.
  return false;
}


void ESPWiFiSensing::stop_sensing_() {
  // Não fazemos cleanup neste teste.
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  Debug stage: TEST 3B");
  ESP_LOGCONFIG(TAG, "  FSM config: MINIMAL");
  ESP_LOGCONFIG(TAG, "  max channels: 1");
  ESP_LOGCONFIG(TAG, "  raw buffer: 1");
  ESP_LOGCONFIG(TAG, "  polling: 1000");
  ESP_LOGCONFIG(TAG, "  add_channel: DISABLED");
  ESP_LOGCONFIG(TAG, "  FSM START: DISABLED");
  ESP_LOGCONFIG(TAG, "  router ping: DISABLED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
