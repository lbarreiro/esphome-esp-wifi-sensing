#include "wifi_sensing.h"

#include <cstring>

#include "esp_err.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
}


void ESPWiFiSensing::loop() {
  // TESTE 2:
  // Esperamos até o Wi-Fi estar ligado e conseguirmos obter
  // o BSSID real do router/AP.
  //
  // Neste teste NÃO iniciamos a FSM da Espressif.

  if (!this->sensing_started_) {
    if (this->get_router_bssid_()) {
      ESP_LOGI(
          TAG,
          "TEST 2 OK - Router BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
          this->peer_mac_[0],
          this->peer_mac_[1],
          this->peer_mac_[2],
          this->peer_mac_[3],
          this->peer_mac_[4],
          this->peer_mac_[5]
      );

      // Usamos esta flag apenas para executar este teste uma vez.
      // A FSM NÃO é iniciada.
      this->sensing_started_ = true;
    }
  }
}


bool ESPWiFiSensing::get_router_bssid_() {
  wifi_ap_record_t ap_info{};

  esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);

  if (err != ESP_OK) {
    return false;
  }

  memcpy(this->peer_mac_, ap_info.bssid, 6);

  return true;
}


bool ESPWiFiSensing::start_sensing_() {
  ESP_LOGI(
      TAG,
      "Router BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
      this->peer_mac_[0],
      this->peer_mac_[1],
      this->peer_mac_[2],
      this->peer_mac_[3],
      this->peer_mac_[4],
      this->peer_mac_[5]
  );

  // Configuração oficial por defeito da Espressif.
  esp_wifi_sensing_fsm_config_t config =
      DEFAULT_ESP_WIFI_SENSING_FSM_CONFIG();

  esp_err_t err =
      esp_wifi_sensing_fsm_create(&config, &this->fsm_);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "FSM create failed: %s",
        esp_err_to_name(err)
    );
    return false;
  }

  err =
      esp_wifi_sensing_fsm_add_channel(
          this->fsm_,
          this->peer_mac_
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "Add channel failed: %s",
        esp_err_to_name(err)
    );

    esp_wifi_sensing_fsm_delete(this->fsm_);
    this->fsm_ = nullptr;

    return false;
  }

  err =
      esp_wifi_sensing_fsm_control(
          this->fsm_,
          ESP_WIFI_SENSING_FSM_CTRL_START,
          nullptr
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "FSM start failed: %s",
        esp_err_to_name(err)
    );

    esp_wifi_sensing_fsm_delete(this->fsm_);
    this->fsm_ = nullptr;

    return false;
  }

  // No modo router, a própria biblioteca mantém tráfego
  // de ping para o gateway para produzir amostras CSI.
  err =
      esp_wifi_sensing_fsm_ping_router_start(
          this->fsm_
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "Router ping start failed: %s",
        esp_err_to_name(err)
    );

    esp_wifi_sensing_fsm_control(
        this->fsm_,
        ESP_WIFI_SENSING_FSM_CTRL_STOP,
        nullptr
    );

    esp_wifi_sensing_fsm_delete(this->fsm_);
    this->fsm_ = nullptr;

    return false;
  }

  ESP_LOGI(TAG, "ESP Wi-Fi Sensing started successfully");

  return true;
}


void ESPWiFiSensing::stop_sensing_() {
  if (this->fsm_ == nullptr) {
    return;
  }

  esp_wifi_sensing_fsm_ping_router_stop(this->fsm_);

  esp_wifi_sensing_fsm_control(
      this->fsm_,
      ESP_WIFI_SENSING_FSM_CTRL_STOP,
      nullptr
  );

  esp_wifi_sensing_fsm_delete(this->fsm_);

  this->fsm_ = nullptr;
  this->sensing_started_ = false;
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(
      TAG,
      "  Status: %s",
      this->sensing_started_ ? "RUNNING" : "WAITING"
  );

  if (this->sensing_started_) {
    ESP_LOGCONFIG(
        TAG,
        "  Router BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
        this->peer_mac_[0],
        this->peer_mac_[1],
        this->peer_mac_[2],
        this->peer_mac_[3],
        this->peer_mac_[4],
        this->peer_mac_[5]
    );
  }
}

}  // namespace esp_wifi_sensing
}  // namespace esphome}  // namespace esphome
