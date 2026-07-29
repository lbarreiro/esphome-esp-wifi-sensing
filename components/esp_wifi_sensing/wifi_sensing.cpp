#include "wifi_sensing.h"

#include "esp_err.h"
#include "esp_netif.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "STEP 4 - Native CSI + router ping");
}


void ESPWiFiSensing::loop() {
  // -------------------------------------------------------
  // 1. Ativar CSI depois de o Wi-Fi estar ligado
  // -------------------------------------------------------

  if (!this->csi_started_) {
    if (!this->start_csi_()) {
      delay(1000);
      return;
    }

    this->csi_started_ = true;

    ESP_LOGI(TAG, "STEP 4 - CSI enabled");
  }

  // -------------------------------------------------------
  // 2. Criar/iniciar ping ao gateway
  // -------------------------------------------------------

  if (!this->ping_started_) {
    if (!this->start_ping_()) {
      delay(1000);
      return;
    }

    this->ping_started_ = true;

    ESP_LOGI(TAG, "STEP 4 OK - Router ping started");
  }

  // -------------------------------------------------------
  // 3. Mostrar contador CSI a cada 10 segundos
  // -------------------------------------------------------

  const uint32_t now = millis();

  if (now - this->last_report_time_ < 10000) {
    return;
  }

  this->last_report_time_ = now;

  const uint32_t current =
      this->csi_packet_count_;

  const uint32_t received =
      current - this->last_reported_count_;

  this->last_reported_count_ = current;

  ESP_LOGI(
      TAG,
      "CSI packets: total=%u, last 10s=%u",
      static_cast<unsigned>(current),
      static_cast<unsigned>(received)
  );
}


bool ESPWiFiSensing::start_csi_() {
  wifi_ap_record_t ap_info{};

  esp_err_t err =
      esp_wifi_sta_get_ap_info(&ap_info);

  if (err != ESP_OK) {
    return false;
  }

  ESP_LOGI(
      TAG,
      "Wi-Fi ready - RSSI %d dBm, channel %u",
      ap_info.rssi,
      ap_info.primary
  );

  // Registar callback CSI
  err =
      esp_wifi_set_csi_rx_cb(
          ESPWiFiSensing::csi_callback_,
          this
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "esp_wifi_set_csi_rx_cb failed: %s",
        esp_err_to_name(err)
    );

    return false;
  }

  // ESP32-C6 / ESP-IDF 5.5.x
  wifi_csi_config_t config{};

  err =
      esp_wifi_set_csi_config(&config);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "esp_wifi_set_csi_config failed: %s",
        esp_err_to_name(err)
    );

    return false;
  }

  err =
      esp_wifi_set_csi(true);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "esp_wifi_set_csi(true) failed: %s",
        esp_err_to_name(err)
    );

    return false;
  }

  ESP_LOGI(TAG, "Native CSI receiver started");

  return true;
}


bool ESPWiFiSensing::start_ping_() {
  // Obter a interface STA criada pelo ESPHome.
  esp_netif_t *netif =
      esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

  if (netif == nullptr) {
    ESP_LOGW(TAG, "Wi-Fi STA netif not ready");
    return false;
  }

  // Obter IP/gateway atual.
  esp_netif_ip_info_t ip_info{};

  esp_err_t err =
      esp_netif_get_ip_info(
          netif,
          &ip_info
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "esp_netif_get_ip_info failed: %s",
        esp_err_to_name(err)
    );

    return false;
  }

  if (ip_info.gw.addr == 0) {
    ESP_LOGW(TAG, "Gateway not available yet");
    return false;
  }

  ESP_LOGI(
      TAG,
      "Router gateway: " IPSTR,
      IP2STR(&ip_info.gw)
  );

  // Configuração base oficial do ESP-IDF.
  esp_ping_config_t ping_config =
      ESP_PING_DEFAULT_CONFIG();

  // Gateway/router como destino.
  ip_addr_t target_addr{};
  target_addr.type = IPADDR_TYPE_V4;
  ip4_addr_set_u32(
      ip_2_ip4(&target_addr),
      ip_info.gw.addr
  );

  ping_config.target_addr = target_addr;

  // Ping contínuo.
  ping_config.count = ESP_PING_COUNT_INFINITE;

  // TESTE 4:
  // 100 ms = 10 pings por segundo.
  //
  // Começamos conservador.
  ping_config.interval_ms = 100;

  // Timeout inferior ao intervalo.
  ping_config.timeout_ms = 80;

  // Payload suficiente para gerar tráfego útil,
  // sem exagerarmos neste primeiro teste.
  ping_config.data_size = 32;

  // Não precisamos de callbacks de ping.
  esp_ping_callbacks_t callbacks{};

  err =
      esp_ping_new_session(
          &ping_config,
          &callbacks,
          &this->ping_handle_
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "esp_ping_new_session failed: %s",
        esp_err_to_name(err)
    );

    this->ping_handle_ = nullptr;
    return false;
  }

  err =
      esp_ping_start(
          this->ping_handle_
      );

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "esp_ping_start failed: %s",
        esp_err_to_name(err)
    );

    esp_ping_delete_session(
        this->ping_handle_
    );

    this->ping_handle_ = nullptr;

    return false;
  }

  ESP_LOGI(
      TAG,
      "Router ping running: interval=100ms (~10 pings/s)"
  );

  return true;
}


void ESPWiFiSensing::csi_callback_(
    void *ctx,
    wifi_csi_info_t *data
) {
  // Callback da Wi-Fi task:
  // continuamos sem fazer processamento aqui.

  if (ctx == nullptr || data == nullptr) {
    return;
  }

  auto *self =
      static_cast<ESPWiFiSensing *>(ctx);

  self->csi_packet_count_++;
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  STEP 4 - Native CSI + router ping");
  ESP_LOGCONFIG(TAG, "  CSI callback: ENABLED");
  ESP_LOGCONFIG(TAG, "  Router ping: 10 pings/s");
  ESP_LOGCONFIG(TAG, "  CSI processing: PACKET COUNTER ONLY");
  ESP_LOGCONFIG(TAG, "  esp-radar processing: NOT STARTED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
