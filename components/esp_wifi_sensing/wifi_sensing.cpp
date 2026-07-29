#include "wifi_sensing.h"

#include "esp_err.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "STEP 3 - Native ESP-IDF CSI test");
}


void ESPWiFiSensing::loop() {
  // ESPHome pode chamar setup() antes de o Wi-Fi estar
  // completamente pronto. Tentamos iniciar CSI apenas
  // quando a interface Wi-Fi já estiver operacional.

  if (!this->csi_started_) {
    if (!this->start_csi_()) {
      delay(1000);
      return;
    }

    this->csi_started_ = true;

    ESP_LOGI(TAG, "STEP 3 OK - CSI enabled");
  }

  const uint32_t now = millis();

  // Relatório a cada 10 segundos.
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
  // Primeiro confirmamos que o ESP está realmente
  // associado ao Access Point.
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

  // Registar callback CSI.
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

  // Configuração CSI.
  //
  // Inicialização a zero primeiro para não deixarmos
  // campos indefinidos.
  wifi_csi_config_t config{};

  config.lltf_en = true;
  config.htltf_en = true;
  config.stbc_htltf2_en = true;
  config.ltf_merge_en = true;
  config.channel_filter_en = false;
  config.manu_scale = false;
  config.shift = 0;

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

  // Ativar receção CSI.
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

  return true;
}


void ESPWiFiSensing::csi_callback_(
    void *ctx,
    wifi_csi_info_t *data
) {
  // IMPORTANTE:
  // Este callback corre dentro da task Wi-Fi.
  // Nada de logs, processamento CSI ou Home Assistant aqui.
  //
  // Por enquanto contamos APENAS os pacotes.

  if (ctx == nullptr || data == nullptr) {
    return;
  }

  auto *self =
      static_cast<ESPWiFiSensing *>(ctx);

  self->csi_packet_count_++;
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  STEP 3 - Native CSI");
  ESP_LOGCONFIG(TAG, "  CSI callback: ENABLED");
  ESP_LOGCONFIG(TAG, "  CSI processing: PACKET COUNTER ONLY");
  ESP_LOGCONFIG(TAG, "  esp-radar processing: NOT STARTED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
