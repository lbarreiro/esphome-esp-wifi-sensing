#include "wifi_sensing.h"

#include <cstdlib>

#include "esp_err.h"
#include "esp_netif.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "STEP 5 - CSI variation test");
}


void ESPWiFiSensing::loop() {
  // -------------------------------------------------------
  // 1. CSI
  // -------------------------------------------------------

  if (!this->csi_started_) {
    if (!this->start_csi_()) {
      delay(1000);
      return;
    }

    this->csi_started_ = true;
    ESP_LOGI(TAG, "STEP 5 - CSI enabled");
  }

  // -------------------------------------------------------
  // 2. Ping ao router
  // -------------------------------------------------------

  if (!this->ping_started_) {
    if (!this->start_ping_()) {
      delay(1000);
      return;
    }

    this->ping_started_ = true;
    ESP_LOGI(TAG, "STEP 5 OK - Router ping started");
  }

  // -------------------------------------------------------
  // 3. Processar nova métrica CSI
  // -------------------------------------------------------

  if (this->new_csi_sample_) {
    const uint32_t metric = this->latest_csi_metric_;

    this->new_csi_sample_ = false;

    if (this->have_previous_sample_) {
      uint32_t variation;

      if (metric >= this->previous_csi_metric_) {
        variation = metric - this->previous_csi_metric_;
      } else {
        variation = this->previous_csi_metric_ - metric;
      }

      this->variation_sum_ += variation;
      this->variation_samples_++;

      if (variation > this->variation_max_) {
        this->variation_max_ = variation;
      }
    }

    this->previous_csi_metric_ = metric;
    this->have_previous_sample_ = true;
  }

  // -------------------------------------------------------
  // 4. Relatório a cada 5 segundos
  // -------------------------------------------------------

  const uint32_t now = millis();

  if (now - this->last_report_time_ < 5000) {
    return;
  }

  this->last_report_time_ = now;

  const uint32_t current =
      this->csi_packet_count_;

  const uint32_t received =
      current - this->last_reported_count_;

  this->last_reported_count_ = current;

  uint32_t average_variation = 0;

  if (this->variation_samples_ > 0) {
    average_variation =
        this->variation_sum_ /
        this->variation_samples_;
  }

  ESP_LOGI(
      TAG,
      "CSI: packets=%u/5s len=%u metric=%u variation avg=%u max=%u",
      static_cast<unsigned>(received),
      static_cast<unsigned>(this->latest_csi_len_),
      static_cast<unsigned>(this->latest_csi_metric_),
      static_cast<unsigned>(average_variation),
      static_cast<unsigned>(this->variation_max_)
  );

  // Começar uma nova janela estatística.
  this->variation_sum_ = 0;
  this->variation_max_ = 0;
  this->variation_samples_ = 0;
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
  esp_netif_t *netif =
      esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

  if (netif == nullptr) {
    ESP_LOGW(TAG, "Wi-Fi STA netif not ready");
    return false;
  }

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

  esp_ping_config_t ping_config =
      ESP_PING_DEFAULT_CONFIG();

  ip_addr_t target_addr{};
  target_addr.type = IPADDR_TYPE_V4;

  ip4_addr_set_u32(
      ip_2_ip4(&target_addr),
      ip_info.gw.addr
  );

  ping_config.target_addr = target_addr;

  ping_config.count = ESP_PING_COUNT_INFINITE;

  // Mantemos exatamente os 10 pings/s do Passo 4.
  ping_config.interval_ms = 100;
  ping_config.timeout_ms = 80;
  ping_config.data_size = 32;

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
  if (
      ctx == nullptr ||
      data == nullptr ||
      data->buf == nullptr ||
      data->len == 0
  ) {
    return;
  }

  auto *self =
      static_cast<ESPWiFiSensing *>(ctx);

  self->csi_packet_count_++;

  // -------------------------------------------------------
  // Métrica extremamente simples para o primeiro teste.
  //
  // Somamos o valor absoluto de todos os bytes CSI.
  //
  // NÃO é ainda um algoritmo de presença.
  // Queremos apenas descobrir se a métrica reage fisicamente
  // quando alguém altera o caminho do sinal Wi-Fi.
  // -------------------------------------------------------

  uint32_t metric = 0;

  const int8_t *buf =
      reinterpret_cast<const int8_t *>(data->buf);

  for (uint16_t i = 0; i < data->len; i++) {
    int value = static_cast<int>(buf[i]);

    metric += static_cast<uint32_t>(
        value < 0 ? -value : value
    );
  }

  self->latest_csi_metric_ = metric;
  self->latest_csi_len_ = data->len;
  self->new_csi_sample_ = true;
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  STEP 5 - CSI variation");
  ESP_LOGCONFIG(TAG, "  CSI callback: ENABLED");
  ESP_LOGCONFIG(TAG, "  Router ping: 10 pings/s");
  ESP_LOGCONFIG(TAG, "  Metric: absolute CSI sum");
  ESP_LOGCONFIG(TAG, "  esp-radar processing: NOT STARTED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
