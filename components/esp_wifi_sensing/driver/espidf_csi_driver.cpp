#include "esp_wifi_sensing/driver/espidf_csi_driver.h"

#include "esp_err.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing::driver";

bool EspIdfCsiDriver::start(void *ctx, wifi_csi_rx_cb_t callback) {
  if (this->started_) {
    return true;
  }

  esp_err_t err = esp_wifi_set_csi_rx_cb(callback, ctx);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_csi_rx_cb failed: %s", esp_err_to_name(err));
    return false;
  }

  wifi_csi_config_t config{};
  err = esp_wifi_set_csi_config(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_csi_config failed: %s", esp_err_to_name(err));
    return false;
  }

  err = esp_wifi_set_csi(true);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_csi(true) failed: %s", esp_err_to_name(err));
    return false;
  }

  this->started_ = true;
  ESP_LOGI(TAG, "Native CSI receiver started");
  return true;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
