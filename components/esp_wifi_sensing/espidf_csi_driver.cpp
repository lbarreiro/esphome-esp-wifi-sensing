#include "espidf_csi_driver.h"

#include "esp_err.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing::driver";

bool EspIdfCsiDriver::start(void *ctx, wifi_csi_cb_t callback) {
  if (this->started_) {
    return true;
  }

  wifi_csi_config_t config{};
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
  config.enable = true;
  config.acquire_csi_legacy = true;
  config.acquire_csi_force_lltf = true;
  config.acquire_csi_ht20 = true;
  config.acquire_csi_ht40 = true;
  config.acquire_csi_vht = false;
  config.acquire_csi_su = false;
  config.acquire_csi_mu = false;
  config.acquire_csi_dcm = false;
  config.acquire_csi_beamformed = false;
  config.acquire_csi_he_stbc_mode = 2;
  config.val_scale_cfg = 0;
  config.dump_ack_en = false;
#elif CONFIG_IDF_TARGET_ESP32C6
  config.enable = true;
  config.acquire_csi_legacy = true;
  config.acquire_csi_ht20 = true;
  config.acquire_csi_ht40 = true;
  config.acquire_csi_su = false;
  config.acquire_csi_mu = false;
  config.acquire_csi_dcm = false;
  config.acquire_csi_beamformed = false;
  config.acquire_csi_he_stbc = 2;
  config.val_scale_cfg = false;
  config.dump_ack_en = false;
#else
  config.lltf_en = true;
  config.htltf_en = false;
  config.stbc_htltf2_en = false;
  config.ltf_merge_en = true;
  config.channel_filter_en = true;
  config.manu_scale = true;
  config.shift = true;
#endif

  esp_err_t err = esp_wifi_set_csi_config(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_csi_config failed: %s", esp_err_to_name(err));
    return false;
  }

  err = esp_wifi_set_csi_rx_cb(callback, ctx);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_csi_rx_cb failed: %s", esp_err_to_name(err));
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
