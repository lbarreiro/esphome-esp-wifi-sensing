#include "gain_compensation_preprocessor.h"

#include "esp_err.h"
#include "esphome/core/log.h"

#if __has_include("esp_csi_gain_ctrl.h")
#include "esp_csi_gain_ctrl.h"
#define ESP_WIFI_SENSING_HAS_GAIN_CTRL 1
#else
#define ESP_WIFI_SENSING_HAS_GAIN_CTRL 0
#endif

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing::gain_compensation";

void GainCompensationPreprocessor::process(
    const CsiPacket &input,
    CsiPacket &output
) {
  output = input;

  if (!this->enabled_ || input.raw_bytes == nullptr || input.len == 0) {
    return;
  }

#if ESP_WIFI_SENSING_HAS_GAIN_CTRL
  if (input.rx_ctrl == nullptr) {
    ESP_LOGW(TAG, "Gain compensation enabled but CSI RX control metadata is unavailable");
    return;
  }

  this->compensated_bytes_.assign(input.raw_bytes, input.raw_bytes + input.len);

  uint8_t agc_gain = 0;
  int8_t fft_gain = 0;
  esp_csi_gain_ctrl_get_rx_gain(input.rx_ctrl, &agc_gain, &fft_gain);

  if (esp_csi_gain_ctrl_get_gain_status() == RX_GAIN_COLLECT) {
    esp_err_t err = esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "esp_csi_gain_ctrl_record_rx_gain failed: %s", esp_err_to_name(err));
    }
  }

  float compensate_gain = 1.0f;
  esp_err_t err =
      esp_csi_gain_ctrl_compensate_rx_gain(
          this->compensated_bytes_.data(),
          input.len,
          false,
          &compensate_gain,
          agc_gain,
          fft_gain
      );
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_csi_gain_ctrl_compensate_rx_gain failed: %s", esp_err_to_name(err));
    return;
  }

  output.raw_bytes = this->compensated_bytes_.data();
#else
  static bool logged_unavailable = false;
  if (!logged_unavailable) {
    ESP_LOGW(TAG, "Gain compensation enabled but esp_csi_gain_ctrl is not available in this build");
    logged_unavailable = true;
  }
#endif
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
