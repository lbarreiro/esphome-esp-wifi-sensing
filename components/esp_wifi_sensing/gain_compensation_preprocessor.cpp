#include "gain_compensation_preprocessor.h"

#include "esp_err.h"
#include "esphome/core/log.h"

#ifdef USE_ESP_WIFI_SENSING_GAIN_COMPENSATION
#include "esp_csi_gain_ctrl.h"
#endif

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing::gain_compensation";

void GainCompensationPreprocessor::set_enabled(bool enabled) {
  if (this->enabled_ == enabled) {
    return;
  }

  this->enabled_ = enabled;
  this->compensated_bytes_.clear();
  this->ready_ = !enabled;
  this->ready_transition_ = false;

#ifdef USE_ESP_WIFI_SENSING_GAIN_COMPENSATION
  if (enabled) {
    esp_csi_gain_ctrl_reset_rx_gain_baseline();
  }
#endif
}

void GainCompensationPreprocessor::process(
    const CsiPacket &input,
    CsiPacket &output
) {
  output = input;

  if (!this->enabled_ || input.raw_bytes == nullptr || input.len == 0) {
    return;
  }

#ifdef USE_ESP_WIFI_SENSING_GAIN_COMPENSATION
  if (input.rx_ctrl == nullptr) {
    ESP_LOGW(TAG, "Gain compensation enabled but CSI RX control metadata is unavailable");
    return;
  }

  uint8_t agc_gain = 0;
  int8_t fft_gain = 0;
  esp_csi_gain_ctrl_get_rx_gain(input.rx_ctrl, &agc_gain, &fft_gain);

  rx_gain_status_t status = esp_csi_gain_ctrl_get_gain_status();
  if (status == RX_GAIN_COLLECT) {
    esp_err_t err = esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "esp_csi_gain_ctrl_record_rx_gain failed: %s", esp_err_to_name(err));
    }

    return;
  }

  if (status != RX_GAIN_READY && status != RX_GAIN_FORCE) {
    return;
  }

  this->compensated_bytes_.assign(input.raw_bytes, input.raw_bytes + input.len);

  float compensate_gain = 1.0f;
  esp_err_t err = esp_csi_gain_ctrl_compensate_rx_gain(
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
  if (!this->ready_) {
    this->ready_transition_ = true;
  }
  this->ready_ = true;
#else
  static bool warned = false;
  if (!warned) {
    warned = true;
    ESP_LOGW(TAG, "Gain compensation enabled but esp_csi_gain_ctrl is not compiled in");
  }
#endif
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
