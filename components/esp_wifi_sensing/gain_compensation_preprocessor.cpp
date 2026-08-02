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
  this->last_gain_status_ = -1;
  this->baseline_sample_count_ = 0;
  this->compensated_packet_count_ = 0;
  this->compensation_log_countdown_ = 0;

#ifdef USE_ESP_WIFI_SENSING_GAIN_COMPENSATION
  if (enabled) {
    esp_csi_gain_ctrl_reset_rx_gain_baseline();
    ESP_LOGI(TAG, "Gain compensation enabled; collecting RX gain baseline");
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
  if (static_cast<int>(status) != this->last_gain_status_) {
    this->last_gain_status_ = static_cast<int>(status);
    ESP_LOGI(
        TAG,
        "RX gain state changed: %d baseline_samples=%u compensated_packets=%u",
        this->last_gain_status_,
        static_cast<unsigned>(this->baseline_sample_count_),
        static_cast<unsigned>(this->compensated_packet_count_)
    );
  }

  if (status == RX_GAIN_COLLECT) {
    esp_err_t err = esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "esp_csi_gain_ctrl_record_rx_gain failed: %s", esp_err_to_name(err));
    } else {
      this->baseline_sample_count_++;
      if (this->baseline_sample_count_ == 1 || (this->baseline_sample_count_ % 64) == 0) {
        ESP_LOGI(
            TAG,
            "RX gain baseline collecting: samples=%u agc=%u fft=%d",
            static_cast<unsigned>(this->baseline_sample_count_),
            static_cast<unsigned>(agc_gain),
            static_cast<int>(fft_gain)
        );
      }
    }

    return;
  }

  if (status != RX_GAIN_READY && status != RX_GAIN_FORCE) {
    return;
  }

  this->compensated_bytes_.assign(input.raw_bytes, input.raw_bytes + input.len);

  const bool log_compensation = this->compensation_log_countdown_ == 0;
  uint32_t raw_abs_sum = 0;
  if (log_compensation) {
    for (uint16_t i = 0; i < input.len; i++) {
      const int value = static_cast<int>(input.raw_bytes[i]);
      raw_abs_sum += static_cast<uint32_t>(value < 0 ? -value : value);
    }
  }

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

  uint32_t compensated_abs_sum = 0;
  if (log_compensation) {
    for (uint16_t i = 0; i < input.len; i++) {
      const int value = static_cast<int>(this->compensated_bytes_[i]);
      compensated_abs_sum += static_cast<uint32_t>(value < 0 ? -value : value);
    }
  }

  this->compensated_packet_count_++;
  if (log_compensation) {
    ESP_LOGI(
        TAG,
        "RX gain compensation active: packets=%u baseline_samples=%u factor=%.3f agc=%u fft=%d raw_sum=%u compensated_sum=%u",
        static_cast<unsigned>(this->compensated_packet_count_),
        static_cast<unsigned>(this->baseline_sample_count_),
        static_cast<double>(compensate_gain),
        static_cast<unsigned>(agc_gain),
        static_cast<int>(fft_gain),
        static_cast<unsigned>(raw_abs_sum),
        static_cast<unsigned>(compensated_abs_sum)
    );
    this->compensation_log_countdown_ = 255;
  } else {
    this->compensation_log_countdown_--;
  }

  output.raw_bytes = this->compensated_bytes_.data();
  output.gain_compensated = true;
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
