#include "wifi_sensing.h"

#include <cstdint>

#include "esp_err.h"
#include "esp_netif.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";

static const char *csi_algorithm_to_string(CsiAlgorithm algorithm) {
  switch (algorithm) {
    case CsiAlgorithm::VARIANCE: return "variance";
    case CsiAlgorithm::AMPLITUDE: return "amplitude";
    case CsiAlgorithm::JITTER: return "jitter";
    case CsiAlgorithm::ESP_RADAR: return "esp_radar";
    case CsiAlgorithm::MVS: return "mvs";
    case CsiAlgorithm::ABSOLUTE_SUM: default: return "absolute_sum";
  }
}

void ESPWiFiSensing::set_gain_compensation_enabled(bool enabled) {
  this->gain_compensation_enabled_ = enabled;
  this->pipeline_.set_gain_compensation_enabled(enabled);
}

void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "CSI motion experiment starting...");
  this->pipeline_.set_gain_compensation_enabled(this->gain_compensation_enabled_);
}

void ESPWiFiSensing::loop() {
  if (!this->csi_started_) {
    if (!this->start_csi_()) { delay(1000); return; }
    this->csi_started_ = true;
    ESP_LOGI(TAG, "CSI enabled");
  }
  if (!this->ping_started_) {
    if (!this->start_ping_()) { delay(1000); return; }
    this->ping_started_ = true;
    ESP_LOGI(TAG, "Router ping started");
  }
  if (this->new_csi_sample_) {
    const uint32_t metric = this->latest_csi_metric_;
    this->new_csi_sample_ = false;
    if (this->have_previous_sample_) {
      const uint32_t variation = metric >= this->previous_csi_metric_ ? metric - this->previous_csi_metric_ : this->previous_csi_metric_ - metric;
      this->variation_sum_ += variation;
      this->variation_samples_++;
      if (variation > this->variation_max_) this->variation_max_ = variation;
    }
    this->previous_csi_metric_ = metric;
    this->have_previous_sample_ = true;
  }
  const uint32_t now = millis();
  if (now - this->last_report_time_ < 5000) return;
  this->last_report_time_ = now;
  const uint32_t current = this->csi_packet_count_;
  const uint32_t received = current - this->last_reported_count_;
  this->last_reported_count_ = current;
  const uint32_t average_variation = this->variation_samples_ > 0 ? this->variation_sum_ / this->variation_samples_ : 0;
  ESP_LOGI(TAG, "CSI: packets=%u/5s len=%u metric=%u variation avg=%u max=%u",
           static_cast<unsigned>(received), static_cast<unsigned>(this->latest_csi_len_),
           static_cast<unsigned>(this->latest_csi_metric_), static_cast<unsigned>(average_variation),
           static_cast<unsigned>(this->variation_max_));
  this->variation_sum_ = 0; this->variation_max_ = 0; this->variation_samples_ = 0;
}

bool ESPWiFiSensing::start_csi_() {
  wifi_ap_record_t ap_info{};
  const esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
  if (err != ESP_OK) return false;
  ESP_LOGI(TAG, "Wi-Fi ready - RSSI %d dBm, channel %u", ap_info.rssi, ap_info.primary);
  return this->driver_.start(this, ESPWiFiSensing::csi_callback_);
}

bool ESPWiFiSensing::start_ping_() {
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == nullptr) return false;
  esp_netif_ip_info_t ip_info{};
  esp_err_t err = esp_netif_get_ip_info(netif, &ip_info);
  if (err != ESP_OK || ip_info.gw.addr == 0) return false;
  esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
  ip_addr_t target_addr{}; target_addr.type = IPADDR_TYPE_V4;
  ip4_addr_set_u32(ip_2_ip4(&target_addr), ip_info.gw.addr);
  ping_config.target_addr = target_addr; ping_config.count = ESP_PING_COUNT_INFINITE;
  ping_config.interval_ms = 100; ping_config.timeout_ms = 80; ping_config.data_size = 32;
  esp_ping_callbacks_t callbacks{};
  err = esp_ping_new_session(&ping_config, &callbacks, &this->ping_handle_);
  if (err != ESP_OK) { this->ping_handle_ = nullptr; return false; }
  err = esp_ping_start(this->ping_handle_);
  if (err != ESP_OK) { esp_ping_delete_session(this->ping_handle_); this->ping_handle_ = nullptr; return false; }
  ESP_LOGI(TAG, "Router ping running: interval=100ms (~10 pings/s)");
  return true;
}

void ESPWiFiSensing::csi_callback_(void *ctx, wifi_csi_info_t *data) {
  if (ctx == nullptr || data == nullptr || data->buf == nullptr || data->len == 0) return;
  auto *self = static_cast<ESPWiFiSensing *>(ctx);
  self->csi_packet_count_++;
  const int8_t *buf = reinterpret_cast<const int8_t *>(data->buf);
  CsiPacket packet{}; packet.len = data->len; packet.raw_bytes = buf; packet.rx_ctrl = &data->rx_ctrl;
  self->pipeline_.process_packet(packet);
  const CsiPacket &processed_packet = self->pipeline_.latest_packet();
  uint32_t metric = 0;
  if (self->selected_algorithm_ == CsiAlgorithm::VARIANCE) {
    metric = self->variance_algorithm_.process(processed_packet);
  } else if (self->selected_algorithm_ == CsiAlgorithm::AMPLITUDE || self->selected_algorithm_ == CsiAlgorithm::ESP_RADAR || self->selected_algorithm_ == CsiAlgorithm::MVS) {
    metric = self->amplitude_algorithm_.process(processed_packet);
  } else if (self->selected_algorithm_ == CsiAlgorithm::JITTER) {
    metric = self->jitter_algorithm_.process(processed_packet);
  } else if (processed_packet.raw_bytes != nullptr) {
    for (uint16_t i = 0; i < processed_packet.len; i++) {
      const int value = static_cast<int>(processed_packet.raw_bytes[i]);
      metric += static_cast<uint32_t>(value < 0 ? -value : value);
    }
    metric = self->algorithm_.process(metric);
  }
  const uint16_t csi_len = self->pipeline_.latest_len();
  self->latest_csi_metric_ = metric;
  self->latest_csi_len_ = csi_len;
  self->new_csi_sample_ = self->pipeline_.has_new_sample();
  const uint32_t sample_count = self->csi_packet_count_;
  const uint32_t timestamp = millis();

  if (timestamp - self->diagnostic_last_log_time_ >= 1000) {
    if (self->diagnostic_have_previous_sample_) {
      const int64_t delta = static_cast<int64_t>(metric) - static_cast<int64_t>(self->diagnostic_previous_csi_metric_);
      const uint64_t absolute_delta = delta < 0 ? static_cast<uint64_t>(-delta) : static_cast<uint64_t>(delta);
      ESP_LOGI(TAG, "CSI_SAMPLE,%u,%u,%s,%s,%u,%u,%lld,%llu", static_cast<unsigned>(timestamp), static_cast<unsigned>(sample_count),
               csi_algorithm_to_string(self->selected_algorithm_), self->gain_compensation_enabled_ ? "ON" : "OFF",
               static_cast<unsigned>(csi_len), static_cast<unsigned>(metric), static_cast<long long>(delta), static_cast<unsigned long long>(absolute_delta));
    } else {
      ESP_LOGI(TAG, "CSI_SAMPLE,%u,%u,%s,%s,%u,%u,N/A,N/A", static_cast<unsigned>(timestamp), static_cast<unsigned>(sample_count),
               csi_algorithm_to_string(self->selected_algorithm_), self->gain_compensation_enabled_ ? "ON" : "OFF",
               static_cast<unsigned>(csi_len), static_cast<unsigned>(metric));
    }
    self->diagnostic_previous_csi_metric_ = metric; self->diagnostic_have_previous_sample_ = true; self->diagnostic_last_log_time_ = timestamp;
  }

  if (self->selected_algorithm_ == CsiAlgorithm::ESP_RADAR) {
    const EspRadarMotionResult radar_result = self->esp_radar_detector_.update(metric, timestamp);
    if (self->motion_binary_sensor_ != nullptr) self->motion_binary_sensor_->publish_state(radar_result.active);
    if (self->diagnostic_publish_rate_limiter_.should_publish(timestamp)) {
      if (self->metric_sensor_ != nullptr) self->metric_sensor_->publish_state(metric);
      if (self->baseline_mean_sensor_ != nullptr) self->baseline_mean_sensor_->publish_state(radar_result.smooth);
      if (self->baseline_stddev_sensor_ != nullptr) self->baseline_stddev_sensor_->publish_state(radar_result.jitter);
      if (self->adaptive_threshold_sensor_ != nullptr) self->adaptive_threshold_sensor_->publish_state(radar_result.enter_level);
      if (self->csi_jitter_sensor_ != nullptr) self->csi_jitter_sensor_->publish_state(radar_result.jitter);
      if (self->csi_enter_level_sensor_ != nullptr) self->csi_enter_level_sensor_->publish_state(radar_result.enter_level);
    }
  } else if (self->selected_algorithm_ == CsiAlgorithm::MVS) {
    const MvsMotionResult mvs_result = self->mvs_detector_.update(metric, timestamp);
    if (self->motion_binary_sensor_ != nullptr) self->motion_binary_sensor_->publish_state(mvs_result.active);
    if (self->diagnostic_publish_rate_limiter_.should_publish(timestamp)) {
      if (self->metric_sensor_ != nullptr) self->metric_sensor_->publish_state(metric);
      if (self->csi_variance_sensor_ != nullptr) self->csi_variance_sensor_->publish_state(mvs_result.variance);
      if (self->csi_variance_threshold_sensor_ != nullptr) self->csi_variance_threshold_sensor_->publish_state(mvs_result.threshold);
    }
  } else {
    const AdaptiveMotionDetectorResult motion_result = self->motion_detector_.update(metric, timestamp);
    if (self->motion_binary_sensor_ != nullptr) self->motion_binary_sensor_->publish_state(motion_result.motion);
    if (self->diagnostic_publish_rate_limiter_.should_publish(timestamp)) {
      if (self->metric_sensor_ != nullptr) self->metric_sensor_->publish_state(metric);
      if (self->baseline_mean_sensor_ != nullptr) self->baseline_mean_sensor_->publish_state(motion_result.baseline_mean);
      if (self->baseline_stddev_sensor_ != nullptr) self->baseline_stddev_sensor_->publish_state(motion_result.baseline_stddev);
      if (self->adaptive_threshold_sensor_ != nullptr) self->adaptive_threshold_sensor_->publish_state(motion_result.adaptive_threshold);
    }
  }
  self->pipeline_.clear_new_sample();
}

void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  CSI callback: ENABLED");
  ESP_LOGCONFIG(TAG, "  Router ping: 10 pings/s");
  ESP_LOGCONFIG(TAG, "  Algorithm: %s", csi_algorithm_to_string(this->selected_algorithm_));
  ESP_LOGCONFIG(TAG, "  Gain compensation: %s", this->gain_compensation_enabled_ ? "ENABLED" : "disabled");
  if (this->selected_algorithm_ == CsiAlgorithm::ESP_RADAR) ESP_LOGCONFIG(TAG, "  ESP Radar FSM: jitter + smooth + enter/exit hysteresis + active filter");
  if (this->selected_algorithm_ == CsiAlgorithm::MVS) ESP_LOGCONFIG(TAG, "  MVS: rolling variance + quiet baseline + hysteresis + consecutive hits");
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
