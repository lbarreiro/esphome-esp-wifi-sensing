#include "wifi_sensing.h"

#include <cstdlib>

#include "esp_err.h"
#include "esp_netif.h"

namespace esphome {
/**
 * @file ESPWiFiSensing Component Architecture Documentation
 * 
 * COMPONENT LIFECYCLE:
 * ====================
 * 1. setup() - Initialization phase
 *    - Called once at component startup
 *    - Logs that CSI variation test is starting
 *    - Does NOT initialize CSI or ping yet (deferred to loop)
 * 
 * 2. loop() - Main execution loop (called repeatedly)
 *    - Phase 1: Lazy initialization of CSI receiver
 *      * Calls start_csi_() on first iteration
 *      * Registers csi_callback_ with ESP-IDF wifi stack
 *      * Retries every 1000ms if initialization fails
 *    - Phase 2: Lazy initialization of ping service
 *      * Calls start_ping_() after CSI is ready
 *      * Configures ping to gateway at 10 pings/second (100ms interval)
 *      * Retries every 1000ms if initialization fails
 *    - Phase 3: Process incoming CSI samples
 *      * Calculates variation metrics between consecutive samples
 *      * Maintains rolling statistics (sum, max, count)
 *    - Phase 4: Report statistics window
 *      * Publishes aggregated metrics at the configured update interval
 *      * Keeps the configured history in a sliding window
 * 
 * CSI ACQUISITION FLOW:
 * ====================
 * Entry Point: start_csi_()
 *   1. Query AP info via esp_wifi_sta_get_ap_info() to confirm Wi-Fi connection
 *   2. Register callback via esp_wifi_set_csi_rx_cb(ESPWiFiSensing::csi_callback_, this)
 *      - Uses static method with context pointer for callback dispatch
 *   3. Configure CSI via esp_wifi_set_csi_config() with default config
 *   4. Enable CSI via esp_wifi_set_csi(true)
 *   5. Once enabled, csi_callback_() invoked asynchronously per packet received
 * 
 * Callback Processing: csi_callback_()
 *   - Static callback from ESP-IDF driver layer
 *   - Extracts CSI buffer (wifi_csi_info_t.buf) containing I/Q subcarrier data
 *   - Computes simple metric: sum of absolute values of all CSI bytes
 *   - Sets new_csi_sample_ flag for loop() to process in next iteration
 *   - Thread-safe: callback updates only volatile member variables
 * 
 * ESPHOME API USAGE:
 * ==================
 * - ESPHome component base class pattern (setup/loop/dump_config lifecycle)
 * - No direct ESPHome sensor/API bindings visible (output mechanism not in this file)
 * - Logging framework: ESP_LOGI, ESP_LOGW, ESP_LOGE, ESP_LOGCONFIG macros
 * - Configuration dump: dump_config() for component introspection
 * - Likely inherits from esphome::Component base class (inferred)
 * - Assumes ESPHome integration layer handles WiFi initialization before setup()
 * 
 * ESPRESSIF OFFICIAL CSI LIBRARY COMPATIBILITY:
 * ==============================================
 * Current implementation uses:
 *   - Low-level ESP-IDF APIs: esp_wifi_set_csi_rx_cb(), esp_wifi_set_csi()
 *   - Direct CSI buffer access: reinterpret_cast<int8_t*>(data->buf)
 *   - Raw metrics on unprocessed CSI subcarrier values
 * 
 * Official esp-radar library (Espressif's CSI sensing stack) could replace:
 *   - BUT requires file dependency analysis to determine if esp-radar is:
 *     * Already integrated into this project
 *     * Compatible with ESPHome's build system
 *     * Provides necessary WiFi presence detection without extra overhead
 *   - esp-radar would handle subcarrier processing, FMCW algorithms
 *   - Current approach (step 5) appears to be experimental baseline
 *   - Architecture suggests this is prototype; production may integrate esp-radar
 * 
 * TECHNICAL RISKS:
 * ================
 * RISK 1: Callback Threading
 *   - csi_callback_() invoked from WiFi driver ISR/task context (not loop context)
 *   - Current code uses only atomic member assignments (latest_csi_metric_, flag)
 *   - Risk: If loop() reads 64-bit metrics, potential tearing on 32-bit CPU
 *   - Mitigation needed: Verify all member updates are atomic or guarded by mutex
 * 
 * RISK 2: Blocking Operations in Loop
 *   - delay(1000) in loop during initialization blocks entire component
 *   - Prevents ESPHome from servicing other components during startup
 *   - Should use non-blocking timer mechanism (e.g., last_attempt_time_)
 *   - Impact: Startup delay multiplies if CSI or ping init fails
 * 
 * RISK 3: Metric Simplicity for Production
 *   - Sum-of-absolute-CSI metric is NOT presence detection algorithm
 *   - No filtering, no ML model, no multipath rejection
 *   - High susceptibility to environmental noise and static objects
 *   - Risk: False positives in presence detection cannot be avoided with current metric
 *   - Code comments acknowledge this is "Step 5" (experimental baseline only)
 * 
 * RISK 4: Memory and Buffer Management
 *   - CSI buffer lifetime: callback receives pointer to ESP-IDF managed buffer
 *   - Current code reads buffer but does NOT copy; relies on synchronous processing
 *   - Risk: If metric calculation is slow or delayed, buffer may be reused/freed
 *   - Mitigation: Verify buffer is valid for entire callback duration (likely true)
 * 
 * RISK 5: Ping-CSI Coupling Assumption
 *   - Code assumes ping traffic triggers CSI reception
 *   - No confirmation that ping packets actually cause CSI capture
 *   - Risk: WiFi driver may not generate CSI for outbound ping frames
 *   - Mitigation needed: Verify CSI is captured for both RX and TX directions
 * 
 * RISK 6: Missing Error Recovery
 *   - Once CSI/ping started, no mechanism to detect failure and restart
 *   - If WiFi driver crashes or callback stops, loop() continues but produces stale data
 *   - Missing watchdog on packet_count or variation metrics
 *   - Risk: Silent data corruption (reports last valid metric indefinitely)
 * 
 * REQUIRED FILES FOR COMPLETE ANALYSIS:
 * =====================================
 * Please provide:
 *   1. wifi_sensing.h - Member variable declarations, class definition
 *   2. CMakeLists.txt or manifest.yaml - Build configuration and dependencies
 *   3. Any caller/wrapper code showing how metrics are exported to ESPHome
 *   4. Header file to confirm thread-safety guarantees on member variables
 */
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::set_gain_compensation_enabled(bool enabled) {
  this->gain_compensation_enabled_ = enabled;
  this->pipeline_.set_gain_compensation_enabled(enabled);
}


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "STEP 5 - CSI variation test");
  this->pipeline_.set_gain_compensation_enabled(this->gain_compensation_enabled_);
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

      this->add_variation_sample_(variation, millis());
    }

    this->previous_csi_metric_ = metric;
    this->have_previous_sample_ = true;
  }

  // -------------------------------------------------------
  // 4. Relatório periódico com janela estatística deslizante
  // -------------------------------------------------------

  const uint32_t now = millis();

  if (now - this->last_report_time_ < this->statistics_update_ms_) {
    return;
  }

  this->last_report_time_ = now;

  const uint32_t current =
      this->csi_packet_count_;

  const uint32_t received =
      current - this->last_reported_count_;

  this->last_reported_count_ = current;

  uint32_t average_variation = 0;

  this->prune_variation_samples_(now);

  if (this->variation_window_count_ > 0) {
    average_variation =
        this->variation_sum_ /
        this->variation_window_count_;
  }

  ESP_LOGI(
      TAG,
      "CSI: packets=%u/%us len=%u metric=%u variation avg=%u max=%u samples=%u",
      static_cast<unsigned>(received),
      static_cast<unsigned>(this->statistics_update_ms_ / 1000),
      static_cast<unsigned>(this->latest_csi_len_),
      static_cast<unsigned>(this->latest_csi_metric_),
      static_cast<unsigned>(average_variation),
      static_cast<unsigned>(this->variation_max_),
      static_cast<unsigned>(this->variation_window_count_)
  );

  if (this->metric_sensor_ != nullptr) {
    this->metric_sensor_->publish_state(this->latest_csi_metric_);
  }

  if (this->variation_avg_sensor_ != nullptr) {
    this->variation_avg_sensor_->publish_state(average_variation);
  }

  if (this->warmup_active_(now)) {
    const uint32_t elapsed_ms = now < this->warmup_time_ms_ ? now : this->warmup_time_ms_;
    ESP_LOGI(
        TAG,
        "Adaptive baseline warm-up (%u/%u s)",
        static_cast<unsigned>(elapsed_ms / 1000),
        static_cast<unsigned>(this->warmup_time_ms_ / 1000)
    );

    this->consecutive_above_threshold_ = 0;
    this->motion_state_ = false;
    this->reset_transient_pulse_test_();

    if (this->motion_binary_sensor_ != nullptr) {
      this->motion_binary_sensor_->publish_state(false);
    }

    return;
  }

  if (this->warmup_time_ms_ > 0 && !this->warmup_complete_logged_) {
    ESP_LOGI(TAG, "Adaptive baseline warm-up complete");
    this->warmup_complete_logged_ = true;
  }

  if (this->adaptive_threshold_enabled_ && !this->adaptive_baseline_.initialized()) {
    this->adaptive_baseline_.update(
        static_cast<float>(average_variation),
        false,
        now,
        this->statistics_update_ms_
    );
  }

  const float decision_threshold = this->adaptive_threshold_enabled_ ?
      this->adaptive_baseline_.adaptive_threshold() :
      static_cast<float>(this->motion_threshold_);

  bool motion_detected = false;

  const bool above_threshold = static_cast<float>(average_variation) > decision_threshold;
  const uint32_t accepted_threshold_crossings = this->evaluate_transient_pulse_candidate_(
      above_threshold,
      average_variation,
      decision_threshold,
      now
  );

  if (accepted_threshold_crossings > 0) {
    this->consecutive_above_threshold_ += accepted_threshold_crossings;
    motion_detected = this->consecutive_above_threshold_ >= this->motion_debounce_;
  } else {
    this->consecutive_above_threshold_ = 0;
    motion_detected = false;
  }

  if (this->adaptive_threshold_enabled_) {
    this->adaptive_baseline_.update(
        static_cast<float>(average_variation),
        motion_detected,
        now,
        this->statistics_update_ms_
    );

    if (this->baseline_mean_sensor_ != nullptr) {
      this->baseline_mean_sensor_->publish_state(this->adaptive_baseline_.baseline_mean());
    }

    if (this->baseline_stddev_sensor_ != nullptr) {
      this->baseline_stddev_sensor_->publish_state(this->adaptive_baseline_.baseline_stddev());
    }

    if (this->adaptive_threshold_sensor_ != nullptr) {
      this->adaptive_threshold_sensor_->publish_state(this->adaptive_baseline_.adaptive_threshold());
    }
  }

  this->motion_state_ = this->apply_motion_hold_(motion_detected, now);

  if (this->motion_binary_sensor_ != nullptr) {
    this->motion_binary_sensor_->publish_state(this->motion_state_);
  }

}


void ESPWiFiSensing::add_variation_sample_(uint32_t variation, uint32_t now) {
  if (this->variation_window_.empty()) {
    const uint32_t expected_samples = (this->statistics_window_ms_ / 100) + 2;
    const uint32_t capacity = expected_samples < 8 ? 8 : expected_samples;
    this->variation_window_.resize(capacity);
  }

  if (this->variation_window_count_ == this->variation_window_.size()) {
    const VariationSample &oldest =
        this->variation_window_[this->variation_window_next_];
    this->variation_sum_ -= oldest.value;
    if (oldest.value == this->variation_max_) {
      this->variation_max_dirty_ = true;
    }
  } else {
    this->variation_window_count_++;
  }

  this->variation_window_[this->variation_window_next_] =
      VariationSample{variation, now};
  this->variation_window_next_ =
      (this->variation_window_next_ + 1) % this->variation_window_.size();
  this->variation_sum_ += variation;

  if (!this->variation_max_dirty_ && variation > this->variation_max_) {
    this->variation_max_ = variation;
  }

  this->prune_variation_samples_(now);
}


void ESPWiFiSensing::prune_variation_samples_(uint32_t now) {
  if (this->variation_window_.empty()) {
    return;
  }

  while (this->variation_window_count_ > 0) {
    const uint32_t oldest_index =
        (this->variation_window_next_ + this->variation_window_.size() -
         this->variation_window_count_) % this->variation_window_.size();
    const VariationSample &oldest = this->variation_window_[oldest_index];

    if (now - oldest.timestamp <= this->statistics_window_ms_) {
      break;
    }

    this->variation_sum_ -= oldest.value;
    if (oldest.value == this->variation_max_) {
      this->variation_max_dirty_ = true;
    }
    this->variation_window_count_--;
  }

  if (this->variation_window_count_ == 0) {
    this->variation_sum_ = 0;
    this->variation_max_ = 0;
    this->variation_max_dirty_ = false;
    return;
  }

  if (this->variation_max_dirty_) {
    uint32_t max_value = 0;
    for (uint32_t i = 0; i < this->variation_window_count_; i++) {
      const uint32_t index =
          (this->variation_window_next_ + this->variation_window_.size() -
           this->variation_window_count_ + i) % this->variation_window_.size();
      if (this->variation_window_[index].value > max_value) {
        max_value = this->variation_window_[index].value;
      }
    }
    this->variation_max_ = max_value;
    this->variation_max_dirty_ = false;
  }
}


uint32_t ESPWiFiSensing::evaluate_transient_pulse_candidate_(
    bool above_threshold, uint32_t average_variation, float decision_threshold, uint32_t now) {
  if (decision_threshold <= 0.0f) {
    return above_threshold ? 1 : 0;
  }

  if (above_threshold) {
    const float excess =
        (static_cast<float>(average_variation) - decision_threshold) / decision_threshold;
    if (!this->transient_pulse_test_.active) {
      const float baseline_mean = this->adaptive_baseline_.baseline_mean();
      this->transient_pulse_test_.ratio_start = baseline_mean > 0.0f ?
          this->adaptive_baseline_.baseline_stddev() / baseline_mean : 0.0f;
      this->transient_pulse_test_.threshold_start = decision_threshold;
      this->transient_pulse_test_.start_time = now;
    }
    this->transient_pulse_test_.active = true;
    this->transient_pulse_test_.area +=
        excess * (static_cast<float>(this->statistics_update_ms_) / 1000.0f);
    this->transient_pulse_test_.elapsed_ms += this->statistics_update_ms_;
    if (excess > this->transient_pulse_test_.peak) {
      this->transient_pulse_test_.peak = excess;
    }
    return 0;
  }

  if (!this->transient_pulse_test_.active) {
    return 0;
  }

  const float peak = this->transient_pulse_test_.peak;
  const float area = this->transient_pulse_test_.area;
  const float equivalent_width = peak > 0.0f ? area / peak : 0.0f;
  const uint32_t accepted_crossings =
      this->transient_pulse_test_.elapsed_ms / this->statistics_update_ms_;
  const bool transient_pass = equivalent_width >= 3.4f;

  ESP_LOGI(
      TAG,
      "Transient pulse test: peak=%.3f area=%.3f equivalent_width=%.3f result=%s",
      peak,
      area,
      equivalent_width,
      transient_pass ? "PASS" : "REJECT"
  );

  const float baseline_mean = this->adaptive_baseline_.baseline_mean();
  const float ratio_end = baseline_mean > 0.0f ?
      this->adaptive_baseline_.baseline_stddev() / baseline_mean : 0.0f;
  const float delta_ratio = ratio_end - this->transient_pulse_test_.ratio_start;
  const uint32_t event_duration_ms = now - this->transient_pulse_test_.start_time;
  const float duration_seconds =
      static_cast<float>(event_duration_ms > 0 ?
          event_duration_ms : this->transient_pulse_test_.elapsed_ms) / 1000.0f;
  const bool duration_valid = duration_seconds > 0.0f;
  const float threshold_rise_rate = duration_valid ?
      (decision_threshold - this->transient_pulse_test_.threshold_start) / duration_seconds : 0.0f;
  const bool baseline_rise_reject = transient_pass && delta_ratio >= 0.07f &&
      threshold_rise_rate >= 25.0f;
  const bool slow_threshold_rise_reject =
      transient_pass && !baseline_rise_reject && duration_valid && threshold_rise_rate < 40.0f;
  const bool pass = transient_pass && !baseline_rise_reject && !slow_threshold_rise_reject;

  if (transient_pass) {
    ESP_LOGI(
        TAG,
        "Baseline-rise test: W_eq=%.3f R_start=%.3f R_end=%.3f delta_R=%.3f "
        "threshold_start=%.3f threshold_end=%.3f threshold_rise_rate=%.3f "
        "duration=%.3f result=%s reason=baseline-variability/threshold-rise",
        equivalent_width,
        this->transient_pulse_test_.ratio_start,
        ratio_end,
        delta_ratio,
        this->transient_pulse_test_.threshold_start,
        decision_threshold,
        threshold_rise_rate,
        duration_seconds,
        baseline_rise_reject ? "REJECT" : "PASS"
    );
  }

  if (transient_pass && !baseline_rise_reject) {
    ESP_LOGI(
        TAG,
        "Slow-threshold-rise test: W_eq=%.3f delta_R=%.3f threshold_start=%.3f "
        "threshold_end=%.3f threshold_rise_rate=%.3f duration=%.3f "
        "Filter1=PASS Filter2=PASS Filter3=%s final=%s%s",
        equivalent_width,
        delta_ratio,
        this->transient_pulse_test_.threshold_start,
        decision_threshold,
        threshold_rise_rate,
        duration_seconds,
        slow_threshold_rise_reject ? "REJECT" : "PASS",
        pass ? "PASS" : "REJECT",
        slow_threshold_rise_reject ? " reason=slow-threshold-rise" : ""
    );
  }

  if (!pass) {
    ESP_LOGI(TAG, "Transient pulse REJECTED before motion decision");
    if (this->rejected_peak_sensor_ != nullptr) {
      this->rejected_peak_sensor_->publish_state(equivalent_width);
    }
  }

  this->reset_transient_pulse_test_();
  return pass ? accepted_crossings : 0;
}


void ESPWiFiSensing::reset_transient_pulse_test_() {
  this->transient_pulse_test_ = TransientPulseTestState{};
}


bool ESPWiFiSensing::warmup_active_(uint32_t now) const {
  return this->warmup_time_ms_ > 0 && now < this->warmup_time_ms_;
}


bool ESPWiFiSensing::apply_motion_hold_(bool motion_detected, uint32_t now) {
  if (motion_detected) {
    this->last_motion_time_ = now;
    return true;
  }

  if (this->motion_hold_time_ms_ == 0 || !this->motion_state_) {
    return false;
  }

  return now - this->last_motion_time_ < this->motion_hold_time_ms_;
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

  if (!this->driver_.start(this, ESPWiFiSensing::csi_callback_)) {
    return false;
  }

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
  // Métrica experimental para o teste.
  //
  // O algoritmo selecionado processa o pacote CSI após os
  // estágios comuns da pipeline.
  //
  // NÃO é ainda um algoritmo de presença.
  // Queremos apenas descobrir se a métrica reage fisicamente
  // quando alguém altera o caminho do sinal Wi-Fi.
  // -------------------------------------------------------

  const int8_t *buf =
      reinterpret_cast<const int8_t *>(data->buf);

  CsiPacket packet{};
  packet.len = data->len;
  packet.raw_bytes = buf;
  packet.rx_ctrl = &data->rx_ctrl;
  packet.first_word_invalid = data->first_word_invalid;

  self->pipeline_.process_packet(packet);

  const ParsedCsiPacket &parsed_packet = self->pipeline_.latest_parsed_packet();

  uint32_t metric = 0;

  if (self->selected_algorithm_ == CsiAlgorithm::VARIANCE) {
    metric = self->variance_algorithm_.process(parsed_packet);
  } else {
    metric = self->absolute_sum_algorithm_.process(parsed_packet);
  }

  self->latest_csi_metric_ = metric;
  self->latest_csi_len_ = self->pipeline_.latest_len();
  self->new_csi_sample_ = self->pipeline_.has_new_sample();
  self->pipeline_.clear_new_sample();
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  STEP 5 - CSI variation");
  ESP_LOGCONFIG(TAG, "  CSI callback: ENABLED");
  ESP_LOGCONFIG(TAG, "  Router ping: 10 pings/s");
  ESP_LOGCONFIG(
      TAG,
      "  Algorithm: %s",
      this->selected_algorithm_ == CsiAlgorithm::VARIANCE ? "Variance" : "Absolute Sum"
  );
  ESP_LOGCONFIG(
      TAG,
      "  Metric: %s",
      this->selected_algorithm_ == CsiAlgorithm::VARIANCE ? "temporal CSI variance" : "absolute CSI sum"
  );
  ESP_LOGCONFIG(TAG, "  Gain compensation: %s", this->gain_compensation_enabled_ ? "ENABLED" : "disabled");
  ESP_LOGCONFIG(TAG, "  Adaptive Threshold: %s", this->adaptive_threshold_enabled_ ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  Sigma multiplier: %.2f", this->sigma_multiplier_);
  ESP_LOGCONFIG(TAG, "  Baseline rise time: %u ms", static_cast<unsigned>(this->baseline_rise_time_ms_));
  ESP_LOGCONFIG(TAG, "  Baseline fall time: %u ms", static_cast<unsigned>(this->baseline_fall_time_ms_));
  ESP_LOGCONFIG(TAG, "  Learning delay: %u ms", static_cast<unsigned>(this->learning_delay_ms_));
  ESP_LOGCONFIG(TAG, "  Warm-up time: %u ms", static_cast<unsigned>(this->warmup_time_ms_));
  ESP_LOGCONFIG(TAG, "  Motion hold time: %u ms", static_cast<unsigned>(this->motion_hold_time_ms_));
  ESP_LOGCONFIG(TAG, "  Statistics window: %u ms", static_cast<unsigned>(this->statistics_window_ms_));
  ESP_LOGCONFIG(TAG, "  Statistics update: %u ms", static_cast<unsigned>(this->statistics_update_ms_));
  ESP_LOGCONFIG(TAG, "  Threshold: %u", static_cast<unsigned>(this->motion_threshold_));
  ESP_LOGCONFIG(TAG, "  Debounce: %u", static_cast<unsigned>(this->motion_debounce_));
  LOG_SENSOR("  ", "CSI Metric", this->metric_sensor_);
  LOG_SENSOR("  ", "CSI Variation Avg", this->variation_avg_sensor_);
  LOG_SENSOR("  ", "Baseline Mean", this->baseline_mean_sensor_);
  LOG_SENSOR("  ", "Baseline StdDev", this->baseline_stddev_sensor_);
  LOG_SENSOR("  ", "Adaptive Threshold", this->adaptive_threshold_sensor_);
  LOG_SENSOR("  ", "Rejected Peak", this->rejected_peak_sensor_);
  LOG_BINARY_SENSOR("  ", "CSI Motion", this->motion_binary_sensor_);
  ESP_LOGCONFIG(TAG, "  esp-radar processing: NOT STARTED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
