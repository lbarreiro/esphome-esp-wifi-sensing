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
 *      * Accumulates statistics (sum, max, count)
 *    - Phase 4: Report statistics window
 *      * Publishes aggregated metrics every 5 seconds
 *      * Resets variation accumulators after reporting
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
  this->gain_compensation_ready_ = this->pipeline_.gain_compensation_ready();
}


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "STEP 5 - CSI variation test");
  this->pipeline_.set_gain_compensation_enabled(this->gain_compensation_enabled_);
  this->gain_compensation_ready_ = this->pipeline_.gain_compensation_ready();
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

  if (this->startup_warmup_active_) {
    this->have_previous_sample_ = false;
    this->variation_sum_ = 0;
    this->variation_max_ = 0;
    this->variation_samples_ = 0;

    if (this->gain_compensation_ready_) {
      this->startup_warmup_active_ = false;
      this->last_report_time_ = millis();
    }
  }

  if (this->new_csi_sample_) {
    const uint32_t metric = this->latest_csi_metric_;
    const bool have_previous_before = this->have_previous_sample_;
    const uint32_t previous_metric_before = this->previous_csi_metric_;
    const uint32_t variation_sum_before = this->variation_sum_;
    const uint32_t variation_samples_before = this->variation_samples_;
    const uint32_t variation_max_before = this->variation_max_;
    bool variation_block_entered = false;
    uint32_t variation = 0;

    this->new_csi_sample_ = false;

    if (!this->startup_warmup_active_) {
      if (this->have_previous_sample_) {
        variation_block_entered = true;

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

    ESP_LOGI(
        TAG,
        "VAR_PIPELINE new_csi_sample=true startup_warmup_active=%s gain_compensation_ready=%s "
        "have_previous_before=%s latest_csi_metric=%u previous_csi_metric=%u "
        "variation_block_entered=%s variation=%u variation_sum_before=%u variation_sum_after=%u "
        "variation_samples_before=%u variation_samples_after=%u variation_max_before=%u variation_max_after=%u "
        "have_previous_after=%s",
        this->startup_warmup_active_ ? "true" : "false",
        this->gain_compensation_ready_ ? "true" : "false",
        have_previous_before ? "true" : "false",
        static_cast<unsigned>(metric),
        static_cast<unsigned>(previous_metric_before),
        variation_block_entered ? "true" : "false",
        static_cast<unsigned>(variation),
        static_cast<unsigned>(variation_sum_before),
        static_cast<unsigned>(this->variation_sum_),
        static_cast<unsigned>(variation_samples_before),
        static_cast<unsigned>(this->variation_samples_),
        static_cast<unsigned>(variation_max_before),
        static_cast<unsigned>(this->variation_max_),
        this->have_previous_sample_ ? "true" : "false"
    );
  } else {
    ESP_LOGI(
        TAG,
        "VAR_PIPELINE new_csi_sample=false startup_warmup_active=%s gain_compensation_ready=%s "
        "have_previous_before=%s latest_csi_metric=%u previous_csi_metric=%u variation_block_entered=false "
        "variation=0 variation_sum_before=%u variation_sum_after=%u variation_samples_before=%u "
        "variation_samples_after=%u variation_max_before=%u variation_max_after=%u have_previous_after=%s",
        this->startup_warmup_active_ ? "true" : "false",
        this->gain_compensation_ready_ ? "true" : "false",
        this->have_previous_sample_ ? "true" : "false",
        static_cast<unsigned>(this->latest_csi_metric_),
        static_cast<unsigned>(this->previous_csi_metric_),
        static_cast<unsigned>(this->variation_sum_),
        static_cast<unsigned>(this->variation_sum_),
        static_cast<unsigned>(this->variation_samples_),
        static_cast<unsigned>(this->variation_samples_),
        static_cast<unsigned>(this->variation_max_),
        static_cast<unsigned>(this->variation_max_),
        this->have_previous_sample_ ? "true" : "false"
    );
  }

  // -------------------------------------------------------
  // 4. Relatório a cada 5 segundos
  // -------------------------------------------------------

  const uint32_t now = millis();

  if (now - this->last_report_time_ < 5000) {
    ESP_LOGI(TAG, "RETURN: report_interval_not_elapsed");
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

  if (this->metric_sensor_ != nullptr) {
    this->metric_sensor_->publish_state(this->latest_csi_metric_);
  }

  if (this->variation_avg_sensor_ != nullptr) {
    this->variation_avg_sensor_->publish_state(average_variation);
  }

  if (this->startup_warmup_active_) {
    this->variation_sum_ = 0;
    this->variation_max_ = 0;
    this->variation_samples_ = 0;
    ESP_LOGI(TAG, this->gain_compensation_ready_ ? "RETURN: warmup_active" : "RETURN: gain_compensation_not_ready");
    return;
  }

  if (this->adaptive_threshold_enabled_ && !this->adaptive_baseline_.initialized()) {
    this->adaptive_baseline_.update(static_cast<float>(average_variation), false, now, 5000);
  }

  const float decision_threshold = this->adaptive_threshold_enabled_ ?
      this->adaptive_baseline_.adaptive_threshold() :
      static_cast<float>(this->motion_threshold_);

  if (static_cast<float>(average_variation) > decision_threshold) {
    this->consecutive_above_threshold_++;
    this->motion_state_ =
        this->consecutive_above_threshold_ >= this->motion_debounce_;
  } else {
    this->consecutive_above_threshold_ = 0;
    this->motion_state_ = false;
  }

  if (this->adaptive_threshold_enabled_) {
    this->adaptive_baseline_.update(
        static_cast<float>(average_variation),
        this->motion_state_,
        now,
        5000
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

  if (this->motion_binary_sensor_ != nullptr) {
    this->motion_binary_sensor_->publish_state(this->motion_state_);
  }

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
  self->gain_compensation_ready_ = self->pipeline_.gain_compensation_ready();

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
  ESP_LOGCONFIG(TAG, "  Threshold: %u", static_cast<unsigned>(this->motion_threshold_));
  ESP_LOGCONFIG(TAG, "  Debounce: %u", static_cast<unsigned>(this->motion_debounce_));
  LOG_SENSOR("  ", "CSI Metric", this->metric_sensor_);
  LOG_SENSOR("  ", "CSI Variation Avg", this->variation_avg_sensor_);
  LOG_SENSOR("  ", "Baseline Mean", this->baseline_mean_sensor_);
  LOG_SENSOR("  ", "Baseline StdDev", this->baseline_stddev_sensor_);
  LOG_SENSOR("  ", "Adaptive Threshold", this->adaptive_threshold_sensor_);
  LOG_BINARY_SENSOR("  ", "CSI Motion", this->motion_binary_sensor_);
  ESP_LOGCONFIG(TAG, "  esp-radar processing: NOT STARTED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
