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
  ESP_LOGI(TAG, "CSI sensing initialization");
  this->pipeline_.set_gain_compensation_enabled(this->gain_compensation_enabled_);
  if (this->selected_algorithm_ == CsiAlgorithm::MVS && this->motion_binary_sensor_ != nullptr) {
    this->motion_binary_sensor_->publish_state(false);
    this->mvs_initial_state_published_ = true;
  }
  if (this->selected_algorithm_ == CsiAlgorithm::MVS && this->motion_128_binary_sensor_ != nullptr) {
    this->motion_128_binary_sensor_->publish_state(false);
    this->mvs_128_initial_state_published_ = true;
  }
}


void ESPWiFiSensing::loop() {
  if (!this->csi_started_) {
    if (!this->start_csi_()) {
      delay(1000);
      return;
    }

    this->csi_started_ = true;
    ESP_LOGI(TAG, "CSI enabled");
  }

  if (!this->ping_started_) {
    if (!this->start_ping_()) {
      delay(1000);
      return;
    }

    this->ping_started_ = true;
    ESP_LOGI(TAG, "Router ping started");
  }

  const uint32_t now = millis();

  if (this->csi_length_last_log_ms_ == 0) this->csi_length_last_log_ms_ = now;
  if (static_cast<uint32_t>(now - this->csi_length_last_log_ms_) >= 60000U) {
    this->csi_length_last_log_ms_ = now;
    ESP_LOGI(TAG, "CSI cumulative packets: 128 bytes=%u | 256 bytes=%u",
             static_cast<unsigned>(this->csi_128_packet_count_.load(std::memory_order_relaxed)),
             static_cast<unsigned>(this->csi_256_packet_count_.load(std::memory_order_relaxed)));
  }

  if (this->selected_algorithm_ == CsiAlgorithm::MVS) {
    if (motion_3_binary_sensor_ != nullptr) {
      const bool seen_3 = mvs_3_confirmation_seen_.load(std::memory_order_acquire);
      const uint32_t last_3 = mvs_3_last_confirmation_ms_.load(std::memory_order_relaxed);
      const bool active_3 = seen_3 && static_cast<uint32_t>(now - last_3) < motion_hold_time_ms_;
      if (!mvs_3_initial_state_published_ || active_3 != mvs_3_published_state_) {
        motion_3_binary_sensor_->publish_state(active_3);
        mvs_3_published_state_ = active_3;
        mvs_3_initial_state_published_ = true;
      }
      if (static_cast<uint32_t>(now - learn_last_log_ms_) >= 60000U) {
        learn_last_log_ms_ = now;
        ESP_LOGI(TAG, "MVS3 learn q=%u ep=%u reject=%u rev=%u hi=%u:%.2f/%.2f lo=%u:%.2f/%.2f (candidate/weight)",
          static_cast<unsigned>(learn_quiet_.load()), static_cast<unsigned>(learn_episodes_.load()),
          static_cast<unsigned>(learn_rejected_.load()), static_cast<unsigned>(learn_revisions_.load()),
          static_cast<unsigned>(learn_best_.load()), learn_best_candidate_.load(), learn_best_weight_.load(),
          static_cast<unsigned>(learn_low_.load()), learn_low_candidate_.load(), learn_low_weight_.load());
        ESP_LOGI(TAG, "MVS3 diag obs=%u held=%u score=%.3f energy=%.6f limit=%.6f rawquiet=%u good=%u/%u discard=%u holdreset=%u gaps=%u",
          static_cast<unsigned>(learn_observations_.load()), static_cast<unsigned>(learn_held_.load()),
          learn_score_.load(), learn_energy_.load(), learn_limit_.load(),
          static_cast<unsigned>(learn_raw_quiet_.load()), static_cast<unsigned>(learn_good_.load()),
          static_cast<unsigned>(learn_count_.load()), static_cast<unsigned>(learn_discarded_.load()),
          static_cast<unsigned>(learn_hold_resets_.load()), static_cast<unsigned>(learn_gaps_.load()));
      }
    }
    // Independent 128-byte comparison output: never affects the 256-byte decision.
    if (this->motion_128_binary_sensor_ != nullptr) {
      const bool seen_128 = this->mvs_128_confirmation_seen_.load(std::memory_order_acquire);
      const uint32_t last_128 = this->mvs_128_last_confirmation_ms_.load(std::memory_order_relaxed);
      const bool active_128 = seen_128 && static_cast<uint32_t>(now - last_128) < this->motion_hold_time_ms_;
      if (!this->mvs_128_initial_state_published_) {
        this->motion_128_binary_sensor_->publish_state(false);
        this->mvs_128_initial_state_published_ = true;
      }
      if (active_128 != this->mvs_128_published_state_) {
        this->mvs_128_published_state_ = active_128;
        ESP_LOGI(TAG, "MVS 128-byte comparison motion %s (hold=%u ms)", active_128 ? "ON" : "OFF",
                 static_cast<unsigned>(this->motion_hold_time_ms_));
        this->motion_128_binary_sensor_->publish_state(active_128);
      }
    }
    // Hold expiry must run on every ESPHome loop, including when CSI stops.
    const bool seen = this->mvs_confirmation_seen_.load(std::memory_order_acquire);
    const uint32_t last = this->mvs_last_confirmation_ms_.load(std::memory_order_relaxed);
    const bool active = seen && static_cast<uint32_t>(now - last) < this->motion_hold_time_ms_;
    if (!this->mvs_initial_state_published_ && this->motion_binary_sensor_ != nullptr) {
      this->motion_binary_sensor_->publish_state(false);
      this->mvs_initial_state_published_ = true;
    }
    if (active != this->mvs_published_state_) {
      this->mvs_published_state_ = active;
      ESP_LOGI(TAG, "MVS motion %s (hold=%u ms)", active ? "ON" : "OFF",
               static_cast<unsigned>(this->motion_hold_time_ms_));
      if (this->motion_binary_sensor_ != nullptr) this->motion_binary_sensor_->publish_state(active);
    }
    if (seen && last != this->mvs_last_logged_confirmation_ms_) {
      // Throttle refresh logs while retaining the actual timestamp on every confirmation.
      if (static_cast<uint32_t>(now - this->mvs_last_logged_confirmation_ms_) >= 10000U) {
        ESP_LOGD(TAG, "MVS hold refreshed (last confirmation=%u)", static_cast<unsigned>(last));
        this->mvs_last_logged_confirmation_ms_ = last;
      }
    }
    const uint32_t last_packet = this->mvs_last_packet_ms_.load(std::memory_order_relaxed);
    const uint32_t last_accepted = this->mvs_last_accepted_ms_.load(std::memory_order_relaxed);
    const uint32_t last_observation = this->mvs_last_observation_ms_.load(std::memory_order_relaxed);
    if (this->mvs_diagnostic_started_ms_ == 0) this->mvs_diagnostic_started_ms_ = now;
    const uint32_t packet_age = now - (last_packet ? last_packet : this->mvs_diagnostic_started_ms_);
    const uint32_t accepted_age = now - (last_accepted ? last_accepted : this->mvs_diagnostic_started_ms_);
    const uint32_t observation_age = now - (last_observation ? last_observation : this->mvs_diagnostic_started_ms_);
    if (observation_age > 5000U && static_cast<uint32_t>(now - this->mvs_last_stall_log_ms_) >= 30000U) {
      if (packet_age > 5000U) {
        ESP_LOGW(TAG, "MVS CSI stalled: no CSI packets for %u ms", static_cast<unsigned>(packet_age));
      } else if (accepted_age > 5000U) {
        ESP_LOGW(TAG, "MVS CSI stalled: packets received but unsupported/rejected for %u ms (rejected=%u)",
                 static_cast<unsigned>(accepted_age), static_cast<unsigned>(this->mvs_rejected_packets_.load(std::memory_order_relaxed)));
      } else {
        ESP_LOGW(TAG, "MVS CSI stalled: valid packets received but no observations for %u ms",
                 static_cast<unsigned>(observation_age));
      }
      this->mvs_last_stall_log_ms_ = now;
    }
    this->new_csi_sample_ = false;
    return;
  }

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

  if (now - this->last_report_time_ < this->statistics_update_ms_) {
    return;
  }

  this->last_report_time_ = now;
  const uint32_t current = this->csi_packet_count_;
  const uint32_t received = current - this->last_reported_count_;
  this->last_reported_count_ = current;

  uint32_t average_variation = 0;
  this->prune_variation_samples_(now);
  if (this->variation_window_count_ > 0) {
    average_variation = this->variation_sum_ / this->variation_window_count_;
  }

  ESP_LOGI(TAG, "CSI: packets=%u/%us len=%u metric=%u variation avg=%u max=%u samples=%u",
           static_cast<unsigned>(received), static_cast<unsigned>(this->statistics_update_ms_ / 1000),
           static_cast<unsigned>(this->latest_csi_len_), static_cast<unsigned>(this->latest_csi_metric_),
           static_cast<unsigned>(average_variation), static_cast<unsigned>(this->variation_max_),
           static_cast<unsigned>(this->variation_window_count_));


  if (this->warmup_active_(now)) {
    this->consecutive_above_threshold_ = 0;
    this->motion_state_ = false;
    if (this->motion_binary_sensor_ != nullptr) this->motion_binary_sensor_->publish_state(false);
    return;
  }

  if (this->adaptive_threshold_enabled_ && !this->adaptive_baseline_.initialized()) {
    this->adaptive_baseline_.update(static_cast<float>(average_variation), false, now, this->statistics_update_ms_);
  }

  const float decision_threshold = this->adaptive_threshold_enabled_ ? this->adaptive_baseline_.adaptive_threshold() : this->motion_threshold_;
  bool motion_detected = false;
  if (static_cast<float>(average_variation) > decision_threshold) {
    this->consecutive_above_threshold_++;
    motion_detected = this->consecutive_above_threshold_ >= this->motion_debounce_;
  } else {
    this->consecutive_above_threshold_ = 0;
  }

  if (this->adaptive_threshold_enabled_) {
    this->adaptive_baseline_.update(static_cast<float>(average_variation), motion_detected, now, this->statistics_update_ms_);
  }

  this->motion_state_ = this->apply_motion_hold_(motion_detected, now);
  if (this->motion_binary_sensor_ != nullptr) this->motion_binary_sensor_->publish_state(this->motion_state_);
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
  #if LWIP_IPV6
  target_addr.type = IPADDR_TYPE_V4;
  #endif

  ip4_addr_set_u32(
      ip_2_ip4(&target_addr),
      ip_info.gw.addr
  );

  ping_config.target_addr = target_addr;

  ping_config.count = ESP_PING_COUNT_INFINITE;

  // Mantemos exatamente os 10 pings/s do Passo 4.
  ping_config.interval_ms = this->ping_interval_ms_;
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

  ESP_LOGI(TAG, "Router ping running: interval=%u ms", static_cast<unsigned>(this->ping_interval_ms_));

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
  if (data->len == 128) self->csi_128_packet_count_.fetch_add(1, std::memory_order_relaxed);
  if (data->len == 256) self->csi_256_packet_count_.fetch_add(1, std::memory_order_relaxed);
  if (self->selected_algorithm_ == CsiAlgorithm::MVS)
    self->mvs_last_packet_ms_.store(millis(), std::memory_order_relaxed);

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

  if (self->selected_algorithm_ == CsiAlgorithm::MVS) {
    const uint32_t sample_now = millis();
    if (parsed_packet.layout_supported && parsed_packet.count >= MvsAlgorithm::kBins)
      self->mvs_last_accepted_ms_.store(sample_now, std::memory_order_relaxed);
    else
      self->mvs_rejected_packets_.fetch_add(1, std::memory_order_relaxed);
    // Primary pipeline still rejects 128-byte packets. Separate CSI profile and
    // calibration for the comparison sensor, using the same configured parameters.
    if (packet.len == 128 && self->motion_128_binary_sensor_ != nullptr) {
      const ParsedCsiPacket &parsed_128 = self->csi_128_parser_.parse(self->pipeline_.latest_packet(), true);
      const MvsResult result_128 = self->mvs_128_algorithm_.process(parsed_128, sample_now);
      if (result_128.motion) {
        self->mvs_128_last_confirmation_ms_.store(self->mvs_128_algorithm_.last_motion_time(),
                                                  std::memory_order_relaxed);
        self->mvs_128_confirmation_seen_.store(true, std::memory_order_release);
      }
    }
    const MvsResult result = self->mvs_algorithm_.process(parsed_packet, sample_now);
    if (result.observation_added) self->mvs_last_observation_ms_.store(sample_now, std::memory_order_relaxed);
    // Only actual confirmed detections refresh the hold, never stale CSI state.
    if (result.motion) {
      const uint32_t confirmed_at = self->mvs_algorithm_.last_motion_time();
      self->mvs_last_confirmation_ms_.store(confirmed_at, std::memory_order_relaxed);
      self->mvs_confirmation_seen_.store(true, std::memory_order_release);
    }
    self->motion_state_ = result.motion;
    self->latest_csi_metric_ = static_cast<uint32_t>(result.score * 100.0f);
    // Third branch parses ORIGINAL bytes, before gain compensation. It cannot
    // inspect result, primary motion state, or the 128-byte comparison state.
    if (packet.len == 256 && self->motion_3_binary_sensor_ != nullptr) {
      const auto &raw_256 = self->csi_3_parser_.parse(packet);
      const auto third = self->mvs_3_algorithm_.process(raw_256, sample_now);
      if (third.motion) {
        self->mvs_3_last_confirmation_ms_.store(self->mvs_3_algorithm_.last_motion_time(), std::memory_order_relaxed);
        self->mvs_3_confirmation_seen_.store(true, std::memory_order_release);
      }
      if (third.observation_added) {
        const auto &learn = self->mvs_3_algorithm_.learning();
        self->learn_quiet_.store(learn.quiet_blocks());
        self->learn_episodes_.store(learn.episodes());
        self->learn_rejected_.store(learn.rejected());
        self->learn_revisions_.store(learn.revisions());
        self->learn_discarded_.store(learn.discarded_blocks());
        self->learn_hold_resets_.store(learn.hold_resets());
        self->learn_gaps_.store(learn.discontinuities());
        self->learn_good_.store(learn.quiet_good());
        self->learn_count_.store(learn.quiet_count());
        self->learn_energy_.store(learn.last_total());
        self->learn_limit_.store(learn.quiet_limit());
        self->learn_raw_quiet_.store(learn.last_raw_quiet());
        self->learn_held_.store(self->mvs_3_algorithm_.motion_state());
        self->learn_score_.store(third.score);
        self->learn_observations_.store(static_cast<uint32_t>(self->mvs_3_algorithm_.observation_count()));
        size_t hi = 0, lo = 0;
        for (size_t i = 1; i < CsiLearning::kBins; ++i) {
          if (learn.candidate(i) > learn.candidate(hi)) hi = i;
          if (learn.candidate(i) < learn.candidate(lo)) lo = i;
        }
        self->learn_best_.store(hi); self->learn_low_.store(lo);
        self->learn_best_candidate_.store(learn.candidate(hi)); self->learn_best_weight_.store(learn.weights()[hi]);
        self->learn_low_candidate_.store(learn.candidate(lo)); self->learn_low_weight_.store(learn.weights()[lo]);
      }
    }
  } else if (self->selected_algorithm_ == CsiAlgorithm::VARIANCE) {
    self->latest_csi_metric_ = self->variance_algorithm_.process(parsed_packet);
  } else {
    self->latest_csi_metric_ = self->absolute_sum_algorithm_.process(parsed_packet);
  }

  self->latest_csi_len_ = self->pipeline_.latest_len();
  self->new_csi_sample_ = self->pipeline_.has_new_sample();
  self->pipeline_.clear_new_sample();
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  Algorithm: %s",
                this->selected_algorithm_ == CsiAlgorithm::MVS ? "MVS" :
                (this->selected_algorithm_ == CsiAlgorithm::VARIANCE ? "Variance" : "Absolute Sum"));
  ESP_LOGCONFIG(TAG, "  Router ping: interval=%u ms (%.2f pings/s)",
                static_cast<unsigned>(this->ping_interval_ms_), 1000.0f / this->ping_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Gain compensation: %s", this->gain_compensation_enabled_ ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  Threshold: %.2f", this->motion_threshold_);
  ESP_LOGCONFIG(TAG, "  Motion hold: %u ms", static_cast<unsigned>(this->motion_hold_time_ms_));
  if (this->selected_algorithm_ == CsiAlgorithm::MVS) {
    ESP_LOGCONFIG(TAG, "  MVS renewal: fixed 2 peaks per window");
    ESP_LOGCONFIG(TAG, "  CSI 256: primary motion output");
    ESP_LOGCONFIG(TAG, "  CSI 128: independent comparison output (%s)",
                  this->motion_128_binary_sensor_ != nullptr ? "enabled" : "disabled");
  } else {
    ESP_LOGCONFIG(TAG, "  Adaptive threshold: %s", this->adaptive_threshold_enabled_ ? "enabled" : "disabled");
    ESP_LOGCONFIG(TAG, "  Sigma multiplier: %.2f", this->sigma_multiplier_);
    ESP_LOGCONFIG(TAG, "  Baseline rise: %u ms", static_cast<unsigned>(this->baseline_rise_time_ms_));
    ESP_LOGCONFIG(TAG, "  Baseline fall: %u ms", static_cast<unsigned>(this->baseline_fall_time_ms_));
    ESP_LOGCONFIG(TAG, "  Learning delay: %u ms", static_cast<unsigned>(this->learning_delay_ms_));
    ESP_LOGCONFIG(TAG, "  Warm-up: %u ms", static_cast<unsigned>(this->warmup_time_ms_));
    ESP_LOGCONFIG(TAG, "  Statistics window: %u ms", static_cast<unsigned>(this->statistics_window_ms_));
    ESP_LOGCONFIG(TAG, "  Statistics update: %u ms", static_cast<unsigned>(this->statistics_update_ms_));
    ESP_LOGCONFIG(TAG, "  Debounce: %u", static_cast<unsigned>(this->motion_debounce_));
  }
  LOG_BINARY_SENSOR("  ", "CSI Motion 256 Learned", this->motion_3_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "CSI Motion 256", this->motion_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "CSI Motion 128 Comparison", this->motion_128_binary_sensor_);
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
