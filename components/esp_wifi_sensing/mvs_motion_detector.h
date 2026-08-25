#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "csi_packet.h"
#include "mvs_ml_weights.h"

namespace esphome {
namespace esp_wifi_sensing {

struct MvsMotionResult {
  bool active{false};
  float variance{0.0f};
  float threshold{5.0f};
  float baseline{0.0f};
  float change_rate{0.0f};
  float spatial_change{0.0f};
  float coherence{1.0f};
  float feature_score{0.0f};
};

// MVS-ML: our MVS implementation of the statistical-ML pipeline used by
// ESPectre ML. The detector keeps the MVS public name/API, but the decision
// path is now: fixed 12 subcarriers -> spatial turbulence -> Hampel ->
// 100-sample window -> 9 statistical features -> 9x32x16x1 MLP.
// No ESPectre runtime code is linked; inference is implemented locally.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t) {}
  void set_threshold_multiplier(float) {}
  void set_enter_hits(uint8_t value) { this->enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { this->exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { this->hold_time_ms_ = value; }

  MvsMotionResult update(const CsiPacket &packet, uint32_t now_ms) {
    const float turbulence = this->calculate_turbulence_(packet);
    if (!std::isfinite(turbulence)) return this->result_();

    const float filtered = this->hampel_(turbulence);
    this->turbulence_[this->write_index_] = filtered;
    this->write_index_ = (this->write_index_ + 1) % kWindow;
    if (this->count_ < kWindow) ++this->count_;
    ++this->packet_counter_;

    this->latest_turbulence_ = filtered;
    if (this->count_ < kWindow) return this->result_();

    // ESPectre evaluates every 25 packets; publication remains controlled by
    // the ESPHome diagnostic rate limiter outside this detector.
    if ((this->packet_counter_ % kEvaluationInterval) != 0) return this->result_();

    float ordered[kWindow];
    for (uint16_t i = 0; i < kWindow; ++i)
      ordered[i] = this->turbulence_[(this->write_index_ + i) % kWindow];

    float features[9];
    this->extract_features_(ordered, features);
    const float probability = this->predict_(features);
    const float score = probability * 10.0f;

    const float previous_score = this->feature_score_;
    this->feature_score_ = score;
    this->change_rate_ = score - previous_score;

    // Diagnostics retain useful names without feeding the old hand-tuned FSM.
    this->variance_ = features[1] * features[1];
    this->spatial_change_ = features[8];
    this->coherence_ = features[6];
    this->baseline_ = features[0];
    this->threshold_ = this->threshold_;

    const bool inference_motion = score >= this->threshold_;

    if (inference_motion) {
      if (this->enter_hits_count_ < this->enter_hits_) ++this->enter_hits_count_;
      this->exit_hits_count_ = 0;
    } else {
      if (this->exit_hits_count_ < this->exit_hits_) ++this->exit_hits_count_;
      this->enter_hits_count_ = 0;
    }

    // Match the ESPectre edge policy: consecutive evaluation hits decide the
    // edge. The existing 60s hold time is retained for this project.
    if (!this->active_ && this->enter_hits_count_ >= this->enter_hits_) {
      this->active_ = true;
      this->hold_until_ms_ = now_ms + this->hold_time_ms_;
      this->enter_hits_count_ = 0;
    }

    if (this->active_ && static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0 &&
        this->exit_hits_count_ >= this->exit_hits_) {
      this->active_ = false;
      this->exit_hits_count_ = 0;
    }

    return this->result_();
  }

 private:
  static constexpr uint16_t kWindow = 100;
  static constexpr uint16_t kEvaluationInterval = 25;
  static constexpr uint8_t kHampelWindow = 7;
  static constexpr float kHampelThreshold = 5.0f;
  static constexpr float kHampelScale = 1.4826f;

  // ESPectre's fixed C6/HT20 ML band: 12 non-consecutive, DC excluded.
  static constexpr uint8_t kSubcarriers[12] = {12, 14, 16, 18, 20, 24, 28, 36, 40, 44, 48, 52};

  float calculate_turbulence_(const CsiPacket &packet) const {
    if (packet.raw_bytes == nullptr) return NAN;
    // ESP CSI is I/Q interleaved. The repository's CSI parser exposes the
    // same raw byte order; each subcarrier occupies two signed int8 values.
    if (packet.len < 2 * 53) return NAN;

    float amplitudes[12];
    float sum = 0.0f;
    for (uint8_t i = 0; i < 12; ++i) {
      const uint16_t sc = kSubcarriers[i];
      const float q = static_cast<float>(packet.raw_bytes[2 * sc]);
      const float in_phase = static_cast<float>(packet.raw_bytes[2 * sc + 1]);
      const float amp = std::sqrt(q * q + in_phase * in_phase);
      amplitudes[i] = amp;
      sum += amp;
    }
    const float mean = sum / 12.0f;
    float variance = 0.0f;
    for (float amp : amplitudes) {
      const float d = amp - mean;
      variance += d * d;
    }
    // ML is trained on raw spatial std, not CV normalization.
    return std::sqrt(variance / 12.0f);
  }

  float hampel_(float value) {
    this->hampel_buffer_[this->hampel_index_] = value;
    this->hampel_index_ = (this->hampel_index_ + 1) % kHampelWindow;
    if (this->hampel_count_ < kHampelWindow) ++this->hampel_count_;
    if (this->hampel_count_ < 3) return value;

    float sorted[kHampelWindow];
    for (uint8_t i = 0; i < this->hampel_count_; ++i) sorted[i] = this->hampel_buffer_[i];
    std::sort(sorted, sorted + this->hampel_count_);
    const float median = sorted[this->hampel_count_ / 2];

    for (uint8_t i = 0; i < this->hampel_count_; ++i)
      sorted[i] = std::fabs(this->hampel_buffer_[i] - median);
    std::sort(sorted, sorted + this->hampel_count_);
    const float mad = sorted[this->hampel_count_ / 2];

    if (mad > 1e-6f && std::fabs(value - median) > kHampelThreshold * kHampelScale * mad)
      return median;
    return value;
  }

  static float median_(float *values, uint16_t n) {
    std::sort(values, values + n);
    return (n & 1) ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) * 0.5f;
  }

  static float percentile_(const float *sorted, uint16_t n, float p) {
    if (n == 0) return 0.0f;
    const float pos = (n - 1) * p;
    const uint16_t lo = static_cast<uint16_t>(pos);
    const uint16_t hi = lo + 1 < n ? lo + 1 : lo;
    return sorted[lo] + (sorted[hi] - sorted[lo]) * (pos - lo);
  }

  void extract_features_(const float *x, float *f) const {
    float mean = 0.0f, min_v = x[0], max_v = x[0];
    for (uint16_t i = 0; i < kWindow; ++i) {
      mean += x[i];
      if (x[i] < min_v) min_v = x[i];
      if (x[i] > max_v) max_v = x[i];
    }
    mean /= kWindow;

    float var = 0.0f;
    for (uint16_t i = 0; i < kWindow; ++i) {
      const float d = x[i] - mean;
      var += d * d;
    }
    var /= kWindow;
    const float stddev = std::sqrt(var);

    float sorted[kWindow];
    std::memcpy(sorted, x, sizeof(sorted));
    std::sort(sorted, sorted + kWindow);
    const float q25 = percentile_(sorted, kWindow, 0.25f);
    const float q75 = percentile_(sorted, kWindow, 0.75f);
    const float med = percentile_(sorted, kWindow, 0.50f);

    float abs_dev[kWindow];
    for (uint16_t i = 0; i < kWindow; ++i) abs_dev[i] = std::fabs(x[i] - med);
    const float mad = median_(abs_dev, kWindow);

    float m3 = 0.0f;
    if (stddev > 1e-10f) {
      for (uint16_t i = 0; i < kWindow; ++i) {
        const float d = x[i] - mean;
        m3 += d * d * d;
      }
      m3 /= kWindow;
      m3 /= (stddev * stddev * stddev);
    }

    float autocov = 0.0f;
    for (uint16_t i = 0; i < kWindow - 1; ++i)
      autocov += (x[i] - mean) * (x[i + 1] - mean);
    autocov /= (kWindow - 1);
    const float autocorr = var > 1e-10f ? autocov / var : 0.0f;

    float waveform = 0.0f;
    for (uint16_t i = 1; i < kWindow; ++i) waveform += std::fabs(x[i] - x[i - 1]);

    f[0] = mean;
    f[1] = stddev;
    f[2] = max_v;
    f[3] = min_v;
    f[4] = q75 - q25;
    f[5] = m3;
    f[6] = autocorr;
    f[7] = mad;
    f[8] = waveform;
  }

  float predict_(const float *features) const {
    float h1[32]{};
    float h2[16]{};

    float x[9];
    for (uint8_t i = 0; i < 9; ++i)
      x[i] = (features[i] - mvs_ml::FEATURE_MEAN[i]) / mvs_ml::FEATURE_SCALE[i];

    for (uint8_t j = 0; j < 32; ++j) {
      float v = mvs_ml::B1[j];
      for (uint8_t i = 0; i < 9; ++i) v += x[i] * mvs_ml::W1[i * 32 + j];
      h1[j] = v > 0.0f ? v : 0.0f;
    }
    for (uint8_t j = 0; j < 16; ++j) {
      float v = mvs_ml::B2[j];
      for (uint8_t i = 0; i < 32; ++i) v += h1[i] * mvs_ml::W2[i * 16 + j];
      h2[j] = v > 0.0f ? v : 0.0f;
    }
    float z = mvs_ml::B3[0];
    for (uint8_t i = 0; i < 16; ++i) z += h2[i] * mvs_ml::W3[i];
    z = std::fmax(-40.0f, std::fmin(40.0f, z));
    return 1.0f / (1.0f + std::exp(-z));
  }

  MvsMotionResult result_() const {
    MvsMotionResult r;
    r.active = this->active_;
    r.variance = this->variance_;
    r.threshold = this->threshold_;
    r.baseline = this->baseline_;
    r.change_rate = this->change_rate_;
    r.spatial_change = this->spatial_change_;
    r.coherence = this->coherence_;
    r.feature_score = this->feature_score_;
    return r;
  }

  float turbulence_[kWindow]{};
  float hampel_buffer_[kHampelWindow]{};
  uint16_t write_index_{0};
  uint16_t count_{0};
  uint32_t packet_counter_{0};
  uint8_t hampel_index_{0};
  uint8_t hampel_count_{0};

  uint8_t enter_hits_{3};
  uint8_t exit_hits_{3};
  uint8_t enter_hits_count_{0};
  uint8_t exit_hits_count_{0};
  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};
  bool active_{false};

  float latest_turbulence_{0.0f};
  float variance_{0.0f};
  float threshold_{5.0f};
  float baseline_{0.0f};
  float change_rate_{0.0f};
  float spatial_change_{0.0f};
  float coherence_{1.0f};
  float feature_score_{0.0f};
};

constexpr uint8_t MvsMotionDetector::kSubcarriers[12];

}  // namespace esp_wifi_sensing
}  // namespace esphome
