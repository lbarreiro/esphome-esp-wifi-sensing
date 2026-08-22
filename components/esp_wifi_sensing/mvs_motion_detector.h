#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

struct MvsMotionResult {
  bool active{false};
  float variance{0.0f};
  float threshold{0.0f};
  float baseline{0.0f};
};

// Windowed variance detector with a time-based quiet calibration and
// time-based state decisions. CSI callbacks can run much faster than 1 Hz;
// motion persistence must therefore be measured in seconds, not callback
// counts.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t value) { this->window_samples_ = value; }
  void set_threshold_multiplier(float value) { this->sigma_multiplier_ = value; }
  void set_enter_hits(uint8_t value) { this->enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { this->exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { this->hold_time_ms_ = value; }

  MvsMotionResult update(uint32_t metric, uint32_t now_ms) {
    const float x = static_cast<float>(metric);
    if (this->count_ < kMaxWindow) {
      this->window_[this->count_++] = x;
    } else {
      for (uint16_t i = 1; i < kMaxWindow; i++) this->window_[i - 1] = this->window_[i];
      this->window_[kMaxWindow - 1] = x;
    }

    const uint16_t n = this->count_ < this->window_samples_ ? this->count_ : this->window_samples_;
    if (n < 8) return this->result_();

    float mean = 0.0f;
    const uint16_t start = this->count_ - n;
    for (uint16_t i = start; i < this->count_; i++) mean += this->window_[i];
    mean /= static_cast<float>(n);

    float variance = 0.0f;
    for (uint16_t i = start; i < this->count_; i++) {
      const float d = this->window_[i] - mean;
      variance += d * d;
    }
    variance /= static_cast<float>(n);
    this->variance_ = variance;

    if (!this->calibrated_) {
      if (this->calibration_start_ms_ == 0) this->calibration_start_ms_ = now_ms;

      if (this->last_calibration_sample_ms_ == 0 ||
          static_cast<uint32_t>(now_ms - this->last_calibration_sample_ms_) >= kCalibrationSampleMs) {
        this->last_calibration_sample_ms_ = now_ms;
        if (this->calibration_count_ < kMaxCalibrationSamples)
          this->calibration_values_[this->calibration_count_++] = variance;
      }

      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationDurationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples) {
        this->finish_calibration_();
      }
      return this->result_();
    }

    const float enter_threshold = this->threshold_for_baseline_();
    const float exit_threshold = this->baseline_ + this->variance_stddev_ *
        std::fmax(0.5f, this->sigma_multiplier_ * 0.60f);
    this->threshold_ = enter_threshold;

    // IMPORTANT: the YAML enter_hits/exit_hits are now seconds. We evaluate
    // the state machine at most once per second, regardless of the CSI
    // callback rate. Previously '2 hits' meant two fast callbacks and could
    // create a false ON almost immediately after every re-arm.
    if (this->last_decision_ms_ != 0 &&
        static_cast<uint32_t>(now_ms - this->last_decision_ms_) < kDecisionIntervalMs) {
      return this->result_();
    }
    this->last_decision_ms_ = now_ms;

    if (!this->active_) {
      if (this->rearm_required_) {
        if (variance <= exit_threshold) {
          this->quiet_rearm_count_++;
          if (this->quiet_rearm_count_ >= kQuietRearmSeconds) {
            this->rearm_required_ = false;
            this->quiet_rearm_count_ = 0;
            this->enter_count_ = 0;
          }
        } else {
          this->quiet_rearm_count_ = 0;
        }
        return this->result_();
      }

      if (variance > enter_threshold) {
        this->enter_count_++;
        if (this->enter_count_ >= this->enter_hits_) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
          this->enter_count_ = 0;
        }
      } else {
        this->enter_count_ = 0;
        if (variance <= exit_threshold)
          this->baseline_ += this->baseline_alpha_ * (variance - this->baseline_);
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      if (variance <= exit_threshold) {
        this->exit_count_++;
        if (this->exit_count_ >= this->exit_hits_) {
          this->active_ = false;
          this->exit_count_ = 0;
          this->enter_count_ = 0;
          this->rearm_required_ = true;
          this->quiet_rearm_count_ = 0;
          this->count_ = 0;
          this->variance_ = 0.0f;
        }
      } else {
        this->exit_count_ = 0;
      }
    }

    return this->result_();
  }

 private:
  void finish_calibration_() {
    std::sort(this->calibration_values_,
              this->calibration_values_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);

    float sum = 0.0f;
    for (uint16_t i = 0; i < usable; i++) sum += this->calibration_values_[i];
    this->baseline_ = sum / static_cast<float>(usable);

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < usable; i++) {
      const float d = this->calibration_values_[i] - this->baseline_;
      sum_sq += d * d;
    }
    this->variance_stddev_ = std::sqrt(sum_sq / static_cast<float>(usable > 1 ? usable - 1 : 1));

    const uint16_t p99_index = static_cast<uint16_t>((usable - 1) * 0.99f);
    this->quiet_p99_ = this->calibration_values_[p99_index];

    this->calibrated_ = true;
    this->threshold_ = this->threshold_for_baseline_();
  }

  float threshold_for_baseline_() const {
    return std::fmax(this->baseline_ + this->variance_stddev_ * this->sigma_multiplier_,
                     this->quiet_p99_);
  }

  MvsMotionResult result_() const {
    MvsMotionResult r;
    r.active = this->active_;
    r.variance = this->variance_;
    r.threshold = this->threshold_;
    r.baseline = this->baseline_;
    return r;
  }

  static constexpr uint16_t kMaxWindow = 64;
  static constexpr uint16_t kMaxCalibrationSamples = 360;
  static constexpr uint16_t kMinimumCalibrationSamples = 240;
  static constexpr uint32_t kCalibrationDurationMs = 300000;
  static constexpr uint32_t kCalibrationSampleMs = 1000;
  static constexpr uint32_t kDecisionIntervalMs = 1000;
  static constexpr uint8_t kQuietRearmSeconds = 10;

  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};

  float calibration_values_[kMaxCalibrationSamples]{};
  uint16_t calibration_count_{0};
  uint32_t calibration_start_ms_{0};
  uint32_t last_calibration_sample_ms_{0};
  uint32_t last_decision_ms_{0};

  float sigma_multiplier_{2.0f};
  float baseline_alpha_{0.005f};
  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  uint8_t enter_count_{0};
  uint8_t exit_count_{0};

  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  bool active_{false};
  bool calibrated_{false};
  bool rearm_required_{false};
  uint8_t quiet_rearm_count_{0};

  float quiet_p99_{0.0f};
  float variance_stddev_{0.0f};
  float baseline_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
