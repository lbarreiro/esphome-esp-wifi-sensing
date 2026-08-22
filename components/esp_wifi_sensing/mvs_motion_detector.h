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

// MVS-style temporal motion detector.
//
// The CSI variance remains the diagnostic signal, but the decision is made
// from a 1 Hz temporal envelope instead of raw CSI callback frequency.
// A motion event is a change in the variance pattern (3 of the last 5
// one-second samples above the robust noise threshold), not a single spike.
// The 60 s hold is presentation-only.
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
      for (uint16_t i = 1; i < kMaxWindow; ++i) this->window_[i - 1] = this->window_[i];
      this->window_[kMaxWindow - 1] = x;
    }

    const uint16_t n = std::min<uint16_t>(this->count_, this->window_samples_);
    if (n < 8) return this->result_();

    float mean = 0.0f;
    const uint16_t start = this->count_ - n;
    for (uint16_t i = start; i < this->count_; ++i) mean += this->window_[i];
    mean /= static_cast<float>(n);

    float variance = 0.0f;
    for (uint16_t i = start; i < this->count_; ++i) {
      const float d = this->window_[i] - mean;
      variance += d * d;
    }
    variance /= static_cast<float>(n);
    this->variance_ = variance;

    // Calibration is explicitly time based: one decision sample per second.
    // This is independent of how many CSI callbacks arrive per second.
    if (!this->calibrated_) {
      if (this->calibration_start_ms_ == 0) this->calibration_start_ms_ = now_ms;
      if (this->last_1hz_ms_ == 0 || static_cast<uint32_t>(now_ms - this->last_1hz_ms_) >= 1000) {
        this->last_1hz_ms_ = now_ms;
        if (this->calibration_count_ < kMaxCalibrationSamples)
          this->calibration_values_[this->calibration_count_++] = variance;
      }
      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationDurationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples) {
        this->finish_calibration_();
      }
      return this->result_();
    }

    // All state-machine decisions happen at 1 Hz. This fixes the fundamental
    // error where enter/exit persistence was previously measured in CSI
    // callbacks rather than seconds.
    if (this->last_1hz_ms_ != 0 && static_cast<uint32_t>(now_ms - this->last_1hz_ms_) < 1000)
      return this->result_();
    this->last_1hz_ms_ = now_ms;

    this->threshold_ = this->threshold_for_baseline_();
    const float exit_level = std::fmax(this->baseline_ + this->variance_stddev_ * 0.50f,
                                       this->quiet_p99_ * 0.45f);

    // Temporal envelope: classify this one-second sample against the robust
    // noise floor. A very large excursion is worth two votes; normal excess
    // energy is one vote. This catches real sustained movement while rejecting
    // the isolated 1-second spikes visible in the audit data.
    const float ratio = variance / std::fmax(1.0f, this->threshold_);
    uint8_t vote = 0;
    if (ratio >= kStrongRatio) vote = 2;
    else if (ratio >= 1.0f) vote = 1;

    if (!this->active_) {
      if (this->rearm_required_) {
        if (variance <= exit_level) {
          if (this->quiet_seconds_ < kRearmSeconds) ++this->quiet_seconds_;
          if (this->quiet_seconds_ >= kRearmSeconds) {
            this->rearm_required_ = false;
            this->quiet_seconds_ = 0;
          }
        } else {
          this->quiet_seconds_ = 0;
        }
        return this->result_();
      }

      // Sliding 5-second vote window. Require >=3 votes. A strong excursion
      // contributes two votes but still cannot trigger from one sample alone.
      if (this->vote_count_ < kVoteWindow) {
        this->votes_[this->vote_count_++] = vote;
      } else {
        for (uint8_t i = 1; i < kVoteWindow; ++i) this->votes_[i - 1] = this->votes_[i];
        this->votes_[kVoteWindow - 1] = vote;
      }

      uint8_t votes = 0;
      for (uint8_t i = 0; i < this->vote_count_; ++i) votes += this->votes_[i];

      if (this->vote_count_ >= kVoteWindow && votes >= kRequiredVotes) {
        this->active_ = true;
        this->hold_until_ms_ = now_ms + this->hold_time_ms_;
        this->vote_count_ = 0;
        for (auto &v : this->votes_) v = 0;
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      // OFF is based on actual quiet signal, not the end of the hold timer.
      if (variance <= exit_level) {
        if (this->exit_quiet_seconds_ < kExitQuietSeconds) ++this->exit_quiet_seconds_;
        if (this->exit_quiet_seconds_ >= kExitQuietSeconds) {
          this->active_ = false;
          this->rearm_required_ = true;
          this->exit_quiet_seconds_ = 0;
          this->vote_count_ = 0;
          for (auto &v : this->votes_) v = 0;
        }
      } else {
        this->exit_quiet_seconds_ = 0;
      }
    }

    return this->result_();
  }

 private:
  void finish_calibration_() {
    std::sort(this->calibration_values_, this->calibration_values_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);

    float sum = 0.0f;
    for (uint16_t i = 0; i < usable; ++i) sum += this->calibration_values_[i];
    this->baseline_ = sum / static_cast<float>(usable);

    float sq = 0.0f;
    for (uint16_t i = 0; i < usable; ++i) {
      const float d = this->calibration_values_[i] - this->baseline_;
      sq += d * d;
    }
    this->variance_stddev_ = std::sqrt(sq / static_cast<float>(std::max<uint16_t>(1, usable - 1)));

    const uint16_t p99_index = static_cast<uint16_t>((usable - 1) * 0.99f);
    this->quiet_p99_ = this->calibration_values_[p99_index];
    this->threshold_ = this->threshold_for_baseline_();
    this->calibrated_ = true;
    this->last_1hz_ms_ = 0;
  }

  float threshold_for_baseline_() const {
    // Robust floor: sigma threshold cannot be below the observed quiet P99.
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
  static constexpr uint8_t kVoteWindow = 5;
  static constexpr uint8_t kRequiredVotes = 3;
  static constexpr float kStrongRatio = 1.75f;
  static constexpr uint8_t kRearmSeconds = 10;
  static constexpr uint8_t kExitQuietSeconds = 5;

  float window_[kMaxWindow]{};
  uint16_t count_{0};
  uint16_t window_samples_{32};

  float calibration_values_[kMaxCalibrationSamples]{};
  uint16_t calibration_count_{0};
  uint32_t calibration_start_ms_{0};
  uint32_t last_1hz_ms_{0};

  float sigma_multiplier_{2.0f};
  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  uint8_t quiet_seconds_{0};
  uint8_t exit_quiet_seconds_{0};

  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  bool active_{false};
  bool calibrated_{false};
  bool rearm_required_{false};

  uint8_t votes_[kVoteWindow]{};
  uint8_t vote_count_{0};

  float quiet_p99_{0.0f};
  float variance_stddev_{0.0f};
  float baseline_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
