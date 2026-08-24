#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

struct MvsMotionResult {
  bool active{false};
  float variance{0.0f};
  float threshold{0.0f};
  float baseline{0.0f};
  float change_rate{0.0f};
  float spatial_change{0.0f};
  float coherence{1.0f};
  float feature_score{0.0f};
};

// MVS (Motion Variance Signature) v3.
// Detects events rather than RF states. The CSI vector is normalized per
// packet so common gain changes are suppressed; motion entry requires a
// temporal onset followed by confirmation of an event, rather than a high
// feature level by itself. This is an original detector, not a copy.
class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t value) { this->window_samples_ = value; }
  void set_threshold_multiplier(float value) { this->sigma_multiplier_ = value; }
  void set_enter_hits(uint8_t value) { this->enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { this->exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { this->hold_time_ms_ = value; }

  MvsMotionResult update(const CsiPacket &packet, uint32_t now_ms) {
    this->update_variance_(packet);
    this->extract_features_(packet);

    if (this->last_1hz_ms_ != 0 && static_cast<uint32_t>(now_ms - this->last_1hz_ms_) < 1000)
      return this->result_();
    this->last_1hz_ms_ = now_ms;

    if (!this->feature_ready_) return this->result_();

    this->change_rate_ = this->spatial_change_ - this->previous_spatial_change_;
    this->previous_spatial_change_ = this->spatial_change_;

    if (!this->calibrated_) {
      if (this->calibration_start_ms_ == 0) this->calibration_start_ms_ = now_ms;
      if (this->calibration_count_ < kCalibrationSamples)
        this->calibration_[this->calibration_count_++] = this->feature_activity_;
      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples)
        this->finish_calibration_();
      return this->result_();
    }

    const float normalized = (this->feature_activity_ - this->baseline_) /
                             std::fmax(0.001f, this->feature_scale_);
    const float positive_normalized = std::fmax(0.0f, normalized);
    this->feature_score_ = 0.70f * this->feature_score_ + 0.30f * positive_normalized;

    const bool level = this->feature_score_ >= kModerateScore;
    const bool strong_level = this->feature_score_ >= kStrongScore;
    const bool spatial = this->spatial_change_ >= kMinimumSpatialChange;
    const bool strong_spatial = this->spatial_change_ >= kStrongSpatialChange;
    const bool rising = this->change_rate_ >= kRiseRate;
    const bool falling = this->change_rate_ <= -kFallRate;

    if (!this->active_) {
      if (this->rearm_required_) {
        if (this->feature_score_ < kQuietScore && this->spatial_change_ < kQuietSpatialChange) {
          if (++this->quiet_seconds_ >= kRearmSeconds) {
            this->rearm_required_ = false;
            this->quiet_seconds_ = 0;
          }
        } else {
          this->quiet_seconds_ = 0;
        }
        return this->result_();
      }

      // Stage 1: find the onset of an event. A high level alone is never
      // enough. We need a meaningful positive transition plus independent
      // spatial/feature evidence.
      if (!this->event_armed_) {
        const int onset_evidence = (rising ? 1 : 0) + (level ? 1 : 0) +
                                   (spatial ? 1 : 0) + (strong_level || strong_spatial ? 1 : 0);
        if (rising && onset_evidence >= kOnsetEvidence) {
          this->event_armed_ = true;
          this->event_age_ = 0;
          this->event_peak_ = this->feature_score_;
          this->event_peak_spatial_ = this->spatial_change_;
          this->event_rise_seen_ = true;
          this->event_fall_seen_ = false;
        }
      } else {
        ++this->event_age_;
        this->event_peak_ = std::fmax(this->event_peak_, this->feature_score_);
        this->event_peak_spatial_ = std::fmax(this->event_peak_spatial_, this->spatial_change_);
        if (falling) this->event_fall_seen_ = true;

        // Stage 2: confirm that the onset developed into a real transient.
        // Either a clear peak followed by a fall, or a strong spatial/feature
        // excursion followed by a change in the signal, can confirm it.
        const bool peak = this->event_peak_ >= kPeakScore ||
                          this->event_peak_spatial_ >= kPeakSpatial;
        const bool returned = this->event_fall_seen_ &&
                              (this->feature_score_ <= this->event_peak_ * kReturnFraction ||
                               this->spatial_change_ <= this->event_peak_spatial_ * kReturnFraction);
        const bool strong_transient = peak && this->event_age_ >= 1 &&
                                      (this->event_fall_seen_ || this->change_rate_ <= 0.0f);

        if (peak && returned && strong_transient) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
          this->reset_event_();
        } else if (this->event_age_ >= kEventWindowSeconds) {
          this->reset_event_();
        }
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      // Exit only after the signal has genuinely returned to the quiet region.
      if (this->feature_score_ < kExitScore && this->spatial_change_ < kMinimumSpatialChange)
        ++this->exit_seconds_;
      else
        this->exit_seconds_ = 0;

      if (this->exit_seconds_ >= kExitSeconds) {
        this->active_ = false;
        this->rearm_required_ = true;
        this->exit_seconds_ = 0;
      }
    }

    // Slow baseline tracking only while the detector is genuinely quiet.
    if (!this->active_ && !this->rearm_required_ && !this->event_armed_ &&
        this->feature_score_ < kQuietScore && this->spatial_change_ < kQuietSpatialChange)
      this->baseline_ = 0.998f * this->baseline_ + 0.002f * this->feature_activity_;

    return this->result_();
  }

 private:
  void reset_event_() {
    this->event_armed_ = false;
    this->event_age_ = 0;
    this->event_peak_ = 0.0f;
    this->event_peak_spatial_ = 0.0f;
    this->event_rise_seen_ = false;
    this->event_fall_seen_ = false;
  }

  void update_variance_(const CsiPacket &packet) {
    if (packet.raw_bytes == nullptr || packet.len < 4) return;
    const uint16_t pairs = std::min<uint16_t>(packet.len / 2, kFeatureBins);
    if (pairs == 0) return;
    float mean = 0.0f;
    for (uint16_t i = 0; i < pairs; ++i) {
      const float a = static_cast<float>(packet.raw_bytes[2 * i]);
      const float b = static_cast<float>(packet.raw_bytes[2 * i + 1]);
      mean += std::sqrt(a * a + b * b);
    }
    mean /= pairs;
    float v = 0.0f;
    for (uint16_t i = 0; i < pairs; ++i) {
      const float a = static_cast<float>(packet.raw_bytes[2 * i]);
      const float b = static_cast<float>(packet.raw_bytes[2 * i + 1]);
      const float m = std::sqrt(a * a + b * b);
      const float d = m - mean;
      v += d * d;
    }
    this->variance_ = v / pairs;
  }

  void extract_features_(const CsiPacket &packet) {
    this->feature_ready_ = false;
    if (packet.raw_bytes == nullptr || packet.len < 8) return;
    const uint16_t pairs = std::min<uint16_t>(packet.len / 2, kFeatureBins);
    float magnitudes[kFeatureBins]{};
    float mean = 0.0f;
    for (uint16_t i = 0; i < pairs; ++i) {
      const float a = static_cast<float>(packet.raw_bytes[2 * i]);
      const float b = static_cast<float>(packet.raw_bytes[2 * i + 1]);
      magnitudes[i] = std::sqrt(a * a + b * b);
      mean += magnitudes[i];
    }
    mean /= pairs;
    if (mean < 1.0f) return;

    float norm[kFeatureBins]{};
    float norm_std = 0.0f;
    for (uint16_t i = 0; i < pairs; ++i) norm[i] = magnitudes[i] / mean;
    for (uint16_t i = 0; i < pairs; ++i) {
      const float d = norm[i] - 1.0f;
      norm_std += d * d;
    }
    norm_std = std::sqrt(norm_std / pairs);

    float temporal = 0.0f;
    float dot = 0.0f, prev_norm_sq = 0.0f, norm_sq = 0.0f;
    if (this->have_previous_vector_ && this->previous_bins_ == pairs) {
      for (uint16_t i = 0; i < pairs; ++i) {
        const float d = norm[i] - this->previous_norm_[i];
        temporal += std::fabs(d);
        dot += norm[i] * this->previous_norm_[i];
        norm_sq += norm[i] * norm[i];
        prev_norm_sq += this->previous_norm_[i] * this->previous_norm_[i];
      }
      temporal /= pairs;
    }

    const float cosine = (norm_sq > 0.001f && prev_norm_sq > 0.001f)
                           ? dot / std::sqrt(norm_sq * prev_norm_sq) : 1.0f;
    this->coherence_ = std::fmax(0.0f, std::fmin(1.0f, cosine));
    this->spatial_change_ = temporal;
    const float incoherence = 1.0f - this->coherence_;
    this->feature_activity_ = 0.70f * temporal + 0.30f * incoherence + 0.15f * norm_std;

    for (uint16_t i = 0; i < pairs; ++i) this->previous_norm_[i] = norm[i];
    this->previous_bins_ = pairs;
    this->have_previous_vector_ = true;
    this->feature_ready_ = true;
  }

  void finish_calibration_() {
    std::sort(this->calibration_, this->calibration_ + this->calibration_count_);
    const uint16_t usable = std::max<uint16_t>(1, (this->calibration_count_ * 9) / 10);
    const uint16_t median_index = usable / 2;
    this->baseline_ = std::fmax(0.001f, this->calibration_[median_index]);

    float deviations[kCalibrationSamples];
    for (uint16_t i = 0; i < usable; ++i)
      deviations[i] = std::fabs(this->calibration_[i] - this->baseline_);
    std::sort(deviations, deviations + usable);
    const float mad = deviations[median_index];
    this->feature_scale_ = std::fmax(0.001f, 1.4826f * mad);
    this->calibrated_ = true;
    this->threshold_ = this->baseline_ + this->feature_scale_ * this->sigma_multiplier_;
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

  static constexpr uint16_t kFeatureBins = 64;
  static constexpr uint16_t kCalibrationSamples = 300;
  static constexpr uint16_t kMinimumCalibrationSamples = 240;
  static constexpr uint32_t kCalibrationMs = 300000;
  static constexpr uint16_t kRearmSeconds = 10;
  static constexpr uint16_t kExitSeconds = 5;
  static constexpr uint16_t kEventWindowSeconds = 6;
  static constexpr int kOnsetEvidence = 3;
  static constexpr float kModerateScore = 1.20f;
  static constexpr float kStrongScore = 2.50f;
  static constexpr float kPeakScore = 1.80f;
  static constexpr float kQuietScore = 0.45f;
  static constexpr float kExitScore = 0.55f;
  static constexpr float kMinimumSpatialChange = 0.020f;
  static constexpr float kStrongSpatialChange = 0.055f;
  static constexpr float kQuietSpatialChange = 0.010f;
  static constexpr float kPeakSpatial = 0.045f;
  static constexpr float kRiseRate = 0.010f;
  static constexpr float kFallRate = 0.008f;
  static constexpr float kReturnFraction = 0.65f;

  uint16_t window_samples_{32};
  float sigma_multiplier_{2.0f};
  uint8_t enter_hits_{2};
  uint8_t exit_hits_{2};
  uint32_t hold_time_ms_{60000};
  uint32_t hold_until_ms_{0};

  float calibration_[kCalibrationSamples]{};
  uint16_t calibration_count_{0};
  uint32_t calibration_start_ms_{0};
  uint32_t last_1hz_ms_{0};

  float previous_norm_[kFeatureBins]{};
  uint16_t previous_bins_{0};
  bool have_previous_vector_{false};
  bool feature_ready_{false};
  bool calibrated_{false};
  bool active_{false};
  bool rearm_required_{false};
  bool event_armed_{false};
  bool event_rise_seen_{false};
  bool event_fall_seen_{false};

  float baseline_{0.0f};
  float feature_scale_{0.0f};
  float variance_{0.0f};
  float threshold_{0.0f};
  float feature_activity_{0.0f};
  float feature_score_{0.0f};
  float spatial_change_{0.0f};
  float previous_spatial_change_{0.0f};
  float change_rate_{0.0f};
  float coherence_{1.0f};

  float event_peak_{0.0f};
  float event_peak_spatial_{0.0f};
  uint16_t event_age_{0};
  uint16_t exit_seconds_{0};
  uint16_t quiet_seconds_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
