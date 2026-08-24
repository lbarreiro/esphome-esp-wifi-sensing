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

// MVS (Motion Variance Signature) v2.
// Uses variance for diagnostics, but motion is decided from a normalized
// spatial-temporal CSI signature: per-subcarrier magnitude change, temporal
// change and spatial contrast. Common-mode amplitude changes are normalized
// out so RF/gain changes are less likely to look like motion.
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

    const float raw_change = this->spatial_change_;
    this->change_rate_ = raw_change - this->previous_spatial_change_;
    this->previous_spatial_change_ = raw_change;

    if (!this->calibrated_) {
      if (this->calibration_start_ms_ == 0) this->calibration_start_ms_ = now_ms;
      if (this->calibration_count_ < kCalibrationSamples) {
        this->calibration_[this->calibration_count_++] = this->feature_activity_;
      }
      if (static_cast<uint32_t>(now_ms - this->calibration_start_ms_) >= kCalibrationMs &&
          this->calibration_count_ >= kMinimumCalibrationSamples) {
        this->finish_calibration_();
      }
      this->previous_feature_activity_ = this->feature_activity_;
      this->have_previous_feature_ = true;
      return this->result_();
    }

    // Robust normalized feature score. The score is relative to the learned
    // quiet distribution, not to raw CSI amplitude.
    const float normalized = (this->feature_activity_ - this->baseline_) /
                             std::fmax(0.001f, this->feature_scale_);
    this->feature_score_ = 0.65f * this->feature_score_ + 0.35f * std::fmax(0.0f, normalized);

    // Common-mode RF changes tend to affect all subcarriers similarly and
    // therefore have low normalized spatial change. Motion must have both
    // enough spatial-temporal change and a coherent transition.
    const bool elevated = this->feature_score_ >= kModerateScore;
    const bool strong = this->feature_score_ >= kStrongScore;
    const bool rising = this->change_rate_ >= kRiseRate;
    const bool falling = this->change_rate_ <= -kFallRate;

    if (!this->active_) {
      if (this->rearm_required_) {
        if (this->feature_score_ < kQuietScore && raw_change < kQuietSpatialChange) {
          if (++this->quiet_seconds_ >= kRearmSeconds) {
            this->rearm_required_ = false;
            this->quiet_seconds_ = 0;
          }
        } else {
          this->quiet_seconds_ = 0;
        }
        return this->result_();
      }

      // A genuine event should create a spatially structured transition.
      if (rising && elevated)
        this->signature_score_ += 2;
      else if (strong && raw_change >= kMinimumSpatialChange)
        this->signature_score_ += 1;
      else if (falling && this->signature_score_ > 0)
        this->signature_score_ += 1;
      else
        this->signature_score_ = std::max(0, this->signature_score_ - 1);

      if (elevated && raw_change >= kMinimumSpatialChange)
        ++this->elevated_seconds_;
      else
        this->elevated_seconds_ = 0;

      if (++this->enter_window_seconds_ >= kEvidenceWindowSeconds) {
        const bool sustained = this->elevated_seconds_ >= kMinimumElevatedSeconds;
        if (this->signature_score_ >= kRequiredSignatureScore ||
            (sustained && this->signature_score_ >= kSustainedSignatureScore)) {
          this->active_ = true;
          this->hold_until_ms_ = now_ms + this->hold_time_ms_;
        }
        this->signature_score_ = 0;
        this->enter_window_seconds_ = 0;
        this->elevated_seconds_ = 0;
      }
    } else if (static_cast<int32_t>(now_ms - this->hold_until_ms_) >= 0) {
      if (this->feature_score_ < kExitScore && raw_change < kMinimumSpatialChange)
        ++this->exit_seconds_;
      else
        this->exit_seconds_ = 0;

      if (this->exit_seconds_ >= kExitSeconds) {
        this->active_ = false;
        this->rearm_required_ = true;
        this->exit_seconds_ = 0;
        this->signature_score_ = 0;
        this->enter_window_seconds_ = 0;
        this->elevated_seconds_ = 0;
      }
    }

    // Adapt only when clearly quiet. This prevents a new RF state during an
    // active event from becoming the new motion-free reference.
    if (!this->active_ && !this->rearm_required_ && this->feature_score_ < kQuietScore &&
        raw_change < kQuietSpatialChange) {
      this->baseline_ = 0.997f * this->baseline_ + 0.003f * this->feature_activity_;
    }

    this->previous_feature_activity_ = this->feature_activity_;
    this->have_previous_feature_ = true;
    return this->result_();
  }

 private:
  void update_variance_(const CsiPacket &packet) {
    if (packet.raw_bytes == nullptr || packet.len < 4) return;
    // Preserve the diagnostic variance concept from the previous MVS.
    float mean = 0.0f;
    const uint16_t pairs = std::min<uint16_t>(packet.len / 2, kFeatureBins);
    if (pairs == 0) return;
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

    // Normalize each packet by its own mean. This suppresses common-mode RSSI
    // and gain changes while retaining the relative CSI shape.
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

    // Feature activity deliberately combines spatial shape change and loss of
    // temporal coherence. A uniform RF/gain shift largely disappears after
    // normalization; a localized CSI pattern change survives.
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
  static constexpr uint16_t kEvidenceWindowSeconds = 5;
  static constexpr uint16_t kMinimumElevatedSeconds = 2;
  static constexpr int kRequiredSignatureScore = 3;
  static constexpr int kSustainedSignatureScore = 4;
  static constexpr float kModerateScore = 2.0f;
  static constexpr float kStrongScore = 3.5f;
  static constexpr float kQuietScore = 0.8f;
  static constexpr float kExitScore = 1.0f;
  static constexpr float kMinimumSpatialChange = 0.035f;
  static constexpr float kQuietSpatialChange = 0.015f;
  static constexpr float kRiseRate = 0.02f;
  static constexpr float kFallRate = 0.02f;

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
  bool have_previous_feature_{false};

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
  float previous_feature_activity_{0.0f};

  int signature_score_{0};
  uint16_t enter_window_seconds_{0};
  uint16_t elevated_seconds_{0};
  uint16_t exit_seconds_{0};
  uint16_t quiet_seconds_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
