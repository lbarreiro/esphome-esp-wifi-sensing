#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace esphome {
namespace esp_wifi_sensing {

// Receives ONLY unweighted, common-mode-centred temporal energy and MVS-3
// confirmations. No reference to either control algorithm exists here.
class CsiLearning {
 public:
  static constexpr size_t kBins = 48;
  CsiLearning() {
    for (size_t i = 0; i < kBins; ++i) weights_[i] = candidates_[i] = 1.0f;
  }
  const float *weights() const { return weights_; }
  float candidate(size_t i) const { return candidates_[i]; }
  uint32_t quiet_blocks() const { return quiet_blocks_; }
  uint32_t episodes() const { return episodes_; }
  uint32_t rejected() const { return rejected_; }
  uint32_t revisions() const { return revisions_; }
  bool in_episode() const { return episode_; }
  bool ready() const { return quiet_blocks_ >= 6; }
  float quiet_jitter() const {
    float sum = 0, norm = 0;
    for (size_t i = 0; i < kBins; ++i) {
      sum += weights_[i] * quiet_[i];
      norm += weights_[i];
    }
    return std::sqrt(sum / norm + 1.0e-4f);
  }
  void discontinuity() {
    if (episode_) ++rejected_;
    episode_ = false;
    quiet_count_ = 0;
    quiet_started_ = false;
    for (size_t i = 0; i < kBins; ++i) quiet_sum_[i] = 0;
  }

  // Returns true only when a complete weight revision has been committed.
  bool observe(const float *energy, uint32_t now, bool confirmed, bool held) {
    float total = 0, floor = 0;
    size_t responsive = 0;
    for (size_t i = 0; i < kBins; ++i) {
      if (!std::isfinite(energy[i]) || energy[i] < 0) { discontinuity(); return false; }
      total += energy[i];
      floor += quiet_[i];
      if (energy[i] > std::max(quiet_[i] * 9.0f, 0.0004f)) ++responsive;
    }
    // Independent raw gate: weighted score / held ON alone cannot label motion.
    const bool raw_motion = ready() && responsive >= 6 &&
        total > std::max(floor * 9.0f, 0.0192f);
    const bool raw_quiet = !ready() ? total < 0.0192f :
        total <= std::max(floor * 2.0f, 0.0048f);

    if (raw_motion) {
      if (!episode_) {
        episode_ = true;
        start_ms_ = now;
        episode_count_ = 0;
        confirmed_ = false;
        for (size_t i = 0; i < kBins; ++i) episode_sum_[i] = 0;
      }
      last_peak_ms_ = now;
      if (episode_count_ < 100000) {
        ++episode_count_;
        for (size_t i = 0; i < kBins; ++i) episode_sum_[i] += energy[i];
      }
    }
    if (episode_) {
      confirmed_ |= confirmed;
      if (raw_quiet && static_cast<uint32_t>(now - last_peak_ms_) >= 2000) {
        // One vote per episode, regardless of packet count or hold duration.
        if (confirmed_ && episode_count_ >= 5 &&
            static_cast<uint32_t>(last_peak_ms_ - start_ms_) >= 500 &&
            static_cast<uint32_t>(now - start_ms_) <= 30000) {
          ++episodes_;
          for (size_t i = 0; i < kBins; ++i) {
            const float response = episode_sum_[i] / episode_count_;
            const float ratio = std::min(20.0f, response / std::max(quiet_[i], 0.0001f));
            movement_[i] += (ratio - movement_[i]) / std::min<uint32_t>(episodes_, 16U);
            const float hit = ratio >= 4.0f ? 1.0f : 0.0f;
            recurrence_[i] += (hit - recurrence_[i]) / std::min<uint32_t>(episodes_, 16U);
          }
        } else {
          ++rejected_;
        }
        episode_ = false;
      }
      reset_quiet_block_();
      return false;
    }

    // Ambiguous observations and the experimental hold cannot teach silence.
    if (!raw_quiet || held) { reset_quiet_block_(); return false; }
    if (!quiet_started_) { quiet_started_ = true; quiet_start_ms_ = now; }
    ++quiet_count_;
    for (size_t i = 0; i < kBins; ++i) quiet_sum_[i] += energy[i];
    if (static_cast<uint32_t>(now - quiet_start_ms_) < 5000 || quiet_count_ < 20) return false;
    ++quiet_blocks_;
    for (size_t i = 0; i < kBins; ++i) {
      const float sample = quiet_sum_[i] / quiet_count_;
      quiet_[i] += (sample - quiet_[i]) / std::min<uint32_t>(quiet_blocks_, 24U);
    }
    reset_quiet_block_();
    if (!ready() || episodes_ < 3) return false;

    float mean_noise = 0;
    for (float q : quiet_) mean_noise += q / kBins;
    for (size_t i = 0; i < kBins; ++i) {
      const float silence = mean_noise / std::max(mean_noise + quiet_[i], 0.0001f);
      const float response = std::min(1.0f, movement_[i] / 10.0f);
      candidates_[i] = std::max(0.75f, std::min(1.50f,
          0.75f + 0.15f * silence + 0.30f * response + 0.30f * recurrence_[i]));
    }
    // New independent evidence is required for EVERY revision; elapsed time
    // alone never repeats a previous vote and never drives weights to a limit.
    if (episodes_ - applied_episodes_ < 3 || quiet_blocks_ - applied_quiet_ < 6 ||
        static_cast<uint32_t>(now - last_apply_ms_) < 300000) return false;
    for (size_t i = 0; i < kBins; ++i) {
      weights_[i] += std::max(-0.05f, std::min(0.05f, candidates_[i] - weights_[i]));
      weights_[i] = std::max(0.75f, std::min(1.50f, weights_[i]));
    }
    applied_episodes_ = episodes_;
    applied_quiet_ = quiet_blocks_;
    last_apply_ms_ = now;
    ++revisions_;
    return true;
  }

 private:
  void reset_quiet_block_() {
    quiet_started_ = false;
    quiet_count_ = 0;
    for (float &v : quiet_sum_) v = 0;
  }
  float weights_[kBins]{}, candidates_[kBins]{}, quiet_[kBins]{};
  float quiet_sum_[kBins]{}, episode_sum_[kBins]{}, movement_[kBins]{}, recurrence_[kBins]{};
  uint32_t quiet_blocks_{0}, episodes_{0}, rejected_{0}, revisions_{0};
  uint32_t applied_episodes_{0}, applied_quiet_{0}, last_apply_ms_{0};
  uint32_t quiet_count_{0}, quiet_start_ms_{0}, episode_count_{0}, start_ms_{0}, last_peak_ms_{0};
  bool quiet_started_{false}, episode_{false}, confirmed_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
