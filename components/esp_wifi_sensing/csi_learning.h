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
  uint32_t discarded_blocks() const { return discarded_blocks_; }
  uint32_t hold_resets() const { return hold_resets_; }
  uint32_t discontinuities() const { return discontinuities_; }
  uint32_t quiet_good() const { return quiet_good_; }
  uint32_t quiet_count() const { return quiet_count_; }
  float last_total() const { return last_total_; }
  float quiet_limit() const { return last_quiet_limit_; }
  bool last_raw_quiet() const { return last_raw_quiet_; }
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
    ++discontinuities_;
    if (episode_) ++rejected_;
    episode_ = false;
    quiet_count_ = 0;
    quiet_started_ = false;
    quiet_count_ = quiet_good_ = 0;
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
    // Robust rolling upper quartile: the old minimum envelope rejected nearly
    // every real-world quiet block despite no confirmed motion or CSI gaps.
    // Only update the bootstrap distribution while not held/confirmed.
    if (!held && !confirmed && !episode_) {
      quiet_history_[quiet_history_next_] = total;
      quiet_history_next_ = (quiet_history_next_ + 1) % kQuietHistory;
      if (quiet_history_count_ < kQuietHistory) ++quiet_history_count_;
    }
    float quiet_reference = total;
    if (quiet_history_count_ >= 20) {
      float sorted[kQuietHistory];
      for (size_t i = 0; i < quiet_history_count_; ++i) sorted[i] = quiet_history_[i];
      std::sort(sorted, sorted + quiet_history_count_);
      quiet_reference = sorted[(quiet_history_count_ - 1) * 3 / 4];
    }
    last_total_ = total;
    last_quiet_limit_ = !ready() ? std::max(quiet_reference * 1.35f, 0.0001f) : std::max(floor * 2.5f, 0.0048f);
    const bool raw_quiet = total <= last_quiet_limit_;
    last_raw_quiet_ = raw_quiet;

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
      if (static_cast<uint32_t>(now - start_ms_) > 30000U) {
        ++rejected_;
        episode_ = false;
        reset_quiet_block_();
        return false;
      }
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

    // Held motion cannot teach quiet. Tolerate isolated RF bursts within a
    // 5-second block, but never include those bursts in its quiet estimate.
    if (held) { ++hold_resets_; reset_quiet_block_(); return false; }
    if (!quiet_started_) { quiet_started_ = true; quiet_start_ms_ = now; }
    ++quiet_count_;
    if (raw_quiet) {
      ++quiet_good_;
      for (size_t i = 0; i < kBins; ++i) quiet_sum_[i] += energy[i];
    }
    if (static_cast<uint32_t>(now - quiet_start_ms_) < 5000 || quiet_count_ < 20) return false;
    if (quiet_good_ < 20 || quiet_good_ * 5 < quiet_count_ * 4) {
      ++discarded_blocks_;
      reset_quiet_block_();
      return false;
    }
    ++quiet_blocks_;
    for (size_t i = 0; i < kBins; ++i) {
      const float sample = quiet_sum_[i] / quiet_good_;
      quiet_[i] += (sample - quiet_[i]) / std::min<uint32_t>(quiet_blocks_, 24U);
    }
    reset_quiet_block_();
    if (!ready() || episodes_ < 3) return false;

    // Only movement evidence ranks bins. Quiet still establishes the
    // per-bin energy baseline used when each confirmed episode is scored,
    // but further quiet blocks cannot erode earned movement candidates.
    float response[kBins];
    float mean_response = 0, mean_recurrence = 0;
    for (size_t i = 0; i < kBins; ++i) {
      response[i] = std::min(1.0f, movement_[i] / 10.0f);
      mean_response += response[i] / kBins;
      mean_recurrence += recurrence_[i] / kBins;
    }
    for (size_t i = 0; i < kBins; ++i) {
      // Stronger emphasis on repeated movement responses than before.
      // Center each signal across all 48 bins: global RF changes cannot
      // promote every bin simply because they all reacted together.
      const float relative_response = response[i] - mean_response;
      const float relative_recurrence = recurrence_[i] - mean_recurrence;
      candidates_[i] = std::max(0.70f, std::min(2.00f,
          1.00f + 1.20f * relative_response + 1.20f * relative_recurrence));
    }
    // New independent evidence is required for EVERY revision; elapsed time
    // alone never repeats a previous vote and never drives weights to a limit.
    if (episodes_ - applied_episodes_ < 3 || quiet_blocks_ - applied_quiet_ < 6 ||
        static_cast<uint32_t>(now - last_apply_ms_) < 300000) return false;
    for (size_t i = 0; i < kBins; ++i) {
      weights_[i] += std::max(-0.10f, std::min(0.10f, candidates_[i] - weights_[i]));
      weights_[i] = std::max(0.70f, std::min(2.00f, weights_[i]));
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
    quiet_count_ = quiet_good_ = 0;
    for (float &v : quiet_sum_) v = 0;
  }
  float weights_[kBins]{}, candidates_[kBins]{}, quiet_[kBins]{};
  float quiet_sum_[kBins]{}, episode_sum_[kBins]{}, movement_[kBins]{}, recurrence_[kBins]{};
  uint32_t quiet_blocks_{0}, episodes_{0}, rejected_{0}, revisions_{0}, discarded_blocks_{0};
  uint32_t hold_resets_{0}, discontinuities_{0};
  float last_total_{0}, last_quiet_limit_{0};
  bool last_raw_quiet_{false};
  static constexpr size_t kQuietHistory = 64;
  float quiet_history_[kQuietHistory]{};
  size_t quiet_history_next_{0}, quiet_history_count_{0};
  uint32_t applied_episodes_{0}, applied_quiet_{0}, last_apply_ms_{0};
  uint32_t quiet_count_{0}, quiet_good_{0}, quiet_start_ms_{0}, episode_count_{0}, start_ms_{0}, last_peak_ms_{0};
  bool quiet_started_{false}, episode_{false}, confirmed_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
