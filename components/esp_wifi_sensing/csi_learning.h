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
  static constexpr size_t kEpisodeWindow = 8;
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
  size_t episode_window_count() const { return history_count_; }
  uint32_t recurrent_bins() const { return recurrent_bins_; }
  uint32_t promoted_bins() const { return promoted_bins_; }
  uint32_t strong_bins() const { return strong_bins_; }
  uint32_t weak_bins() const { return weak_bins_; }
  uint32_t last_episode_over10x() const { return last_episode_over10x_; }
  uint32_t last_episode_over256x() const { return last_episode_over256x_; }
  float mean_candidate() const { return mean_candidate_; }
  uint8_t hits(size_t i) const { return hit_counts_[i]; }
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
          // One bounded, quantized contrast vote per *confirmed episode*,
          // never per packet or per second of held motion.
          last_episode_over10x_ = last_episode_over256x_ = 0;
          for (size_t i = 0; i < kBins; ++i) {
            const float response = episode_sum_[i] / episode_count_;
            const float ratio = response / std::max(quiet_[i], 0.0001f);
            if (ratio >= 10.0f) ++last_episode_over10x_;
            if (ratio >= 256.0f) ++last_episode_over256x_;
            // Retain contrast differences from 4x through 256x, rather than
            // treating 10x and 20x as equally strong. 0 = no episode hit.
            uint8_t vote = 0;
            if (ratio >= 4.0f) {
              const float strength = std::log2(std::min(ratio, 256.0f) / 4.0f) / 6.0f;
              vote = static_cast<uint8_t>(1 + 254.0f * strength);
            }
            response_history_[history_next_][i] = vote;
          }
          history_next_ = (history_next_ + 1) % kEpisodeWindow;
          if (history_count_ < kEpisodeWindow) ++history_count_;
          // Candidate weights change only on fresh, validated movement
          // evidence. A quiet-only period may update the reference for the
          // NEXT episode, but cannot reshuffle an already learnt ranking.
          if (history_count_ == kEpisodeWindow) update_candidates_();
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
    // Three distinct hits in a *complete* rolling window of eight confirmed
    // episodes are required before any positive weighting is possible.
    if (!ready() || history_count_ < kEpisodeWindow) return false;
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
  void update_candidates_() {
    recurrent_bins_ = promoted_bins_ = strong_bins_ = weak_bins_ = 0;
    mean_candidate_ = 0.0f;
    for (size_t i = 0; i < kBins; ++i) {
      uint8_t hits = 0;
      uint32_t strength_sum = 0;
      for (size_t ep = 0; ep < kEpisodeWindow; ++ep) {
        const uint8_t vote = response_history_[ep][i];
        if (vote != 0) {
          ++hits;
          strength_sum += vote;
        }
      }
      hit_counts_[i] = hits;
      float target = 1.0f;
      if (hits >= 3) {
        ++recurrent_bins_;
        ever_promoted_[i] = true;
        // Absolute evidence matters: numerous genuinely good bins can ALL
        // earn high weights. Normalization in temporal_jitter_ preserves the
        // global score if every bin receives the same weight.
        const float repeatability = static_cast<float>(hits) / kEpisodeWindow;
        const float contrast = static_cast<float>(strength_sum) / (255.0f * hits);
        target += repeatability * (0.35f + 0.65f * contrast);
      } else if (hits == 0 && !ever_promoted_[i]) {
        // Persistently nonresponsive bins can be downweighted to 0.70,
        // never discarded. Formerly useful bins instead decay toward 1.00
        // when their old positive evidence leaves the rolling window.
        target = 0.70f;
      }
      candidates_[i] = std::max(0.70f, std::min(2.00f, target));
      if (candidates_[i] > 1.0001f) ++promoted_bins_;
      if (candidates_[i] >= 1.50f) ++strong_bins_;
      if (candidates_[i] <= 0.80f) ++weak_bins_;
      mean_candidate_ += candidates_[i] / kBins;
    }
  }

  void reset_quiet_block_() {
    quiet_started_ = false;
    quiet_count_ = quiet_good_ = 0;
    for (float &v : quiet_sum_) v = 0;
  }
  float weights_[kBins]{}, candidates_[kBins]{}, quiet_[kBins]{};
  float quiet_sum_[kBins]{}, episode_sum_[kBins]{};
  // 8 episodes x 48 one-byte movement contrast votes: no flash writes.
  uint8_t response_history_[kEpisodeWindow][kBins]{}, hit_counts_[kBins]{};
  bool ever_promoted_[kBins]{};
  size_t history_next_{0}, history_count_{0};
  uint32_t recurrent_bins_{0}, promoted_bins_{0}, strong_bins_{0}, weak_bins_{0};
  uint32_t last_episode_over10x_{0}, last_episode_over256x_{0};
  float mean_candidate_{1.0f};
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
