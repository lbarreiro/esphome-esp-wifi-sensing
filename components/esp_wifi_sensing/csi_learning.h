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
  static constexpr size_t kMinHits = 5;
  static constexpr size_t kPreQuietWindow = 8;
  CsiLearning() {
    for (size_t i = 0; i < kBins; ++i) weights_[i] = candidates_[i] = 1.0f;
  }
  const float *weights() const { return weights_; }
  float candidate(size_t i) const { return candidates_[i]; }
  float quiet_energy(size_t i) const { return quiet_[i]; }
  // Select by independently validated 5/8 evidence, never by relative rank
  // alone. No priority path exists until two bins actually qualify.
  bool select_top_two(size_t (&selected)[2]) const {
    if (history_count_ < kEpisodeWindow) return false;
    size_t found = 0;
    for (size_t i = 0; i < kBins; ++i) {
      if (hit_counts_[i] < kMinHits || candidates_[i] <= 1.0001f) continue;
      size_t pos = 0;
      while (pos < found && (candidates_[selected[pos]] > candidates_[i] ||
             (candidates_[selected[pos]] == candidates_[i] && selected[pos] < i))) ++pos;
      if (pos >= 2) continue;
      if (found < 2) ++found;
      for (size_t j = found - 1; j > pos; --j) selected[j] = selected[j - 1];
      selected[pos] = i;
    }
    return found == 2;
  }
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
  uint32_t eligible_episodes() const { return eligible_episodes_; }
  uint32_t skipped_episodes() const { return skipped_episodes_; }
  uint32_t last_prequiet_bins() const { return last_prequiet_bins_; }
  uint32_t last_transition_bins() const { return last_transition_bins_; }
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
    episode_pre_valid_ = false;
    pre_count_ = pre_next_ = 0;
    pre_last_ms_ = 0;
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
        episode_pre_valid_ = pre_count_ == kPreQuietWindow &&
            static_cast<uint32_t>(now - pre_last_ms_) <= 2000U;
        last_prequiet_bins_ = 0;
        for (size_t i = 0; i < kBins; ++i) {
          episode_sum_[i] = 0;
          episode_above_[i] = 0;
          float pre_sum = 0, pre_peak = 0;
          if (episode_pre_valid_) {
            for (size_t j = 0; j < kPreQuietWindow; ++j) {
              const float v = pre_energy_[j][i];
              pre_sum += v;
              pre_peak = std::max(pre_peak, v);
            }
          }
          episode_pre_mean_[i] = pre_sum / kPreQuietWindow;
          // A permanently noisy bin was NOT quiet before motion; it cannot
          // earn a hit just by staying over the movement threshold.
          const float idle_limit = std::max(quiet_[i] * 3.0f, 0.0002f);
          episode_prequiet_[i] = episode_pre_valid_ &&
              pre_peak <= idle_limit && episode_pre_mean_[i] <= idle_limit;
          if (episode_prequiet_[i]) ++last_prequiet_bins_;
          episode_threshold_[i] = std::max(
              std::max(quiet_[i] * 4.0f, episode_pre_mean_[i] * 4.0f), 0.0004f);
        }
        // A new transition requires a fresh preceding quiet interval.
        pre_count_ = pre_next_ = 0;
      }
      last_peak_ms_ = now;
      if (episode_count_ < 100000) {
        ++episode_count_;
        for (size_t i = 0; i < kBins; ++i) {
          episode_sum_[i] += energy[i];
          if (episode_prequiet_[i] && energy[i] > episode_threshold_[i])
            ++episode_above_[i];
        }
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
          // Only independent confirmed episodes with a fresh pre-motion
          // baseline are usable to teach the last-eight movement window.
          const bool separated = eligible_episodes_ == 0 ||
              static_cast<uint32_t>(now - last_eligible_ms_) >= 5000U;
          if (episode_pre_valid_ && separated) {
            ++eligible_episodes_;
            last_eligible_ms_ = now;
            last_transition_bins_ = last_episode_over10x_ = last_episode_over256x_ = 0;
            for (size_t i = 0; i < kBins; ++i) {
              const float response = episode_sum_[i] / episode_count_;
              const float baseline = std::max(
                  std::max(quiet_[i], episode_pre_mean_[i]), 0.0001f);
              const float ratio = response / baseline;
              // At least three above-threshold observations AND >=25%
              // of the episode. A solitary RF burst never earns a hit.
              const bool transitioned = episode_prequiet_[i] &&
                  episode_above_[i] >= 3U &&
                  static_cast<uint32_t>(episode_above_[i]) * 4U >= episode_count_ &&
                  ratio >= 4.0f;
              uint8_t vote = 0;
              if (transitioned) {
                ++last_transition_bins_;
                if (ratio >= 10.0f) ++last_episode_over10x_;
                if (ratio >= 256.0f) ++last_episode_over256x_;
                // Contrast 4x -> 256x is retained in the score; a small
                // denominator alone cannot create an arbitrary 256x reward.
                const float strength =
                    std::log2(std::min(ratio, 256.0f) / 4.0f) / 6.0f;
                vote = static_cast<uint8_t>(1 + 254.0f * strength);
              }
              response_history_[history_next_][i] = vote;
            }
            history_next_ = (history_next_ + 1) % kEpisodeWindow;
            if (history_count_ < kEpisodeWindow) ++history_count_;
            // Quiet-only data cannot modify candidates already earned.
            if (history_count_ == kEpisodeWindow) update_candidates_();
          } else {
            ++skipped_episodes_;
          }
        } else {
          ++rejected_;
        }
        episode_ = false;
      }
      reset_quiet_block_();
      return false;
    }

    // The pre-movement evidence is independent of held ON: a genuine
    // stable quiet interval can precede a new motion while hold is active.
    // Never collect the triggering frame or a confirmed motion frame.
    if (!raw_motion && raw_quiet && !confirmed) {
      for (size_t i = 0; i < kBins; ++i)
        pre_energy_[pre_next_][i] = energy[i];
      pre_next_ = (pre_next_ + 1) % kPreQuietWindow;
      if (pre_count_ < kPreQuietWindow) ++pre_count_;
      pre_last_ms_ = now;
    }
    // Held motion cannot teach the long-term quiet baseline.
    // Tolerate isolated RF bursts within a five-second block.
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
    // Five genuine quiet-to-movement hits in eight qualified episodes are
    // required before a bin can earn a positive weight.
    if (!ready() || history_count_ < kEpisodeWindow) return false;
    // New independent evidence is required for EVERY revision; elapsed time
    // alone never repeats a previous vote and never drives weights to a limit.
    if (episodes_ - applied_episodes_ < 3 || quiet_blocks_ - applied_quiet_ < 6 ||
        static_cast<uint32_t>(now - last_apply_ms_) < 300000) return false;
    for (size_t i = 0; i < kBins; ++i) {
      // Apply 90% of the newly learned correction per authorized revision.
      // The episode/quiet/time gates above still prevent a silent-only drift.
      weights_[i] += 0.90f * (candidates_[i] - weights_[i]);
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
      if (hits >= kMinHits) {
        ++recurrent_bins_;
        ever_promoted_[i] = true;
        // Absolute evidence matters: numerous genuinely good bins can ALL
        // earn high weights. Normalization in temporal_jitter_ preserves the
        // global score if every bin receives the same weight.
        const float repeatability = static_cast<float>(hits) / kEpisodeWindow;
        const float contrast = static_cast<float>(strength_sum) / (255.0f * hits);
        // Strength matters much more than mere recurrence. A marginal 4x
        // reaction is weak; eight clean, repeated 256x responses can reach 2.
        target += repeatability * (0.15f + 0.85f * contrast);
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
  // A rolling 0.8-second pre-motion baseline, independent of ON hold.
  // The snapshot is frozen at the episode's first raw-motion observation.
  float pre_energy_[kPreQuietWindow][kBins]{};
  float episode_pre_mean_[kBins]{}, episode_threshold_[kBins]{};
  uint16_t episode_above_[kBins]{};
  bool episode_prequiet_[kBins]{};
  size_t pre_next_{0}, pre_count_{0};
  uint32_t pre_last_ms_{0};
  bool episode_pre_valid_{false};
  // 8 episodes x 48 one-byte movement contrast votes: no flash writes.
  uint8_t response_history_[kEpisodeWindow][kBins]{}, hit_counts_[kBins]{};
  bool ever_promoted_[kBins]{};
  size_t history_next_{0}, history_count_{0};
  uint32_t recurrent_bins_{0}, promoted_bins_{0}, strong_bins_{0}, weak_bins_{0};
  uint32_t last_episode_over10x_{0}, last_episode_over256x_{0};
  uint32_t last_prequiet_bins_{0}, last_transition_bins_{0};
  uint32_t eligible_episodes_{0}, skipped_episodes_{0}, last_eligible_ms_{0};
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
