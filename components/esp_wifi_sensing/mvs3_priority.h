#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "csi_learning.h"
#include "csi_noise_learning.h"

namespace esphome {
namespace esp_wifi_sensing {

// Only the experimental MVS-3 uses this independent decision path.
// With two or three qualified bins, 70% is shared by the selected group.
// Below two qualified bins, the ordinary weighted 48-bin score is retained.
class Mvs3Priority {
 public:
  static constexpr size_t kTop = 2;  // Independent confirmation retains its two-bin gate.
  static constexpr size_t kMaxTop = 3;  // Global 70/30 can consider three bins.
  // Existing TOP members may be retained at 4/8 when fewer than two new bins qualify at 5/8.
  static constexpr size_t kTopRetentionHits = 4;
  static constexpr size_t kBins = CsiLearning::kBins;
  static constexpr size_t kMaxWindow = 64;
  static constexpr float kTopShare = 0.70f;
  static constexpr float kOtherShare = 0.30f;
  static constexpr float kEpsilon = 1.0e-4f;
  static constexpr float kQuietFloor = 0.003f;

  bool active() const { return active_; }
  size_t top_bin(size_t index) const { return index < active_count_ ? top_[index] : 0; }
  size_t active_count() const { return active_count_; }
  void set_noise_learning(const CsiNoiseLearning *noise) { noise_ = noise; }
  void set_multiplicative_ranking(bool enabled) { multiplicative_ranking_ = enabled; }
  float score() const { return score_; }
  uint32_t confirmations() const { return confirmations_; }

  void set_filter_window(size_t count) { window_ = count; clear_filter(); }
  void set_filter_count(uint8_t count) { count_ = count; clear_filter(); }
  void clear_filter() {
    for (size_t j = 0; j < kMaxWindow; ++j) filter_[j] = false;
    next_ = 0;
    hits_ = 0;
  }
  void reset_after_gap() {
    clear_filter();
    score_ = 0;
  }

  // The episode window, not the weight revision, controls priority membership.
  // Run only after NEW eligible evidence (or after a weight revision).
  // A disqualified bin is replaced immediately if two alternatives qualify;
  // otherwise the priority path falls back to the weighted 48-bin metric.
  // When both old bins still qualify, a challenger must exceed the weakest
  // current candidate by 0.05 for TWO consecutive eligible episodes.
  // This prevents small ranking oscillations from churning the Top 2.
  // Returns true only when membership or activation actually changes.
  bool refresh(const CsiLearning &learning, bool weights_changed = false) {
    const uint32_t eligible = learning.eligible_episodes();
    const bool new_episode = eligible != last_eligible_episodes_;
    if (!new_episode && !weights_changed) return false;
    last_eligible_episodes_ = eligible;

    // Snapshot the previous TOP before any replacement. Only these members may
    // use the 4/8 retention rule; new members still need the normal 5/8 gate.
    const size_t previous_count = active_count_;
    size_t previous_top[kMaxTop]{};
    for (size_t j = 0; j < previous_count; ++j) previous_top[j] = top_[j];

    size_t best[kTop]{};
    size_t found = 0;
    if (learning.episode_window_count() == CsiLearning::kEpisodeWindow) {
      // Select every normal 5/8-qualified carrier first, using the existing
      // movement/silence ranking unchanged.
      for (size_t i = 0; i < kBins; ++i) {
        if (!qualified_(learning, i)) continue;
        size_t pos = 0;
        while (pos < found && (rank_(learning, best[pos]) > rank_(learning, i) ||
            (rank_(learning, best[pos]) == rank_(learning, i) && best[pos] < i))) ++pos;
        if (pos >= kTop) continue;
        if (found < kTop) ++found;
        for (size_t j = found - 1; j > pos; --j) best[j] = best[j - 1];
        best[pos] = i;
      }

      // If fewer than two carriers qualify at 5/8, fill the remaining pair
      // positions only with incumbents still at 4/8. Their existing silence
      // quality helps choose which incumbent fills a missing position.
      if (active_ && found < kTop) {
        size_t fallback[kMaxTop]{};
        size_t fallback_count = 0;
        for (size_t j = 0; j < previous_count; ++j) {
          const size_t i = previous_top[j];
          if (learning.hits(i) < kTopRetentionHits) continue;
          bool already_selected = false;
          for (size_t k = 0; k < found; ++k)
            already_selected = already_selected || best[k] == i;
          if (already_selected) continue;

          size_t pos = fallback_count;
          while (pos > 0 && rank_(learning, fallback[pos - 1]) < rank_(learning, i)) {
            fallback[pos] = fallback[pos - 1];
            --pos;
          }
          fallback[pos] = i;
          ++fallback_count;
        }
        for (size_t j = 0; j < fallback_count && found < kTop; ++j)
          best[found++] = fallback[j];
      }
    }
    const bool enough = found == kTop;
    if (!enough) {
      pending_count_ = 0;
      if (!active_) return false;
      active_ = false;
      active_count_ = 0;
      for (size_t j = 0; j < kMaxTop; ++j) top_[j] = 0;
      recalibrate_(learning);
      return true;
    }

    const bool same_pair = active_ &&
        ((top_[0] == best[0] && top_[1] == best[1]) ||
         (top_[0] == best[1] && top_[1] == best[0]));
    if (same_pair) {
      // Update displayed ranking without resetting the detection filter.
      top_[0] = best[0];
      top_[1] = best[1];
      pending_count_ = 0;
      const bool third_changed = refresh_third_(learning);
      if (weights_changed || third_changed) recalibrate_(learning);
      return third_changed;
    }

    bool old_retained = active_;
    if (active_) {
      for (size_t j = 0; j < kTop; ++j)
        old_retained = old_retained && learning.hits(top_[j]) >= kTopRetentionHits;
    }
    if (old_retained) {
      // Only consider a better-ranked pair if the improvement is meaningful
      // and verified in successive independent episodes.
      const float replacement_margin = multiplicative_ranking_ ? 0.05f :
          ((noise_ && noise_->ready()) ? 0.025f : 0.05f);
      if (!new_episode ||
          rank_(learning,best[1]) < std::min(rank_(learning,top_[0]),
                                             rank_(learning,top_[1])) +
              replacement_margin) {
        pending_count_ = 0;
        const bool third_changed = refresh_third_(learning);
        if (weights_changed || third_changed) recalibrate_(learning);
        return third_changed;
      }
      if (pending_count_ != 0 && pending_[0] == best[0] && pending_[1] == best[1]) {
        ++pending_count_;
      } else {
        pending_[0] = best[0];
        pending_[1] = best[1];
        pending_count_ = 1;
      }
      if (pending_count_ < 2) {
        const bool third_changed = refresh_third_(learning);
        if (weights_changed || third_changed) recalibrate_(learning);
        return third_changed;
      }
    }

    // First valid pair, or replacement of an invalid pair: no extra delay.
    active_ = true;
    top_[0] = best[0];
    top_[1] = best[1];
    active_count_ = kTop;
    refresh_third_(learning, previous_top, previous_count);
    pending_count_ = 0;
    recalibrate_(learning);
    return true;
  }

  // The independent priority confirmation retains the original TWO-bin
  // simultaneous-response gate; the global 70/30 is unrestricted.
  bool joint_response(const float *energy, const CsiLearning &learning, float threshold) const {
    if (!active_) return false;
    for (size_t j = 0; j < kTop; ++j) {
      const size_t i = top_[j];
      const float quiet = std::sqrt(learning.quiet_energy(i) + kEpsilon);
      const float current = std::sqrt(energy[i] + kEpsilon);
      const float individual_score = 2.0f * current / std::max(quiet, kQuietFloor);
      if (!(individual_score >= threshold)) return false;
    }
    return true;
  }

  // Two separately normalized populations: the 2-3 selected bins receive
  // 70% of global decision energy; the other 46-45 receive 30%, regardless
  // of their previously learned relative weights.
  float combined_energy(const float *energy, const float *weights, float *selected_energy = nullptr,
                        bool allow_priority = true) const {
    float all_energy = 0.0f, all_weight = 0.0f;
    for (size_t i = 0; i < kBins; ++i) {
      all_energy += energy[i] * weights[i];
      all_weight += weights[i];
    }
    if (!active_) {
      if (selected_energy != nullptr) *selected_energy = 0.0f;
      return all_energy / all_weight;
    }
    float top_energy = 0.0f, top_weight = 0.0f;
    float pair_energy = 0.0f, pair_weight = 0.0f;
    for (size_t j = 0; j < active_count_; ++j) {
      const size_t i = top_[j];
      const float weighted = energy[i] * weights[i];
      top_energy += weighted;
      top_weight += weights[i];
      if (j < kTop) {  // Keep the separate two-bin detector's input unchanged.
        pair_energy += weighted;
        pair_weight += weights[i];
      }
    }
    const float top_mean = top_energy / top_weight;
    const float other_mean = std::max(0.0f, all_energy - top_energy) / (all_weight - top_weight);
    if (selected_energy != nullptr) *selected_energy = pair_energy / pair_weight;
    // The independently gated confirmation retains its own two-bin input;
    // global 70/30 includes all selected members unconditionally.
    if (!allow_priority) return all_energy / all_weight;
    return kTopShare * top_mean + kOtherShare * other_mean;
  }

  float quiet_jitter(const CsiLearning &learning) const {
    float energy[kBins];
    for (size_t i = 0; i < kBins; ++i) energy[i] = learning.quiet_energy(i);
    return std::sqrt(combined_energy(energy, learning.weights()) + kEpsilon);
  }

  // Independent confirmation window. This path may trigger even if the
  // global 70/30 metric is below threshold. It uses its OWN quiet reference.
  bool observe(float selected_energy, float threshold, bool held, bool joint_response,
               bool global_above, float quiet_alpha, float quiet_update_limit) {
    if (!active_) { score_ = 0; return false; }
    const float jitter = std::sqrt(selected_energy + kEpsilon);
    score_ = 2.0f * jitter / std::max(quiet_jitter_, kQuietFloor);
    const bool above = joint_response && score_ >= threshold;
    if (filter_[next_]) --hits_;
    filter_[next_] = above;
    if (above) ++hits_;
    next_ = (next_ + 1) % window_;
    const uint8_t required = held ? 2 : count_;
    const bool confirmed = hits_ >= required;
    if (confirmed) {
      ++confirmations_;
      clear_filter();
    }
    if (!above && !global_above && score_ < quiet_update_limit) {
      quiet_jitter_ += quiet_alpha * (jitter - quiet_jitter_);
      quiet_jitter_ = std::max(quiet_jitter_, kQuietFloor);
    }
    return confirmed;
  }

 private:
  bool qualified_(const CsiLearning &learning, size_t i) const {
    return learning.hits(i) >= CsiLearning::kMinHits &&
           learning.candidate(i) > 1.0001f;
  }

  // The active selector retains its existing ranking. The parallel shadow
  // selector ranks the same 5/8-qualified bins as movement * (1 + silence),
  // with silence quality kept on its existing continuous 0..1 scale.
  float rank_(const CsiLearning &learning, size_t i) const {
    const float movement = std::max(0.0f, std::min(1.0f, learning.candidate(i) - 1.0f));
    const float silence = noise_ != nullptr ? noise_->quality(i) : 0.0f;
    if (multiplicative_ranking_) return movement * (1.0f + silence);
    if (noise_ == nullptr || !noise_->ready()) return learning.candidate(i);
    return 0.5f * movement + 0.5f * silence;
  }

  // Prefer any new 5/8-qualified third bin. If none exists, retain a previous
  // TOP member at 4/8; silence quality orders these fallback members.
  bool refresh_third_(const CsiLearning &learning, const size_t *previous_top = nullptr,
                      size_t previous_count = 0) {
    if (!active_) return false;
    size_t best_third = kBins;
    for (size_t i = 0; i < kBins; ++i) {
      if (i == top_[0] || i == top_[1] ||
          !qualified_(learning, i)) continue;
      if (best_third == kBins || rank_(learning,i) > rank_(learning,best_third) ||
          (rank_(learning,i) == rank_(learning,best_third) && i < best_third))
        best_third = i;
    }
    if (best_third == kBins) {
      if (previous_top != nullptr) {
        for (size_t j = 0; j < previous_count; ++j) {
          const size_t i = previous_top[j];
          if (i == top_[0] || i == top_[1] || learning.hits(i) < kTopRetentionHits) continue;
          if (best_third == kBins || rank_(learning, i) > rank_(learning, best_third))
            best_third = i;
        }
      } else if (active_count_ >= kMaxTop && top_[2] != top_[0] && top_[2] != top_[1] &&
                 learning.hits(top_[2]) >= kTopRetentionHits) {
        best_third = top_[2];
      }
    }
    const size_t next_count = best_third == kBins ? kTop : kMaxTop;
    const bool changed = active_count_ != next_count ||
        (next_count == kMaxTop && top_[2] != best_third);
    active_count_ = next_count;
    top_[2] = next_count == kMaxTop ? best_third : 0;
    return changed;
  }

  void recalibrate_(const CsiLearning &learning) {
    float top_quiet = 0.0f, top_weight = 0.0f;
    if (active_) {
      for (size_t j = 0; j < kTop; ++j) {
        const size_t i = top_[j];
        top_quiet += learning.quiet_energy(i) * learning.weights()[i];
        top_weight += learning.weights()[i];
      }
      top_quiet /= top_weight;
    }
    quiet_jitter_ = std::sqrt(top_quiet + kEpsilon);
    score_ = 0.0f;
    clear_filter();
  }

  size_t top_[kMaxTop]{};
  size_t active_count_{0};
  const CsiNoiseLearning *noise_{nullptr};
  size_t pending_[kTop]{};
  uint8_t pending_count_{0};
  uint32_t last_eligible_episodes_{0};
  bool active_{false};
  bool multiplicative_ranking_{false};
  bool filter_[kMaxWindow]{};
  size_t window_{8}, next_{0};
  uint8_t count_{3}, hits_{0};
  float quiet_jitter_{0.02f}, score_{0};
  uint32_t confirmations_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
