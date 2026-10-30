#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "csi_learning.h"

namespace esphome {
namespace esp_wifi_sensing {

// Only the experimental MVS-3 uses this independent decision path.
// Before two bins qualify, the ordinary weighted 48-bin score is retained.
class Mvs3Priority {
 public:
  static constexpr size_t kTop = 2;
  static constexpr size_t kBins = CsiLearning::kBins;
  static constexpr size_t kMaxWindow = 64;
  static constexpr float kTopShare = 0.70f;
  static constexpr float kOtherShare = 0.30f;
  static constexpr float kEpsilon = 1.0e-4f;
  static constexpr float kQuietFloor = 0.003f;

  bool active() const { return active_; }
  size_t top_bin(size_t index) const { return top_[index]; }
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

    size_t best[kTop]{};
    const bool enough = learning.select_top_two(best);
    if (!enough) {
      pending_count_ = 0;
      if (!active_) return false;
      active_ = false;
      for (size_t j = 0; j < kTop; ++j) top_[j] = 0;
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
      if (weights_changed) recalibrate_(learning);
      return false;
    }

    bool old_qualified = active_;
    if (active_) {
      for (size_t j = 0; j < kTop; ++j) {
        const size_t i = top_[j];
        old_qualified = old_qualified &&
            learning.hits(i) >= CsiLearning::kMinHits &&
            learning.candidate(i) > 1.0001f;
      }
    }
    if (old_qualified) {
      // Only consider a better-ranked pair if the improvement is meaningful
      // and verified in successive independent episodes.
      if (!new_episode ||
          learning.candidate(best[1]) < std::min(learning.candidate(top_[0]),
                                                learning.candidate(top_[1])) + 0.05f) {
        pending_count_ = 0;
        if (weights_changed) recalibrate_(learning);
        return false;
      }
      if (pending_count_ != 0 && pending_[0] == best[0] && pending_[1] == best[1]) {
        ++pending_count_;
      } else {
        pending_[0] = best[0];
        pending_[1] = best[1];
        pending_count_ = 1;
      }
      if (pending_count_ < 2) {
        if (weights_changed) recalibrate_(learning);
        return false;
      }
    }

    // First valid pair, or replacement of an invalid pair: no extra delay.
    active_ = true;
    top_[0] = best[0];
    top_[1] = best[1];
    pending_count_ = 0;
    recalibrate_(learning);
    return true;
  }

  // Both priority bins must show individual signal energy substantially above
  // their OWN learned quiet energy in the SAME observation. One strong noisy
  // bin cannot authorize either priority decision path.
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

  // Two separately normalized populations: the two evidence-qualified
  // bins receive 70% of the global decision energy; the other 46 receive
  // 30%, regardless of their previously learned relative weights.
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
    for (size_t j = 0; j < kTop; ++j) {
      const size_t i = top_[j];
      top_energy += energy[i] * weights[i];
      top_weight += weights[i];
    }
    const float top_mean = top_energy / top_weight;
    const float other_mean = std::max(0.0f, all_energy - top_energy) / (all_weight - top_weight);
    if (selected_energy != nullptr) *selected_energy = top_mean;
    // Keep the two-bin diagnostic/quiet update fed even when only one bin
    // reacts; the actual global decision safely falls back to all 48 bins.
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

  size_t top_[kTop]{};
  size_t pending_[kTop]{};
  uint8_t pending_count_{0};
  uint32_t last_eligible_episodes_{0};
  bool active_{false};
  bool filter_[kMaxWindow]{};
  size_t window_{8}, next_{0};
  uint8_t count_{3}, hits_{0};
  float quiet_jitter_{0.02f}, score_{0};
  uint32_t confirmations_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
