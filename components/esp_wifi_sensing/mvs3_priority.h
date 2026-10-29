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

  // Loss of 5/8 recurrence is acted upon at the end of the NEW eligible
  // episode, not deferred until the next weight revision. The selector may
  // reactivate ONLY in configure(), at a guarded revision.
  bool suspend_if_unqualified(const CsiLearning &learning) {
    if (!active_) return false;
    bool qualified = learning.episode_window_count() == CsiLearning::kEpisodeWindow;
    for (size_t j = 0; j < kTop; ++j) {
      const size_t i = top_[j];
      qualified = qualified && learning.hits(i) >= CsiLearning::kMinHits &&
                  learning.candidate(i) > 1.0001f;
    }
    if (qualified) return false;
    active_ = false;
    for (size_t j = 0; j < kTop; ++j) top_[j] = 0;
    reset_after_gap();
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

  // Selection occurs only at a guarded learning revision. It cannot be
  // promoted just because a single fresh episode changes the candidate rank.
  void configure(const CsiLearning &learning) {
    size_t selected[kTop]{};
    active_ = learning.select_top_two(selected);
    for (size_t j = 0; j < kTop; ++j) top_[j] = active_ ? selected[j] : 0;
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
    score_ = 0;
    clear_filter();
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
    if (!active_ || !allow_priority) {
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
  size_t top_[kTop]{};
  bool active_{false};
  bool filter_[kMaxWindow]{};
  size_t window_{8}, next_{0};
  uint8_t count_{3}, hits_{0};
  float quiet_jitter_{0.02f}, score_{0};
  uint32_t confirmations_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
