#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "csi_learning.h"

namespace esphome {
namespace esp_wifi_sensing {

// Noise-only companion to movement learning. Never writes to movement
// candidates, qualifying votes or the reference MVS1 detector.
// A full uninterrupted hour of eligible observations is required before
// any noise score can participate in priority selection.
class CsiNoiseLearning {
 public:
  static constexpr size_t kBins = CsiLearning::kBins;
  static constexpr uint32_t kMinuteMs = 60000U;
  static constexpr uint32_t kHourMs = 3600000U;
  static constexpr size_t kMinutes = 60;
  static constexpr size_t kWindow = 8;
  static constexpr float kEpsilon = 1.0e-4f;
  static constexpr float kFloor = 0.003f;

  bool ready() const { return ready_; }
  uint32_t accepted_minutes() const { return accepted_minutes_; }
  uint32_t provisional_minutes() const { return pending_minutes_; }
  uint32_t interruptions() const { return interruptions_; }
  float quality(size_t i) const { return ready_ ? quality_[i] : 0.0f; }

  // True if at least one of MVS1/MVS3 confirmed a detection, is in its
  // 120-second hold, or a CSI discontinuity prevented a trustworthy hour.
  void interrupt() {
    if (!started_) return;
    ++interruptions_;
    started_ = false;
    session_validated_ = false;
    pending_minutes_ = 0;
    // The pending slot counter invalidates old minutes; avoid wiping 24 KiB
    // in the time-sensitive Wi-Fi callback on every movement transition.
    current_ = Minute{};
    for (size_t i=0; i<kBins; ++i) recent_[i] = 0;
    // Keep the last VERIFIED baseline; motion never trains noise.
  }

  void observe(const float *energy, const CsiLearning &learning,
               uint32_t now, bool quiet_allowed, float movement_threshold) {
    if (!quiet_allowed || !learning.ready()) {
      interrupt();
      return;
    }
    if (!started_) {
      started_ = true;
      began_at_ = minute_started_ = last_observation_ = now;
      current_ = Minute{};
      for (size_t i=0; i<kBins; ++i) recent_[i] = 0;
      return;  // First sample is only a timing seed.
    }
    const uint32_t elapsed = static_cast<uint32_t>(now - last_observation_);
    if (elapsed > 2000U || elapsed == 0) {
      interrupt();
      started_ = true;
      began_at_ = minute_started_ = last_observation_ = now;
      return;
    }
    last_observation_ = now;
    if (static_cast<uint32_t>(now - minute_started_) >= kMinuteMs) {
      finish_minute_();
      minute_started_ = now;
      current_ = Minute{};
    }
    // Use half the motion threshold for subthreshold noise characterization.
    // Motion confirmation itself remains at the configured full threshold.
    const float peak_threshold = std::max(1.0f, movement_threshold * 0.5f);
    ++current_.observations;
    for (size_t i=0; i<kBins; ++i) {
      const float quiet = std::sqrt(learning.quiet_energy(i) + kEpsilon);
      const float current = std::sqrt(std::max(energy[i], 0.0f) + kEpsilon);
      const float score = 2.0f * current / std::max(quiet, kFloor);
      const bool peak = std::isfinite(score) && score >= peak_threshold;
      recent_[i] = static_cast<uint8_t>((recent_[i] << 1U) | (peak ? 1U : 0U));
      if (peak) {
        ++current_.peaks[i];
        current_.intensity[i] += std::min(3.0f, score / peak_threshold - 1.0f) / 3.0f;
      }
      if (__builtin_popcount(static_cast<unsigned>(recent_[i])) >= 3)
        ++current_.persistent[i];
    }
  }

  // Silentest three *across all 48*, not only 5/8-qualified movement bins.
  void quietest_three(size_t (&bins)[3]) const {
    bins[0]=0; bins[1]=1; bins[2]=2;
    if (!ready_) return;
    auto better = [this](size_t a, size_t b) {
      return quality_[a] > quality_[b] ||
          (quality_[a] == quality_[b] && a < b);
    };
    for (size_t a=1; a<3; ++a)
      for (size_t b=a; b>0 && better(bins[b], bins[b-1]); --b)
        std::swap(bins[b], bins[b-1]);
    for (size_t i=3; i<kBins; ++i) {
      for (size_t p=0; p<3; ++p) {
        if (better(i, bins[p])) {
          for (size_t j=2; j>p; --j) bins[j]=bins[j-1];
          bins[p]=i;
          break;
        }
      }
    }
  }

 private:
  struct Minute {
    uint32_t observations{0};
    uint16_t peaks[kBins]{};
    uint16_t persistent[kBins]{};
    float intensity[kBins]{};
  };
  // Exactly 60 provisional 1-minute summaries, RAM only (~24 KiB).
  Minute pending_[kMinutes]{};
  Minute current_{};
  uint8_t recent_[kBins]{};
  float quality_[kBins]{};
  uint32_t began_at_{0}, minute_started_{0}, last_observation_{0};
  uint32_t accepted_minutes_{0}, interruptions_{0};
  size_t pending_minutes_{0};
  bool started_{false}, session_validated_{false}, ready_{false};

  static float noise_penalty_(uint64_t peaks, uint64_t persistent,
                              double intensity, uint64_t observations) {
    if (!observations) return 0.0f;
    // Three equally weighted terms, each normalized by ALL quiet observations:
    // frequency, capped peak intensity per observation, and 3/8 persistence.
    // In particular, a single extreme peak can NEVER contribute 1/3 of the
    // penalty by itself, as happened with the old intensity / peaks formula.
    const double n = static_cast<double>(observations);
    const float frequency = static_cast<float>(std::min(1.0, static_cast<double>(peaks) / n));
    const float strength = static_cast<float>(std::min(1.0, intensity / n));
    const float persistence = static_cast<float>(std::min(1.0, static_cast<double>(persistent) / n));
    return (frequency + strength + persistence) / 3.0f;
  }

  void finish_minute_() {
    if (current_.observations == 0) { interrupt(); return; }
    if (!session_validated_) {
      if (pending_minutes_ < kMinutes)
        pending_[pending_minutes_++] = current_;
      if (pending_minutes_ == kMinutes &&
          static_cast<uint32_t>(last_observation_ - began_at_) >= kHourMs) {
        uint64_t total_obs = 0;
        uint64_t peak[kBins]{}, persistent[kBins]{};
        double intensity[kBins]{};
        for (size_t m=0; m<kMinutes; ++m) {
          total_obs += pending_[m].observations;
          for (size_t i=0; i<kBins; ++i) {
            peak[i] += pending_[m].peaks[i];
            persistent[i] += pending_[m].persistent[i];
            intensity[i] += pending_[m].intensity[i];
          }
        }
        for (size_t i=0; i<kBins; ++i) {
          const float new_quality = 1.0f - noise_penalty_(peak[i], persistent[i],
                                                          intensity[i], total_obs);
          // All new silent sessions must pass their own 60-minute validation.
          // A previous verified baseline survives movement interruptions.
          quality_[i] = ready_ ? 0.5f * quality_[i] + 0.5f * new_quality : new_quality;
        }
        accepted_minutes_ += kMinutes;
        ready_ = true;
        session_validated_ = true;
        // The hour is now consolidated; old provisional slots need no wipe.

      }
    } else {
      // Keep learning while silence continues, with bounded memory.
      for (size_t i=0; i<kBins; ++i) {
        const float new_quality = 1.0f -
            noise_penalty_(current_.peaks[i], current_.persistent[i],
                           current_.intensity[i], current_.observations);
        quality_[i] = 0.98f * quality_[i] + 0.02f * new_quality;
      }
      ++accepted_minutes_;
    }
  }
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
