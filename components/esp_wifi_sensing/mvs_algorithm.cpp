#include "mvs_algorithm.h"

namespace esphome {
namespace esp_wifi_sensing {

namespace {
constexpr float EPS = 1.0e-3f;
constexpr uint32_t BASELINE_BOOT_SAMPLES = 20;
constexpr float NOISE_FLOOR = 0.02f;
constexpr float BASELINE_ALPHA_QUIET = 0.006f;
constexpr float BASELINE_ALPHA_CHANNEL = 0.0015f;
constexpr uint8_t ENTER_OBSERVATIONS = 3;
constexpr uint8_t EXIT_OBSERVATIONS = 3;
constexpr size_t EXIT_RECENT_SAMPLES = 4;
}  // namespace

MvsResult MvsAlgorithm::process(const ParsedCsiPacket &packet, uint32_t now_ms) {
  float frame[kBins]{};
  if (!this->make_frame_(packet, frame)) {
    return MvsResult{this->baseline_initialized_, this->motion_state_, this->last_score_, false, this->total_observations_};
  }

  float observation[kBins]{};
  if (!this->make_observation_(frame, now_ms, observation)) {
    return MvsResult{this->baseline_initialized_, this->motion_state_, this->last_score_, false, this->total_observations_};
  }
  this->total_observations_++;

  if (!this->baseline_initialized_) {
    this->update_baseline_(observation, false);
    this->last_score_ = 0.0f;
    return MvsResult{false, false, this->last_score_, true, this->total_observations_};
  }

  const FrameFeatures features = this->compute_features_(observation);
  this->push_history_(features);
  this->last_score_ = this->score_window_();
  const bool motion = this->update_fsm_(this->last_score_, now_ms);

  const bool quiet = this->last_score_ < (this->threshold_ * 0.45f);
  const bool common_channel_shift = features.common_ratio > 0.78f && this->last_score_ < (this->threshold_ * 0.85f);
  if (!motion && (quiet || common_channel_shift)) {
    this->update_baseline_(observation, common_channel_shift);
  }

  return MvsResult{this->history_count_ >= kWindowSamples / 2, motion, this->last_score_, true, this->total_observations_};
}

bool MvsAlgorithm::make_frame_(const ParsedCsiPacket &packet, float *frame) const {
  if (!packet.layout_supported || packet.count < kBins) {
    return false;
  }

  float sums[kBins]{};
  uint16_t counts[kBins]{};
  for (size_t i = 0; i < packet.count; i++) {
    const size_t bin = (i * kBins) / packet.count;
    sums[bin] += std::log1pf(packet.subcarriers[i].amplitude);
    counts[bin]++;
  }

  for (size_t i = 0; i < kBins; i++) {
    if (counts[i] == 0) {
      return false;
    }
    frame[i] = sums[i] / counts[i];
  }
  return true;
}

bool MvsAlgorithm::make_observation_(const float *frame, uint32_t now_ms, float *observation) {
  // CSI packets may arrive much faster than 1 Hz. Accumulate all frames, but only
  // advance the MVS temporal window when real elapsed time reaches kUpdateIntervalMs.
  if (!this->timing_started_) {
    this->timing_started_ = true;
    this->last_observation_ms_ = now_ms;
  }

  for (size_t i = 0; i < kBins; i++) {
    this->accumulator_[i] += frame[i];
  }
  if (this->accumulator_count_ < UINT16_MAX) {
    this->accumulator_count_++;
  }

  if (now_ms - this->last_observation_ms_ < kUpdateIntervalMs) {
    return false;
  }

  const float inv = this->accumulator_count_ > 0 ? 1.0f / this->accumulator_count_ : 1.0f;
  for (size_t i = 0; i < kBins; i++) {
    observation[i] = this->accumulator_[i] * inv;
    this->accumulator_[i] = 0.0f;
  }
  this->accumulator_count_ = 0;
  this->last_observation_ms_ = now_ms;
  return true;
}

MvsAlgorithm::FrameFeatures MvsAlgorithm::compute_features_(const float *frame) const {
  // Keep the original per-bin amplitude profile for common-mode estimation; only
  // remove the common component after it has been measured.
  float delta[kBins]{};
  float common = 0.0f;
  for (size_t i = 0; i < kBins; i++) {
    delta[i] = frame[i] - baseline_[i];
    common += delta[i];
  }
  common /= kBins;

  float total_energy = 0.0f;
  float spatial_energy = 0.0f;
  float residual_abs = 0.0f;
  float roughness = 0.0f;
  for (size_t i = 0; i < kBins; i++) {
    const float norm = std::max(noise_[i], NOISE_FLOOR);
    const float normalized_delta = delta[i] / norm;
    const float residual = (delta[i] - common) / norm;
    total_energy += normalized_delta * normalized_delta;
    spatial_energy += residual * residual;
    residual_abs += std::fabs(residual);
    if (i > 0) {
      const float prev_residual = delta[i - 1] - common;
      const float edge_norm = std::max((noise_[i] + noise_[i - 1]) * 0.5f, NOISE_FLOOR);
      roughness += std::fabs(((delta[i] - common) - prev_residual) / edge_norm);
    }
  }

  const float common_energy = (common * common * kBins) / (NOISE_FLOOR * NOISE_FLOOR);
  return FrameFeatures{
      std::sqrt(spatial_energy / kBins),
      residual_abs / kBins,
      roughness / (kBins - 1),
      common_energy / (common_energy + spatial_energy + EPS),
  };
}

void MvsAlgorithm::update_baseline_(const float *frame, bool channel_drift) {
  if (baseline_samples_ == 0) {
    for (size_t i = 0; i < kBins; i++) {
      baseline_[i] = frame[i];
      noise_[i] = 0.08f;
    }
    baseline_samples_ = 1;
    return;
  }

  if (!baseline_initialized_) {
    for (size_t i = 0; i < kBins; i++) {
      const float delta = frame[i] - baseline_[i];
      baseline_[i] += delta / static_cast<float>(baseline_samples_ + 1);
      noise_[i] += (std::fabs(delta) - noise_[i]) / static_cast<float>(baseline_samples_ + 1);
      noise_[i] = std::max(noise_[i], NOISE_FLOOR);
    }
    baseline_samples_++;
    baseline_initialized_ = baseline_samples_ >= BASELINE_BOOT_SAMPLES;
    return;
  }

  const float alpha = channel_drift ? BASELINE_ALPHA_CHANNEL : BASELINE_ALPHA_QUIET;
  for (size_t i = 0; i < kBins; i++) {
    const float delta = frame[i] - baseline_[i];
    baseline_[i] += alpha * delta;
    noise_[i] = std::max(NOISE_FLOOR, noise_[i] + alpha * (std::fabs(delta) - noise_[i]));
  }
}

void MvsAlgorithm::push_history_(const FrameFeatures &features) {
  history_[history_next_] = features;
  history_next_ = (history_next_ + 1) % kWindowSamples;
  if (history_count_ < kWindowSamples) history_count_++;
}

float MvsAlgorithm::score_window_() const {
  if (history_count_ == 0) return 0.0f;
  float residual = 0.0f, mad = 0.0f, rough = 0.0f, temporal = 0.0f, rf_penalty = 0.0f;
  size_t active_frames = 0;
  for (size_t n = 0; n < history_count_; n++) {
    const size_t idx = (history_next_ + kWindowSamples - history_count_ + n) % kWindowSamples;
    const FrameFeatures &f = history_[idx];
    residual += f.residual_rms;
    mad += f.residual_mad;
    rough += f.spatial_roughness;
    rf_penalty += f.common_ratio;
    if ((f.residual_rms + f.residual_mad + f.spatial_roughness) > 4.0f) {
      active_frames++;
    }
    if (n > 0) {
      const size_t prev = (idx + kWindowSamples - 1) % kWindowSamples;
      temporal += std::fabs(f.residual_rms - history_[prev].residual_rms) + 0.4f * std::fabs(f.spatial_roughness - history_[prev].spatial_roughness);
    }
  }
  const float inv = 1.0f / history_count_;
  const float motion_pattern = (1.8f * residual + 2.4f * mad + 1.5f * rough) * inv;
  const float temporal_texture = history_count_ > 1 ? temporal / (history_count_ - 1) : 0.0f;
  const float common_factor = 1.0f - std::min(0.75f, (rf_penalty * inv) * 0.75f);
  const float active_factor = std::min(1.0f, static_cast<float>(active_frames) / 4.0f);
  return std::max(0.0f, (motion_pattern + 1.2f * temporal_texture) * common_factor * active_factor);
}

bool MvsAlgorithm::update_fsm_(float score, uint32_t now_ms) {
  const float enter = threshold_;
  const float exit = threshold_ * 0.62f;

  if (score > enter) {
    enter_count_ = std::min<uint8_t>(ENTER_OBSERVATIONS, enter_count_ + 1);
    exit_count_ = 0;
  } else {
    enter_count_ = 0;

    // Once the mandatory hold has expired, do not use the full 32-second window
    // for EXIT. That window intentionally remembers recent movement and would
    // otherwise keep the FSM ON long after the physical event has ended.
    // EXIT is based on the most recent observations, giving the FSM a real-time
    // recovery path while preserving hysteresis and 3-sample confirmation.
    float recent_residual = 0.0f;
    float recent_mad = 0.0f;
    float recent_rough = 0.0f;
    float recent_common = 0.0f;
    const size_t recent_count = std::min(history_count_, EXIT_RECENT_SAMPLES);
    for (size_t n = 0; n < recent_count; n++) {
      const size_t idx = (history_next_ + kWindowSamples - recent_count + n) % kWindowSamples;
      const FrameFeatures &f = history_[idx];
      recent_residual += f.residual_rms;
      recent_mad += f.residual_mad;
      recent_rough += f.spatial_roughness;
      recent_common += f.common_ratio;
    }
    const float recent_inv = recent_count > 0 ? 1.0f / recent_count : 1.0f;
    const float recent_common_factor = 1.0f - std::min(0.75f, recent_common * recent_inv * 0.75f);
    const float recent_score = std::max(
        0.0f, (1.8f * recent_residual + 2.4f * recent_mad + 1.5f * recent_rough) * recent_inv * recent_common_factor);

    if (recent_score < exit) {
      exit_count_ = std::min<uint8_t>(EXIT_OBSERVATIONS, exit_count_ + 1);
    } else {
      exit_count_ = 0;
    }
  }

  if (enter_count_ >= ENTER_OBSERVATIONS) {
    motion_state_ = true;
    last_motion_time_ = now_ms;
  }

  if (motion_state_) {
    if (now_ms - last_motion_time_ < hold_time_ms_) {
      return true;
    }
    if (exit_count_ >= EXIT_OBSERVATIONS) {
      motion_state_ = false;
    }
  }
  return motion_state_;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
