#include "mvs_algorithm.h"

namespace esphome {
namespace esp_wifi_sensing {

namespace {
constexpr float EPS = 1.0e-4f;
constexpr float QUIET_FLOOR = 0.003f;
constexpr float QUIET_ALPHA = 0.01f;
constexpr float QUIET_UPDATE_LIMIT = 3.0f;
constexpr uint8_t FILTER_COUNT = 3;
constexpr uint32_t QUIET_BOOT_SAMPLES = 20;
}  // namespace

MvsResult MvsAlgorithm::process(const ParsedCsiPacket &packet, uint32_t now_ms) {
  float frame[kBins]{};
  if (!this->make_frame_(packet, frame)) {
    return MvsResult{this->quiet_samples_ >= QUIET_BOOT_SAMPLES, this->motion_state_, this->last_score_, false,
                     this->total_observations_};
  }

  float observation[kBins]{};
  if (!this->make_observation_(frame, now_ms, observation)) {
    return MvsResult{this->quiet_samples_ >= QUIET_BOOT_SAMPLES, this->motion_state_, this->last_score_, false,
                     this->total_observations_};
  }
  this->total_observations_++;

  const float jitter = this->temporal_jitter_(observation);
  if (!this->have_previous_profile_) {
    return MvsResult{false, false, 0.0f, true, this->total_observations_};
  }

  // Espressif's motion path is based on CSI waveform jitter, not distance from
  // a long-lived room baseline. Normalise current temporal jitter by the quiet
  // jitter floor so the existing threshold remains a dimensionless sensitivity.
  this->last_score_ = 2.0f * jitter / std::max(this->quiet_jitter_, QUIET_FLOOR);

  if (this->quiet_samples_ < QUIET_BOOT_SAMPLES) {
    this->quiet_jitter_ += (jitter - this->quiet_jitter_) / static_cast<float>(this->quiet_samples_ + 1);
    this->quiet_jitter_ = std::max(this->quiet_jitter_, QUIET_FLOOR);
    this->quiet_samples_++;
    this->clear_filter_();
    return MvsResult{this->quiet_samples_ >= QUIET_BOOT_SAMPLES, false, this->last_score_, true,
                     this->total_observations_};
  }

  const bool above = this->last_score_ >= this->threshold_;
  const bool confirmed = this->update_filter_(above);

  // Equivalent in spirit to Espressif filter_window/filter_count: isolated CSI
  // outliers cannot trigger motion; several jitter excursions inside a short
  // window are required. Once confirmed, consume that window so a static RF
  // offset cannot repeatedly retrigger the 120 s hold.
  if (confirmed) {
    this->motion_state_ = true;
    this->last_motion_time_ = now_ms;
    this->clear_filter_();
  }

  // Learn only genuinely quiet temporal jitter. A permanent change to the room
  // produces a transient and then becomes quiet automatically; no room baseline
  // reset is required and there is no stale absolute score to retrigger later.
  if (!above && this->last_score_ < QUIET_UPDATE_LIMIT) {
    this->quiet_jitter_ += QUIET_ALPHA * (jitter - this->quiet_jitter_);
    this->quiet_jitter_ = std::max(this->quiet_jitter_, QUIET_FLOOR);
  }

  if (this->motion_state_ && static_cast<uint32_t>(now_ms - this->last_motion_time_) >= this->hold_time_ms_) {
    this->motion_state_ = false;
    this->clear_filter_();
  }

  return MvsResult{true, this->motion_state_, this->last_score_, true, this->total_observations_};
}

bool MvsAlgorithm::make_frame_(const ParsedCsiPacket &packet, float *frame) const {
  if (!packet.layout_supported || packet.count < kBins) return false;

  float sums[kBins]{};
  uint16_t counts[kBins]{};
  for (size_t i = 0; i < packet.count; i++) {
    const size_t bin = (i * kBins) / packet.count;
    sums[bin] += std::log1pf(packet.subcarriers[i].amplitude);
    counts[bin]++;
  }
  for (size_t i = 0; i < kBins; i++) {
    if (counts[i] == 0) return false;
    frame[i] = sums[i] / counts[i];
  }
  return true;
}

bool MvsAlgorithm::make_observation_(const float *frame, uint32_t now_ms, float *observation) {
  if (!this->timing_started_) {
    this->timing_started_ = true;
    this->last_observation_ms_ = now_ms;
  }

  for (size_t i = 0; i < kBins; i++) this->accumulator_[i] += frame[i];
  if (this->accumulator_count_ < UINT16_MAX) this->accumulator_count_++;

  if (static_cast<uint32_t>(now_ms - this->last_observation_ms_) < kUpdateIntervalMs) return false;

  const float inv = this->accumulator_count_ > 0 ? 1.0f / this->accumulator_count_ : 1.0f;
  for (size_t i = 0; i < kBins; i++) {
    observation[i] = this->accumulator_[i] * inv;
    this->accumulator_[i] = 0.0f;
  }
  this->accumulator_count_ = 0;
  this->last_observation_ms_ = now_ms;
  return true;
}

float MvsAlgorithm::temporal_jitter_(const float *observation) {
  float mean = 0.0f;
  for (size_t i = 0; i < kBins; i++) mean += observation[i];
  mean /= static_cast<float>(kBins);

  float profile[kBins]{};
  for (size_t i = 0; i < kBins; i++) profile[i] = observation[i] - mean;

  if (!this->have_previous_profile_) {
    for (size_t i = 0; i < kBins; i++) this->previous_profile_[i] = profile[i];
    this->have_previous_profile_ = true;
    return 0.0f;
  }

  float energy = 0.0f;
  for (size_t i = 0; i < kBins; i++) {
    const float delta = profile[i] - this->previous_profile_[i];
    energy += delta * delta;
    this->previous_profile_[i] = profile[i];
  }
  return std::sqrt(energy / static_cast<float>(kBins) + EPS);
}

bool MvsAlgorithm::update_filter_(bool above) {
  if (this->filter_[this->filter_next_]) this->filter_hits_--;
  this->filter_[this->filter_next_] = above;
  if (above) this->filter_hits_++;
  this->filter_next_ = (this->filter_next_ + 1) % kFilterWindow;
  return this->filter_hits_ >= FILTER_COUNT;
}

void MvsAlgorithm::clear_filter_() {
  for (size_t i = 0; i < kFilterWindow; i++) this->filter_[i] = false;
  this->filter_next_ = 0;
  this->filter_hits_ = 0;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
