#include "experimental_mvs_algorithm.h"

namespace esphome {
namespace esp_wifi_sensing {

namespace {
constexpr float JITTER_EPSILON = 1.0e-4f;
constexpr float QUIET_FLOOR = 0.003f;
}  // namespace

MvsResult ExperimentalMvsAlgorithm::process(const ParsedCsiPacket &packet, uint32_t now_ms) {
  // A gap/layout change is not a movement episode. Drop partial windows.
  if (packet.layout_supported && packet.count >= kBins) {
    if (packet_seen_ && (static_cast<uint32_t>(now_ms - last_packet_ms_) >
        std::max<uint32_t>(2000U, update_interval_ms_ * 3U) || packet.count != last_packet_count_)) {
      learning_.discontinuity();
      have_previous_profile_ = false;
      accumulator_count_ = 0;
      timing_started_ = false;
      for (float &v : accumulator_) v = 0;
      clear_filter_();
      priority_.reset_after_gap();
    }
    packet_seen_ = true;
    last_packet_ms_ = now_ms;
    last_packet_count_ = packet.count;
  }
  float frame[kBins]{};
  if (!this->make_frame_(packet, frame)) {
    return MvsResult{this->quiet_samples_ >= this->quiet_boot_samples_, this->motion_state_, this->last_score_, false,
                     this->total_observations_};
  }

  float observation[kBins]{};
  if (!this->make_observation_(frame, now_ms, observation)) {
    return MvsResult{this->quiet_samples_ >= this->quiet_boot_samples_, this->motion_state_, this->last_score_, false,
                     this->total_observations_};
  }
  this->total_observations_++;

  const bool had_profile = have_previous_profile_;
  const float jitter = this->temporal_jitter_(observation);
  // Preserve initial reference boot semantics; after a discontinuity/revision
  // the seed observation must not recalibrate or produce a decision.
  if (!had_profile && total_observations_ > 1) {
    return MvsResult{false, false, 0.0f, true, this->total_observations_};
  }

  // Espressif's motion path is based on CSI waveform jitter, not distance from
  // a long-lived room baseline. Normalise current temporal jitter by the quiet
  // jitter floor so the existing threshold remains a dimensionless sensitivity.
  this->last_score_ = 2.0f * jitter / std::max(this->quiet_jitter_, QUIET_FLOOR);

  if (this->quiet_samples_ < this->quiet_boot_samples_) {
    this->quiet_jitter_ += (jitter - this->quiet_jitter_) / static_cast<float>(this->quiet_samples_ + 1);
    this->quiet_jitter_ = std::max(this->quiet_jitter_, QUIET_FLOOR);
    this->quiet_samples_++;
    learning_.observe(raw_energy_, now_ms, false, false);
    this->clear_filter_();
    return MvsResult{this->quiet_samples_ >= this->quiet_boot_samples_, false, this->last_score_, true,
                     this->total_observations_};
  }

  // The ESPHome loop can expire the published ON state even when CSI is absent.
  // Synchronize the algorithm's internal state before choosing the confirmation
  // requirement, so stale ON never gets the more permissive renewal threshold.
  if (this->motion_state_ &&
      static_cast<uint32_t>(now_ms - this->last_motion_time_) >= this->hold_time_ms_) {
    this->motion_state_ = false;
    this->clear_filter_();
    this->priority_.clear_filter();
  }

  const bool above = this->last_score_ >= this->threshold_;
  const bool global_confirmed = this->update_filter_(above);
  const bool priority_confirmed = this->priority_.observe(
      selected_energy_, threshold_, motion_state_, above,
      quiet_alpha_, quiet_update_limit_);
  const bool confirmed = global_confirmed || priority_confirmed;

  // Equivalent in spirit to Espressif filter_window/filter_count: isolated CSI
  // outliers cannot trigger motion; several jitter excursions inside a short
  // window are required. Once confirmed, consume that window so a static RF
  // offset cannot repeatedly retrigger the 120 s hold.
  if (confirmed) {
    this->motion_state_ = true;
    this->last_motion_time_ = now_ms;
    this->clear_filter_();
    this->priority_.clear_filter();
  }

  // Learn only genuinely quiet temporal jitter. A permanent change to the room
  // produces a transient and then becomes quiet automatically; no room baseline
  // reset is required and there is no stale absolute score to retrigger later.
  if (!above && (!priority_.active() || priority_.score() < threshold_) &&
      this->last_score_ < this->quiet_update_limit_) {
    this->quiet_jitter_ += this->quiet_alpha_ * (jitter - this->quiet_jitter_);
    this->quiet_jitter_ = std::max(this->quiet_jitter_, QUIET_FLOOR);
  }

  if (learning_.observe(raw_energy_, now_ms, confirmed, motion_state_)) {
    // Reproject independent raw quiet energy into the new weighted metric.
    // Never mix old-scale jitter/filter history with the new weights.
    // Select up to four evidence-qualified bins ONLY on a guarded revision.
    // With fewer than four, preserve the original 48-bin MVS3 metric.
    priority_.configure(learning_);
    quiet_jitter_ = priority_.quiet_jitter(learning_);
    quiet_samples_ = quiet_boot_samples_;
    have_previous_profile_ = false;
    clear_filter_();
    last_score_ = 0;
  }
  return MvsResult{true, this->motion_state_, this->last_score_, true, this->total_observations_};
}

bool ExperimentalMvsAlgorithm::make_frame_(const ParsedCsiPacket &packet, float *frame) const {
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

bool ExperimentalMvsAlgorithm::make_observation_(const float *frame, uint32_t now_ms, float *observation) {
  if (!this->timing_started_) {
    this->timing_started_ = true;
    this->last_observation_ms_ = now_ms;
  }

  for (size_t i = 0; i < kBins; i++) this->accumulator_[i] += frame[i];
  if (this->accumulator_count_ < UINT16_MAX) this->accumulator_count_++;

  if (static_cast<uint32_t>(now_ms - this->last_observation_ms_) < this->update_interval_ms_) return false;

  const float inv = this->accumulator_count_ > 0 ? 1.0f / this->accumulator_count_ : 1.0f;
  for (size_t i = 0; i < kBins; i++) {
    observation[i] = this->accumulator_[i] * inv;
    this->accumulator_[i] = 0.0f;
  }
  this->accumulator_count_ = 0;
  this->last_observation_ms_ = now_ms;
  return true;
}

float ExperimentalMvsAlgorithm::temporal_jitter_(const float *observation) {
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

  for (size_t i = 0; i < kBins; i++) {
    const float delta = profile[i] - this->previous_profile_[i];
    raw_energy_[i] = delta * delta;
    this->previous_profile_[i] = profile[i];
  }
  // Before four qualified bins exist, this is exactly the former 48-bin
  // weighted average. Once qualified, the top four provide 70% of global
  // decision energy; the other 44 contribute the remaining 30%.
  const float energy = priority_.combined_energy(
      raw_energy_, learning_.weights(), &selected_energy_);
  return std::sqrt(energy + JITTER_EPSILON);
}

bool ExperimentalMvsAlgorithm::update_filter_(bool above) {
  if (this->filter_[this->filter_next_]) this->filter_hits_--;
  this->filter_[this->filter_next_] = above;
  if (above) this->filter_hits_++;
  this->filter_next_ = (this->filter_next_ + 1) % this->filter_window_;
  // OFF -> ON uses the configured count (3 by default).
  // While ON, two fresh peaks in the same window renew the 120-second hold.
  const uint8_t required_hits = this->motion_state_ ? 2 : this->filter_count_;
  return this->filter_hits_ >= required_hits;
}

void ExperimentalMvsAlgorithm::clear_filter_() {
  for (size_t i = 0; i < this->filter_window_; i++) this->filter_[i] = false;
  this->filter_next_ = 0;
  this->filter_hits_ = 0;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome

