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
  // The 70/30 and ordinary 48-bin metrics have different energy scales.
  // Compare each only with its own quiet reference.
  const float &quiet_reference = using_priority_metric_ ? quiet_jitter_ : normal_quiet_jitter_;
  this->last_score_ = 2.0f * jitter / std::max(quiet_reference, QUIET_FLOOR);

  if (this->quiet_samples_ < this->quiet_boot_samples_) {
    this->quiet_jitter_ += (jitter - this->quiet_jitter_) / static_cast<float>(this->quiet_samples_ + 1);
    this->quiet_jitter_ = std::max(this->quiet_jitter_, QUIET_FLOOR);
    this->normal_quiet_jitter_ = this->quiet_jitter_;
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
      selected_energy_, threshold_, motion_state_, joint_response_, above,
      quiet_alpha_, quiet_update_limit_);
  const bool confirmed = global_confirmed || priority_confirmed;

  // Equivalent in spirit to Espressif filter_window/filter_count: isolated CSI
  // outliers cannot trigger motion; several jitter excursions inside a short
  // window are required. Once confirmed, consume that window so a static RF
  // offset cannot repeatedly retrigger the 120 s hold.
  if (confirmed) {
    // Track what really refreshed the 120-second hold. The global path
    // may be using either its 48-bin fallback or the joint-gated 70/30 metric.
    // Last route is diagnostic-only; it does not affect motion decisions.
    last_confirmation_route_ = (global_confirmed && priority_confirmed) ? 4 :
        priority_confirmed ? 3 : (using_priority_metric_ ? 2 : 1);
    this->motion_state_ = true;
    this->last_motion_time_ = now_ms;
    this->clear_filter_();
    this->priority_.clear_filter();
  }

  // Learn only genuinely quiet temporal jitter. A permanent change to the room
  // produces a transient and then becomes quiet automatically; no room baseline
  // reset is required and there is no stale absolute score to retrigger later.
  if (!above && (!priority_.active() || priority_.score() < threshold_)) {
    const float normal_score = 2.0f * normal_jitter_ /
        std::max(normal_quiet_jitter_, QUIET_FLOOR);
    if (normal_score < quiet_update_limit_) {
      normal_quiet_jitter_ += quiet_alpha_ * (normal_jitter_ - normal_quiet_jitter_);
      normal_quiet_jitter_ = std::max(normal_quiet_jitter_, QUIET_FLOOR);
    }
    if (priority_.active()) {
      const float weighted_score = 2.0f * priority_jitter_ /
          std::max(quiet_jitter_, QUIET_FLOOR);
      if (weighted_score < quiet_update_limit_) {
        quiet_jitter_ += quiet_alpha_ * (priority_jitter_ - quiet_jitter_);
        quiet_jitter_ = std::max(quiet_jitter_, QUIET_FLOOR);
      }
    } else {
      quiet_jitter_ = normal_quiet_jitter_;
    }
  }

  const bool revised = learning_.observe(raw_energy_, now_ms, confirmed, motion_state_);
  // Refresh the priority pair on new eligible episodes, independently of
  // guarded weight revisions. An invalid pair is replaced immediately if two
  // qualified bins exist; a merely better pair requires stable evidence.
  const bool selection_changed = priority_.refresh(learning_, revised);
  if (revised || selection_changed) {
    // A change of bins OR weights changes the effective score scale. Reset
    // MVS3's own profiles/filters; preserve the current 120-second ON hold.
    normal_quiet_jitter_ = learning_.quiet_jitter();
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
    normal_jitter_ = priority_jitter_ = 0.0f;
    selected_energy_ = 0.0f;
    joint_response_ = using_priority_metric_ = false;
    return 0.0f;
  }

  for (size_t i = 0; i < kBins; i++) {
    const float delta = profile[i] - this->previous_profile_[i];
    raw_energy_[i] = delta * delta;
    this->previous_profile_[i] = profile[i];
  }
  // Apply the 70/30 global weighting whenever a qualified Top 2 exists.
  // The simultaneous-response gate remains exclusively on the independent
  // Top 2 confirmation path; it does not restrict the global 70/30 metric.
  joint_response_ = priority_.joint_response(raw_energy_, learning_, threshold_);
  using_priority_metric_ = priority_.active();
  ++scored_observations_;
  if (using_priority_metric_) ++priority_metric_observations_;
  const float normal_energy = priority_.combined_energy(
      raw_energy_, learning_.weights(), nullptr, false);
  const float priority_energy = priority_.combined_energy(
      raw_energy_, learning_.weights(), &selected_energy_, true);
  normal_jitter_ = std::sqrt(normal_energy + JITTER_EPSILON);
  priority_jitter_ = std::sqrt(priority_energy + JITTER_EPSILON);
  return using_priority_metric_ ? priority_jitter_ : normal_jitter_;
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

