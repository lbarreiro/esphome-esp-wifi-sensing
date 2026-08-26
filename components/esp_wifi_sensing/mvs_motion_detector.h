#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "csi_packet.h"
#include "mvs_ml_weights.h"

namespace esphome {
namespace esp_wifi_sensing {

struct MvsMotionResult {
  bool active{false}; float variance{0.0f}; float threshold{5.0f}; float baseline{0.0f};
  float change_rate{0.0f}; float spatial_change{0.0f}; float coherence{1.0f}; float feature_score{0.0f};
};

class MvsMotionDetector {
 public:
  void set_window_samples(uint16_t) {}
  void set_threshold_multiplier(float) {}
  void set_threshold(float value) { threshold_ = value; }
  void set_enter_hits(uint8_t value) { enter_hits_ = value; }
  void set_exit_hits(uint8_t value) { exit_hits_ = value; }
  void set_hold_time_ms(uint32_t value) { hold_time_ms_ = std::max<uint32_t>(120000U, value); }

  MvsMotionResult update(const CsiPacket &packet, uint32_t now_ms) {
    const float turbulence = calculate_turbulence_(packet);
    if (!std::isfinite(turbulence)) return result_();
    const float filtered = hampel_(turbulence);
    turbulence_[write_index_] = filtered;
    write_index_ = (write_index_ + 1) % kWindow;
    if (count_ < kWindow) ++count_;
    ++packet_counter_;
    latest_turbulence_ = filtered;
    if (count_ < kWindow || (packet_counter_ % kEvaluationInterval) != 0) return result_();

    float ordered[kWindow];
    for (uint16_t i = 0; i < kWindow; ++i) ordered[i] = turbulence_[(write_index_ + i) % kWindow];
    float features[9]; extract_features_(ordered, features);
    const float score = predict_(features) * 10.0f;
    const float previous_score = feature_score_;
    feature_score_ = score;
    change_rate_ = score - previous_score;
    variance_ = features[1] * features[1];
    spatial_change_ = features[8]; coherence_ = features[6]; baseline_ = features[0];

    const bool positive = score > threshold_;
    if (positive) {
      if (enter_hits_count_ < enter_hits_) ++enter_hits_count_;
      exit_hits_count_ = 0;
      if (active_) hold_until_ms_ = now_ms + hold_time_ms_;
    } else {
      if (exit_hits_count_ < exit_hits_) ++exit_hits_count_;
      enter_hits_count_ = 0;
    }
    if (!active_ && enter_hits_count_ >= enter_hits_) {
      active_ = true; hold_until_ms_ = now_ms + hold_time_ms_;
      enter_hits_count_ = 0; exit_hits_count_ = 0;
    }
    if (active_ && static_cast<int32_t>(now_ms - hold_until_ms_) >= 0 && exit_hits_count_ >= exit_hits_) {
      active_ = false; exit_hits_count_ = 0;
    }
    return result_();
  }

 private:
  static constexpr uint16_t kWindow = 100, kEvaluationInterval = 25;
  static constexpr uint8_t kHampelWindow = 7;
  static constexpr float kHampelThreshold = 5.0f, kHampelScale = 1.4826f, kMlTemperature = 5.0f;
  static constexpr uint8_t kSubcarriers[12] = {12,14,16,18,20,24,28,36,40,44,48,52};

  float calculate_turbulence_(const CsiPacket &packet) const {
    if (packet.raw_bytes == nullptr || packet.len < 2 * 53) return NAN;
    float amplitudes[12], sum = 0.0f;
    for (uint8_t i = 0; i < 12; ++i) {
      const uint16_t sc = kSubcarriers[i];
      const int8_t in_phase = static_cast<int8_t>(packet.raw_bytes[2 * sc]);
      const int8_t quadrature = static_cast<int8_t>(packet.raw_bytes[2 * sc + 1]);
      const float amp = std::sqrt(static_cast<float>(in_phase) * in_phase + static_cast<float>(quadrature) * quadrature);
      amplitudes[i] = amp; sum += amp;
    }
    const float mean = sum / 12.0f;
    float variance = 0.0f;
    for (float amp : amplitudes) { const float d = amp - mean; variance += d * d; }
    return std::sqrt(variance / 12.0f);
  }

  float hampel_(float value) {
    hampel_buffer_[hampel_index_] = value; hampel_index_ = (hampel_index_ + 1) % kHampelWindow;
    if (hampel_count_ < kHampelWindow) ++hampel_count_; if (hampel_count_ < 3) return value;
    float sorted[kHampelWindow]; for (uint8_t i=0;i<hampel_count_;++i) sorted[i]=hampel_buffer_[i];
    std::sort(sorted, sorted+hampel_count_); const float median=sorted[hampel_count_/2];
    for (uint8_t i=0;i<hampel_count_;++i) sorted[i]=std::fabs(hampel_buffer_[i]-median);
    std::sort(sorted, sorted+hampel_count_); const float mad=sorted[hampel_count_/2];
    return (mad > 1e-6f && std::fabs(value-median) > kHampelThreshold*kHampelScale*mad) ? median : value;
  }

  static float median_(float *v, uint16_t n) { std::sort(v,v+n); return (n&1)?v[n/2]:(v[n/2-1]+v[n/2])*0.5f; }
  static float percentile_(const float *s,uint16_t n,float p) { const float pos=(n-1)*p; const uint16_t lo=(uint16_t)pos, hi=lo+1<n?lo+1:lo; return s[lo]+(s[hi]-s[lo])*(pos-lo); }

  void extract_features_(const float *x,float *f) const {
    float mean=0,min_v=x[0],max_v=x[0]; for(uint16_t i=0;i<kWindow;++i){mean+=x[i];min_v=std::min(min_v,x[i]);max_v=std::max(max_v,x[i]);} mean/=kWindow;
    float var=0; for(uint16_t i=0;i<kWindow;++i){float d=x[i]-mean;var+=d*d;} var/=kWindow; const float sd=std::sqrt(var);
    float sorted[kWindow]; std::memcpy(sorted,x,sizeof(sorted)); std::sort(sorted,sorted+kWindow);
    const float q25=percentile_(sorted,kWindow,.25f),q75=percentile_(sorted,kWindow,.75f),med=percentile_(sorted,kWindow,.5f);
    float dev[kWindow]; for(uint16_t i=0;i<kWindow;++i)dev[i]=std::fabs(x[i]-med); const float mad=median_(dev,kWindow);
    float m3=0; if(sd>1e-10f){for(uint16_t i=0;i<kWindow;++i){float d=x[i]-mean;m3+=d*d*d;}m3/=kWindow;m3/=(sd*sd*sd);}
    float ac=0; for(uint16_t i=0;i<kWindow-1;++i)ac+=(x[i]-mean)*(x[i+1]-mean); ac/=(kWindow-1); const float ar=var>1e-10f?ac/var:0;
    float waveform=0; for(uint16_t i=1;i<kWindow;++i)waveform+=std::fabs(x[i]-x[i-1]);
    f[0]=mean;f[1]=sd;f[2]=max_v;f[3]=min_v;f[4]=q75-q25;f[5]=m3;f[6]=ar;f[7]=mad;f[8]=waveform;
  }

  float predict_(const float *features) const {
    float h1[32]{},h2[16]{},x[9]; for(uint8_t i=0;i<9;++i)x[i]=(features[i]-mvs_ml::FEATURE_MEAN[i])/mvs_ml::FEATURE_SCALE[i];
    for(uint8_t j=0;j<32;++j){float v=mvs_ml::B1[j];for(uint8_t i=0;i<9;++i)v+=x[i]*mvs_ml::W1[i*32+j];h1[j]=v>0?v:0;}
    for(uint8_t j=0;j<16;++j){float v=mvs_ml::B2[j];for(uint8_t i=0;i<32;++i)v+=h1[i]*mvs_ml::W2[i*16+j];h2[j]=v>0?v:0;}
    float z=mvs_ml::B3[0];for(uint8_t i=0;i<16;++i)z+=h2[i]*mvs_ml::W3[i]; z/=kMlTemperature; z=std::fmax(-20.0f,std::fmin(20.0f,z));
    return 1.0f/(1.0f+std::exp(-z));
  }

  MvsMotionResult result_() const { MvsMotionResult r;r.active=active_;r.variance=variance_;r.threshold=threshold_;r.baseline=baseline_;r.change_rate=change_rate_;r.spatial_change=spatial_change_;r.coherence=coherence_;r.feature_score=feature_score_;return r; }
  float turbulence_[kWindow]{},hampel_buffer_[kHampelWindow]{};
  uint16_t write_index_{0},count_{0}; uint32_t packet_counter_{0},hold_until_ms_{0}; uint8_t hampel_index_{0},hampel_count_{0};
  uint8_t enter_hits_{3},exit_hits_{3},enter_hits_count_{0},exit_hits_count_{0}; uint32_t hold_time_ms_{120000}; bool active_{false};
  float latest_turbulence_{0},variance_{0},threshold_{5},baseline_{0},change_rate_{0},spatial_change_{0},coherence_{1},feature_score_{0};
};
constexpr uint8_t MvsMotionDetector::kSubcarriers[12];
}
}
