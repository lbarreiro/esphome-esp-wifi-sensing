#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "csi_learning.h"
#include "csi_timestamp.h"
#include "experimental_mvs_algorithm.h"

using namespace esphome::esp_wifi_sensing;

static void step(CsiLearning &l, uint32_t &now, float value, bool confirm=false, bool held=false) {
  float e[48];
  for (size_t i=0;i<48;++i) e[i] = i<24 ? value : 0.000001f;
  now += 100;
  l.observe(e, now, confirm, held);
}
static void all_step(CsiLearning &l, uint32_t &now, float value, bool confirm=false, bool held=false) {
  float e[48];
  for (size_t i=0;i<48;++i) e[i] = value;
  now += 100;
  l.observe(e, now, confirm, held);
}
static void sparse_step(CsiLearning &l, uint32_t &now, float value, bool confirm=false, bool held=false) {
  float e[48];
  for (size_t i=0;i<48;++i) e[i] = i<6 ? value : 0.000001f;
  now += 100;
  l.observe(e, now, confirm, held);
}
static void varied_step(CsiLearning &l, uint32_t &now, unsigned ep,
                        bool moving, bool confirm=false) {
  float e[48];
  for (size_t i=0;i<48;++i) e[i]=0.000001f;
  if (moving) {
    for (size_t i=0;i<6;++i) e[i]=0.02f;
    if (ep<3) e[6]=0.02f;  // three independent episode hits
    if (ep<2) e[7]=0.02f;  // two hits must not qualify
    e[8]=0.02f;           // strong contrast on all eight episodes
    e[9]=0.004f;          // weaker contrast at the same recurrence
  }
  now+=100;
  l.observe(e,now,confirm,moving);
}
static void varied_episode(CsiLearning &l, uint32_t &now, unsigned ep) {
  for (unsigned i=0;i<15;++i) varied_step(l,now,ep,true,i==5);
  for (unsigned i=0;i<25;++i) varied_step(l,now,ep,false);
}
static void quiet(CsiLearning &l, uint32_t &now, unsigned samples=400) {
  for(unsigned i=0;i<samples;++i) step(l,now,0.000001f);
}
static void episode(CsiLearning &l,uint32_t &now,bool feedback=true) {
  for(unsigned i=0;i<15;++i) step(l,now,0.02f,feedback && i==5,true);
  for(unsigned i=0;i<25;++i) step(l,now,0.000001f,false,true);
}
int main() {
  CsiLearning l; uint32_t now=0;
  quiet(l,now,3200);
  assert(l.ready() && l.episodes()==0 && l.revisions()==0);
  episode(l,now,false); assert(l.episodes()==0 && l.rejected()==1);
  step(l,now,0.02f,true,true); quiet(l,now,30);
  assert(l.episodes()==0); // isolated spike cannot teach a movement episode
  episode(l,now); episode(l,now);
  quiet(l,now); assert(l.revisions()==0);
  episode(l,now);
  assert(l.episodes()==3 && l.revisions()==0);
  for(unsigned i=0;i<400;++i) step(l,now,0.000001f,false,true);
  assert(l.revisions()==0); // never revise during the held ON period
  quiet(l,now);
  assert(l.revisions()==0); // no promotion until eight confirmed episodes
  for (unsigned i=0;i<5;++i) episode(l,now);
  assert(l.episode_window_count()==8 && l.revisions()==0);
  quiet(l,now);
  assert(l.revisions()==1);
  float saved[48];
  for(size_t i=0;i<48;++i) {
    saved[i]=l.weights()[i];
    assert(saved[i]>=0.70f && saved[i]<=2.00f && std::fabs(saved[i]-1)<=0.10001f);
  }
  assert(saved[0]>saved[47]);
  assert(std::isfinite(l.quiet_jitter()) && l.quiet_jitter()>=0.01f);
  quiet(l,now,10000);
  assert(l.revisions()==1); // time / repeated quiet never reuses old evidence
  for(size_t i=0;i<48;++i) assert(saved[i]==l.weights()[i]);

  for(unsigned batch=0;batch<20;++batch) {
    episode(l,now); episode(l,now); episode(l,now); quiet(l,now,3100);
    for(size_t i=0;i<48;++i) assert(l.weights()[i]>=0.70f && l.weights()[i]<=2.00f);
  }
  const auto count=l.episodes();
  for(unsigned i=0;i<10;++i) step(l,now,0.02f,true,true);
  l.discontinuity(); quiet(l,now);
  assert(l.episodes()==count);
  float invalid[48]{}; invalid[0]=std::numeric_limits<float>::quiet_NaN();
  assert(!l.observe(invalid,now,false,false));

  // A response common to all bins must not inflate all weights together.
  CsiLearning uniform; uint32_t uniform_now=0;
  for (unsigned i=0;i<3200;++i) all_step(uniform,uniform_now,0.000001f);
  for (unsigned n=0;n<8;++n) {
    for (unsigned i=0;i<15;++i) all_step(uniform,uniform_now,0.02f,i==5,true);
    for (unsigned i=0;i<25;++i) all_step(uniform,uniform_now,0.000001f,false,true);
  }
  for (unsigned i=0;i<400;++i) all_step(uniform,uniform_now,0.000001f);
  assert(uniform.revisions()==1);
  // Uniform movement evidence gives uniform weighting, not a false ranking.
  // Weighted temporal jitter normalizes by the sum, so a common scalar does
  // not change the actual MVS-3 score.
  for (size_t i=0;i<48;++i) {
    assert(std::fabs(uniform.candidate(i)-uniform.candidate(0))<0.0001f);
    assert(std::fabs(uniform.weights()[i]-uniform.weights()[0])<0.0001f);
  }

  // Six consistently responsive bins can reach 2.00, while all 48 remain active.
  CsiLearning sparse; uint32_t sparse_now=0;
  for (unsigned i=0;i<3200;++i) sparse_step(sparse,sparse_now,0.000001f);
  for (unsigned n=0;n<8;++n) {
    for (unsigned i=0;i<15;++i) sparse_step(sparse,sparse_now,0.02f,i==5,true);
    for (unsigned i=0;i<25;++i) sparse_step(sparse,sparse_now,0.000001f,false,true);
  }
  for (unsigned i=0;i<400;++i) sparse_step(sparse,sparse_now,0.000001f);
  assert(sparse.revisions()==1);
  assert(sparse.candidate(0)>1.90f);
  assert(sparse.candidate(47)<1.0f);
  assert(std::fabs(sparse.weights()[0]-1.10f)<0.0001f);
  assert(std::fabs(sparse.weights()[47]-0.90f)<0.0001f);

  // Once movement has established the ranking, learning more silence must
  // not move candidates (or weights) when no further episode is confirmed.
  float ranked_candidates[48], ranked_weights[48];
  for (size_t i=0;i<48;++i) {
    ranked_candidates[i]=sparse.candidate(i);
    ranked_weights[i]=sparse.weights()[i];
  }
  const uint32_t initial_quiet=sparse.quiet_blocks();
  for (unsigned i=0;i<1600;++i) sparse_step(sparse,sparse_now,0.0002f);
  assert(sparse.quiet_blocks()>initial_quiet);
  assert(sparse.episodes()==8 && sparse.revisions()==1);
  for (size_t i=0;i<48;++i) {
    assert(sparse.candidate(i)==ranked_candidates[i]);
    assert(sparse.weights()[i]==ranked_weights[i]);
  }

  // One vote per confirmed episode, with three of the last eight required.
  // Recurrence AND motion/quiet contrast matter independently.
  CsiLearning varied; uint32_t varied_now=0;
  for (unsigned i=0;i<3200;++i) all_step(varied,varied_now,0.000001f);
  for (unsigned ep=0;ep<8;++ep) varied_episode(varied,varied_now,ep);
  assert(varied.episode_window_count()==8 && varied.episodes()==8);
  assert(varied.hits(6)==3 && varied.hits(7)==2);
  assert(varied.hits(8)==8 && varied.hits(9)==8);
  assert(varied.candidate(6)>1.0f);
  assert(std::fabs(varied.candidate(7)-1.0f)<0.0001f);
  assert(varied.candidate(8)>varied.candidate(9));
  assert(varied.candidate(8)>1.90f);
  assert(varied.candidate(47)==0.70f);
  quiet(varied,varied_now);
  assert(varied.revisions()==1);
  assert(varied.weights()[8]<=1.10001f && varied.weights()[8]>=1.09999f);
  // Rolling out former hits causes a previously useful bin to return
  // toward neutral, rather than being permanently rewarded or discarded.
  for (unsigned ep=8;ep<16;++ep) varied_episode(varied,varied_now,ep);
  assert(varied.hits(6)==0 && varied.hits(7)==0);
  assert(std::fabs(varied.candidate(6)-1.0f)<0.0001f);
  assert(std::fabs(varied.candidate(7)-0.70f)<0.0001f);
  assert(varied.hits(8)==8);

  // CSI callback timestamps can be slightly ahead of the ESPHome loop, but
  // genuine packet age and uint32 millis() rollover must still work.
  assert(csi_elapsed_or_zero(1000U, 1001U)==0U);
  assert(csi_elapsed_or_zero(1000U, 999U)==1U);
  assert(csi_elapsed_or_zero(6001U, 1000U)==5001U);
  assert(csi_elapsed_or_zero(10U, UINT32_MAX-9U)==20U);

  // A stationary but high-noise room must bootstrap without a fixed gate.
  CsiLearning noisy; uint32_t noisy_now=0;
  for (unsigned i=0; i<800; ++i) step(noisy,noisy_now,0.0005f);
  assert(noisy.ready() && noisy.quiet_blocks()>=6);
  // A single RF spike every four seconds must not erase the whole block.
  CsiLearning bursty; uint32_t burst_now=0;
  for (unsigned i=0; i<1200; ++i)
    step(bursty,burst_now,(i%40==39) ? 0.02f : 0.000001f);
  assert(bursty.ready() && bursty.quiet_blocks()>=6);
  // Sustained high-energy ambiguity cannot silently teach the baseline
  // after a quiet reference has already been established.
  const uint32_t before_bursts=bursty.quiet_blocks();
  for (unsigned i=0; i<200; ++i) step(bursty,burst_now,0.02f);
  assert(bursty.quiet_blocks()==before_bursts);

  // Reproduce noisy stationary CSI: 25-50% of observations fall below
  // the obsolete minimum-envelope threshold, with no motion/hold/gaps.
  CsiLearning realistic; uint32_t realistic_now=0;
  for (unsigned i=0; i<1200; ++i) {
    const float e = (i%11==0 ? 0.12f : (i%5==0 ? 0.055f :
        (i%3==0 ? 0.032f : 0.020f))) / 24.0f;
    step(realistic, realistic_now, e);
  }
  assert(realistic.ready() && realistic.quiet_blocks()>=6);
  assert(realistic.hold_resets()==0 && realistic.discontinuities()==0);

  // Unsigned elapsed time works across millis() rollover.
  CsiLearning wrap; uint32_t clock=UINT32_MAX-20000;
  quiet(wrap,clock,400); assert(wrap.ready());

  // Exercise actual C++ MVS-3 with neutral weights against the reference.
  MvsAlgorithm primary; ExperimentalMvsAlgorithm third;
  ParsedCsiPacket packet{}; packet.layout_supported=true; packet.count=48;
  for(uint32_t t=0;t<2000;++t) {
    for(size_t i=0;i<48;++i) packet.subcarriers[i].amplitude =
      std::expm1(2.0f + 0.05f*std::sin(float(i)) +
        ((t>=400 && t<450) ? 0.5f*std::sin(i*0.55f+t*0.37f) : 0.0f));
    auto p=primary.process(packet,t*100), e=third.process(packet,t*100);
    assert(p.motion==e.motion);
    if(t>30) assert(std::fabs(p.score-e.score)<0.001f);
  }
  assert(third.learning().episodes()==1);
  assert(third.learning().revisions()==0);

  ExperimentalMvsAlgorithm adaptive;
  bool revised=false;
  for(uint32_t t=0;t<12400;++t) {
    const bool moving=(t%1600 >= 400 && t%1600 < 450);
    for(size_t i=0;i<48;++i) packet.subcarriers[i].amplitude =
      std::expm1(2.0f + 0.05f*std::sin(float(i)) +
        (moving ? 0.5f*std::sin(i*0.55f+t*0.37f) : 0.0f));
    auto e=adaptive.process(packet,t*100);
    if(adaptive.learning().revisions()>0) {
      revised=true;
      assert(!e.motion); // committing/reseeding weights never creates motion
      assert(std::isfinite(e.score));
    }
  }
  std::fprintf(stderr, "adaptive: episodes=%u revisions=%u quiet=%u rejected=%u revised=%d\\n",
               adaptive.learning().episodes(), adaptive.learning().revisions(),
               adaptive.learning().quiet_blocks(), adaptive.learning().rejected(), revised);
  assert(revised && adaptive.learning().episodes()==8);
  assert(adaptive.learning().revisions()==1);
  std::puts("C++ learning/weights/episodes/gaps/rollover and neutral MVS equivalence: PASS");
}
