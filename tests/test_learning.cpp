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
    if (ep<5) e[6]=0.02f;  // five independent episode hits
    if (ep<4) e[7]=0.02f;  // four hits must not qualify
    e[8]=0.02f;           // strong contrast on all eight episodes
    e[9]=0.004f;          // weaker contrast at the same recurrence
  }
  now+=100;
  l.observe(e,now,confirm,moving);
}
static void varied_episode(CsiLearning &l, uint32_t &now, unsigned ep) {
  for (unsigned i=0;i<15;++i) varied_step(l,now,ep,true,i==5);
  for (unsigned i=0;i<65;++i) varied_step(l,now,ep,false);
}
static void quiet(CsiLearning &l, uint32_t &now, unsigned samples=400) {
  for(unsigned i=0;i<samples;++i) step(l,now,0.000001f);
}
static void episode(CsiLearning &l,uint32_t &now,bool feedback=true) {
  for(unsigned i=0;i<15;++i) step(l,now,0.02f,feedback && i==5,true);
  for(unsigned i=0;i<65;++i) step(l,now,0.000001f,false,true);
}
int main() {
  CsiLearning l; uint32_t now=0;
  quiet(l,now,3200);
  assert(l.ready() && l.episodes()==0 && l.revisions()==0);
  episode(l,now,false); assert(l.episodes()==0 && l.rejected()==1);
  step(l,now,0.02f,true,true); quiet(l,now,30);
  assert(l.episodes()==0); // isolated spike cannot teach a movement episode
  quiet(l,now,100);  // build a fresh pre-motion baseline after that candidate
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
    assert(saved[i]>=0.70f && saved[i]<=2.00f);
    // Apply precisely 90% of each learnt delta on the first revision.
    const float expected=1.0f+0.90f*(l.candidate(i)-1.0f);
    assert(std::fabs(saved[i]-expected)<0.0001f);
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
    for (unsigned i=0;i<65;++i) all_step(uniform,uniform_now,0.000001f,false,true);
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
    for (unsigned i=0;i<65;++i) sparse_step(sparse,sparse_now,0.000001f,false,true);
  }
  for (unsigned i=0;i<400;++i) sparse_step(sparse,sparse_now,0.000001f);
  assert(sparse.revisions()==1);
  assert(sparse.candidate(0)>1.90f);
  assert(sparse.candidate(47)<1.0f);
  assert(std::fabs(sparse.weights()[0]-(1.0f+0.90f*(sparse.candidate(0)-1.0f)))<0.0001f);
  assert(std::fabs(sparse.weights()[47]-0.73f)<0.0001f);

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

  // One vote per confirmed episode, with five of the last eight required.
  // Recurrence AND motion/quiet contrast matter independently.
  CsiLearning varied; uint32_t varied_now=0;
  for (unsigned i=0;i<3200;++i) all_step(varied,varied_now,0.000001f);
  for (unsigned ep=0;ep<8;++ep) varied_episode(varied,varied_now,ep);
  assert(varied.episode_window_count()==8 && varied.episodes()==8);
  assert(varied.eligible_episodes()==8 && varied.skipped_episodes()==0);
  assert(varied.hits(6)==5 && varied.hits(7)==4);
  assert(varied.hits(8)==8 && varied.hits(9)==8);
  assert(varied.candidate(6)>1.0f);
  assert(std::fabs(varied.candidate(7)-1.0f)<0.0001f);
  assert(varied.recurrent_bins()>=7);
  assert(varied.promoted_bins()>=7);
  assert(varied.candidate(8)>varied.candidate(9));
  assert(varied.candidate(8)>1.90f);
  assert(varied.candidate(47)==0.70f);
  quiet(varied,varied_now);
  assert(varied.revisions()==1);
  assert(std::fabs(varied.weights()[8]-(1.0f+0.90f*(varied.candidate(8)-1.0f)))<0.0001f);
  assert(varied.weights()[8]>1.8f);

  // Priority is never enabled with an incomplete or unqualified episode window.
  Mvs3Priority priority;
  assert(!priority.refresh(CsiLearning{}));
  assert(!priority.active());
  float energy[48]{}, unit_weights[48]{};
  for (size_t i=0;i<48;++i) unit_weights[i]=1.0f;
  assert(std::fabs(priority.combined_energy(energy,unit_weights))<0.00001f);
  assert(priority.refresh(varied));
  assert(priority.active());
  bool selected[48]{};
  for (size_t j=0;j<2;++j) {
    const auto b=priority.top_bin(j);
    assert(b<48 && !selected[b]);
    selected[b]=true;
    assert(varied.hits(b)>=5 && varied.candidate(b)>1.0f);
  }
  // A top-two-only disturbance contributes 70% of the total energy,
  // regardless of how low the remaining 46 bins are.
  for (size_t i=0;i<48;++i) energy[i]=selected[i] ? 1.0f : 0.0f;
  float top_energy=0.0f;
  assert(std::fabs(priority.combined_energy(energy,unit_weights,&top_energy)-0.70f)<0.0001f);
  assert(std::fabs(top_energy-1.0f)<0.0001f);
  for (size_t i=0;i<48;++i) energy[i]=selected[i] ? 0.0f : 1.0f;
  assert(std::fabs(priority.combined_energy(energy,unit_weights,&top_energy)-0.30f)<0.0001f);
  assert(std::fabs(top_energy)<0.0001f);
  for (size_t i=0;i<48;++i) energy[i]=1.0f;
  assert(std::fabs(priority.combined_energy(energy,unit_weights)-1.0f)<0.0001f);

  // Explicit weighted 70/30 split: internal learned weights change the
  // average within each group, never the group's 70% / 30% budget.
  float unequal_weights[48];
  for (size_t i=0;i<48;++i) unequal_weights[i]=selected[i] ? 1.9f : 0.72f;
  for (size_t i=0;i<48;++i) energy[i]=selected[i] ? 1.0f : 0.0f;
  assert(std::fabs(priority.combined_energy(energy,unequal_weights)-0.70f)<0.0001f);
  for (size_t i=0;i<48;++i) energy[i]=selected[i] ? 0.0f : 1.0f;
  assert(std::fabs(priority.combined_energy(energy,unequal_weights)-0.30f)<0.0001f);

  // The two must individually respond within the SAME observation.
  // One extremely energetic but noisy bin is insufficient for either
  // independent confirmation OR the global 70/30 path.
  float individual[48]{};
  for (size_t i=0;i<48;++i) individual[i]=0.000001f;
  const auto first_bin=priority.top_bin(0);
  const auto second_bin=priority.top_bin(1);
  individual[first_bin]=0.06f;
  assert(!priority.joint_response(individual,varied,8.1f));
  assert(std::fabs(priority.combined_energy(individual,unit_weights,nullptr,false)
                   -priority.combined_energy(individual,unit_weights,nullptr,true))>0.001f);
  assert(std::fabs(priority.combined_energy(individual,unit_weights,nullptr,false)
                   -(0.06f+47.0f*0.000001f)/48.0f)<0.00001f);
  individual[second_bin]=0.06f;
  assert(priority.joint_response(individual,varied,8.1f));
  assert(priority.combined_energy(individual,unit_weights,nullptr,true)>
         priority.combined_energy(individual,unit_weights,nullptr,false));

  // The two bins have their OWN 3-of-8 confirmation; a single spike is
  // not enough, and their ON confirmation does not need the 46 others.
  // Even repeated strong energy cannot pass if only one bin responded.
  priority.clear_filter();
  for (unsigned i=0;i<12;++i)
    assert(!priority.observe(0.06f,8.1f,false,false,false,0.01f,3.0f));
  assert(priority.confirmations()==0);
  priority.clear_filter();
  assert(!priority.observe(0.03f,8.1f,false,true,false,0.01f,3.0f));
  priority.clear_filter(); // a lone spike cannot cross into the next episode
  assert(!priority.observe(0.0f,8.1f,false,true,false,0.01f,3.0f));
  assert(!priority.observe(0.03f,8.1f,false,true,false,0.01f,3.0f));
  assert(!priority.observe(0.03f,8.1f,false,true,false,0.01f,3.0f));
  assert(priority.observe(0.03f,8.1f,false,true,false,0.01f,3.0f));
  assert(priority.confirmations()==1);
  // After ON, the existing two-peak renewal rule is preserved.
  assert(!priority.observe(0.03f,8.1f,true,true,false,0.01f,3.0f));
  assert(priority.observe(0.03f,8.1f,true,true,false,0.01f,3.0f));
  assert(priority.confirmations()==2);
  // Rolling out former hits causes a previously useful bin to return
  // toward neutral, rather than being permanently rewarded or discarded.
  for (unsigned ep=8;ep<16;++ep) varied_episode(varied,varied_now,ep);
  assert(varied.hits(6)==0 && varied.hits(7)==0);
  assert(std::fabs(varied.candidate(6)-1.0f)<0.0001f);
  assert(std::fabs(varied.candidate(7)-0.70f)<0.0001f);
  assert(varied.hits(8)==8);

  // The raw episode gate still requires six responding bins to learn.
  // After eight episodes, two consistent bins can become priority bins
  // even if the other four vary on every episode; one is NOT sufficient.
  auto run_exact_group = [](unsigned fixed) {
    CsiLearning learn;
    uint32_t clock=0;
    float energy[48]{};
    for(unsigned k=0;k<3200;++k) {
      for(size_t i=0;i<48;++i) energy[i]=0.000001f;
      clock+=100;
      learn.observe(energy,clock,false,false);
    }
    for(unsigned ep=0;ep<8;++ep) {
      for(unsigned t=0;t<15;++t) {
        for(size_t i=0;i<48;++i) energy[i]=0.000001f;
        for(size_t i=0;i<fixed;++i) energy[i]=0.02f;
        for(size_t j=0;j<6-fixed;++j)
          energy[fixed+ep*(6-fixed)+j]=0.02f;
        clock+=100;
        learn.observe(energy,clock,t==5,true);
      }
      for(unsigned t=0;t<65;++t) {
        for(size_t i=0;i<48;++i) energy[i]=0.000001f;
        clock+=100;
        learn.observe(energy,clock,false,false);
      }
    }
    assert(learn.eligible_episodes()==8 && learn.episode_window_count()==8);
    Mvs3Priority path;
    assert(path.refresh(learn)==(fixed>=2));
    assert(path.active()==(fixed>=2));
    if(fixed>=2) {
      for(size_t i=0;i<2;++i) {
        assert(path.top_bin(i)<fixed);
        assert(learn.hits(path.top_bin(i))==8);
      }
      // Next eight eligible episodes respond only on a NEW pair.
      // The old pair becomes invalid at four old votes; the new pair
      // becomes qualified at five votes. Switching happens without a
      // guarded revision, with a brief 48-bin fallback while <2 qualify.
      bool suspended=false, reactivated=false;
      const uint32_t revision_at_selection=learn.revisions();
      for(unsigned ep=0;ep<8;++ep) {
        for(unsigned t=0;t<15;++t) {
          for(size_t i=0;i<48;++i) energy[i]=0.000001f;
          for(size_t i=12;i<18;++i) energy[i]=0.02f;
          clock+=100;
          learn.observe(energy,clock,t==5,true);
          const bool changed=path.refresh(learn);
          if(changed && !path.active()) suspended=true;
          if(changed && path.active() && path.top_bin(0)>=12) reactivated=true;
        }
        for(unsigned t=0;t<65;++t) {
          for(size_t i=0;i<48;++i) energy[i]=0.000001f;
          clock+=100;
          learn.observe(energy,clock,false,false);
          const bool changed=path.refresh(learn);
          if(changed && !path.active()) suspended=true;
          if(changed && path.active() && path.top_bin(0)>=12) reactivated=true;
        }
        if(ep==3) {
          assert(suspended && !path.active()); // no pair has 5/8
          assert(path.top_bin(0)==0 && path.top_bin(1)==0);
        }
        if(ep==4) {
          assert(reactivated && path.active()); // replacements now have 5/8
          assert(path.top_bin(0)>=12 && path.top_bin(1)>=12);
        }
      }
      assert(suspended && reactivated && path.active());
      assert(learn.revisions()==revision_at_selection);
      assert(learn.recurrent_bins()>=2);
      assert(!path.refresh(learn)); // silence cannot churn the pair

    }
  };
  run_exact_group(1);
  run_exact_group(2);

  // An RF-noisy bin and an isolated spike MUST NOT be credited as motion.
  // Both are observed alongside genuinely responsive bins in eight episodes.
  CsiLearning noisy_hits; uint32_t noisy_hits_now=0;
  for (unsigned i=0;i<3200;++i) all_step(noisy_hits,noisy_hits_now,0.000001f);
  auto noisy_step = [&](bool moving, unsigned j, bool confirm) {
    float e[48]{};
    for (size_t k=0;k<48;++k) e[k]=0.000001f;
    e[10]=0.001f; // always noisy, already elevated before the movement
    if (moving) {
      for (size_t k=0;k<6;++k) e[k]=0.02f;
      e[11]=j==0 ? 0.02f : 0.000001f; // one spike, never sustained
      e[12]=0.005f; // valid modest contrast, repeated on every episode
    }
    noisy_hits_now+=100;
    noisy_hits.observe(e,noisy_hits_now,confirm,moving);
  };
  for (unsigned n=0;n<25;++n) noisy_step(false,n,false);
  for (unsigned ep=0;ep<8;++ep) {
    for (unsigned j=0;j<15;++j) noisy_step(true,j,j==5);
    for (unsigned j=0;j<65;++j) noisy_step(false,j,false);
  }
  assert(noisy_hits.eligible_episodes()==8);
  assert(noisy_hits.hits(10)==0); // noise was not a quiet -> movement transition
  assert(noisy_hits.hits(11)==0); // lone spike was rejected
  assert(noisy_hits.hits(12)==8 && noisy_hits.candidate(12)>1.0f);
  assert(noisy_hits.last_transition_bins()>=7);

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
  // Diagnostic counters distinguish the selected Top 2 from the number of
  // observations where 70/30 was actually applied.
  assert(third.scored_observations()>0);
  assert(third.priority_metric_observations()==0);
  assert(third.last_confirmation_route()==1); // no Top2: global 48-bin path

  // Isolation regression: run the reference MVS-256 both alone and beside
  // a training MVS-3. Its score, ON/OFF and timing must remain IDENTICAL,
  // including after MVS-3 revises its internal weights.
  MvsAlgorithm reference_alone, reference_beside;
  ExperimentalMvsAlgorithm adaptive, learned_alone;
  bool revised=false;
  for(uint32_t t=0;t<13200;++t) {
    const bool moving=(t%1600 >= 400 && t%1600 < 450);
    for(size_t i=0;i<48;++i) packet.subcarriers[i].amplitude =
      std::expm1(2.0f + 0.05f*std::sin(float(i)) +
        (moving ? 0.5f*std::sin(i*0.55f+t*0.37f) : 0.0f));
    const auto a=reference_alone.process(packet,t*100);
    const auto learned_control=learned_alone.process(packet,t*100);
    const auto e=adaptive.process(packet,t*100);
    const auto b=reference_beside.process(packet,t*100);
    // Also prove the reverse: running MVS1 beside MVS-3 cannot change
    // the learned branch's state, score or its 48 learned weights.
    assert(learned_control.ready==e.ready);
    assert(learned_control.motion==e.motion);
    assert(learned_control.score==e.score);
    assert(learned_control.observation_count==e.observation_count);
    assert(learned_alone.scored_observations()==adaptive.scored_observations());
    assert(learned_alone.priority_metric_observations()==adaptive.priority_metric_observations());
    assert(adaptive.priority_metric_observations()<=adaptive.scored_observations());
    assert(learned_alone.last_confirmation_route()==adaptive.last_confirmation_route());
    assert(learned_alone.learning().episodes()==adaptive.learning().episodes());
    assert(learned_alone.learning().revisions()==adaptive.learning().revisions());
    for(size_t i=0;i<CsiLearning::kBins;++i)
      assert(learned_alone.learning().weights()[i]==adaptive.learning().weights()[i]);
    assert(a.ready==b.ready);
    assert(a.motion==b.motion);
    assert(a.observation_added==b.observation_added);
    assert(a.observation_count==b.observation_count);
    assert(a.score==b.score);
    if(adaptive.learning().revisions()>0) {
      revised=true;
      assert(!e.motion); // committing/reseeding weights never creates motion
      assert(std::isfinite(e.score));
    }
  }
  assert(revised && adaptive.learning().episodes()==8);
  assert(adaptive.learning().revisions()==1);
  std::puts("C++ MVS3 dynamic top-2 priority, 70/30 gate, 90% weight revisions, rolling episodes and MVS1 isolation: PASS");
}
