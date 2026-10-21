#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "csi_learning.h"
#include "experimental_mvs_algorithm.h"

using namespace esphome::esp_wifi_sensing;

static void step(CsiLearning &l, uint32_t &now, float value, bool confirm=false, bool held=false) {
  float e[48];
  for (size_t i=0;i<48;++i) e[i] = i<24 ? value : 0.000001f;
  now += 100;
  l.observe(e, now, confirm, held);
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
  assert(l.revisions()==1);
  float saved[48];
  for(size_t i=0;i<48;++i) {
    saved[i]=l.weights()[i];
    assert(saved[i]>=0.75f && saved[i]<=1.50f && std::fabs(saved[i]-1)<=0.05001f);
  }
  assert(saved[0]>saved[47]);
  assert(std::isfinite(l.quiet_jitter()) && l.quiet_jitter()>=0.01f);
  quiet(l,now,10000);
  assert(l.revisions()==1); // time / repeated quiet never reuses old evidence
  for(size_t i=0;i<48;++i) assert(saved[i]==l.weights()[i]);

  for(unsigned batch=0;batch<20;++batch) {
    episode(l,now); episode(l,now); episode(l,now); quiet(l,now,3100);
    for(size_t i=0;i<48;++i) assert(l.weights()[i]>=0.75f && l.weights()[i]<=1.50f);
  }
  const auto count=l.episodes();
  for(unsigned i=0;i<10;++i) step(l,now,0.02f,true,true);
  l.discontinuity(); quiet(l,now);
  assert(l.episodes()==count);
  float invalid[48]{}; invalid[0]=std::numeric_limits<float>::quiet_NaN();
  assert(!l.observe(invalid,now,false,false));

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
  for(uint32_t t=0;t<6000;++t) {
    const bool moving=(t>=400 && t<450) || (t>=2000 && t<2050) || (t>=3600 && t<3650);
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
  assert(revised && adaptive.learning().episodes()==3);
  assert(adaptive.learning().revisions()==1);
  std::puts("C++ learning/weights/episodes/gaps/rollover and neutral MVS equivalence: PASS");
}
