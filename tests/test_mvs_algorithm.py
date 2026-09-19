import math
import random
from pathlib import Path

BINS=48; WINDOW=8; COUNT=3; HOLD=1200

class RadarModel:
    """Executable behavioural model of the C++ jitter/filter/hold pipeline at 10 Hz."""
    def __init__(self, threshold=9.5):
        self.threshold=threshold; self.prev=None; self.quiet=.02; self.qn=0
        self.window=[0]*WINDOW; self.pos=0; self.hits=0
        self.active=False; self.last=-10**9
    def step(self, frame, tick):
        mean=sum(frame)/BINS
        profile=[x-mean for x in frame]
        if self.prev is None:
            self.prev=profile; return False,0.0
        jitter=(sum((a-b)**2 for a,b in zip(profile,self.prev))/BINS+1e-4)**.5
        self.prev=profile
        score=2.0*jitter/max(self.quiet,.003)
        if self.qn<20:
            self.quiet+=(jitter-self.quiet)/(self.qn+1); self.quiet=max(self.quiet,.003); self.qn+=1
            return False,score
        above=score>=self.threshold
        self.hits-=self.window[self.pos]; self.window[self.pos]=int(above); self.hits+=int(above); self.pos=(self.pos+1)%WINDOW
        if self.hits>=COUNT:
            self.active=True; self.last=tick; self.window=[0]*WINDOW; self.hits=0; self.pos=0
        if not above and score<3.0:
            self.quiet+=.01*(jitter-self.quiet); self.quiet=max(self.quiet,.003)
        if self.active and tick-self.last>=HOLD:
            self.active=False; self.window=[0]*WINDOW; self.hits=0; self.pos=0
        return self.active,score

BASE=[2.0+math.sin(i*.31)*.08 for i in range(BINS)]
def frame(t, motion=0.0, static=0.0, common=0.0, noise=.002, drift=0.0):
    rnd=random.Random(t)
    return [BASE[i]+common+drift+static*math.sin(i*.43)+
            motion*(math.sin(i*.55+t*.37)+.3*math.sin(i*1.9+t*.21))+
            rnd.uniform(-noise,noise) for i in range(BINS)]

def run(n=5000, fn=lambda t:{}):
    d=RadarModel(); states=[]; scores=[]
    for t in range(n):
        s,score=d.step(frame(t,**fn(t)),t); states.append(s); scores.append(score)
    return states,scores

def transitions(states):
    return [i for i in range(1,len(states)) if states[i]!=states[i-1]]

def test_01_empty_room_long_term(): assert not any(run(20000)[0])
def test_02_normal_rf_noise(): assert not any(run(5000,lambda t:{"noise":.006})[0])
def test_03_person_enters_and_moves():
    s,_=run(2000,lambda t:{"motion":.25 if 300<=t<380 else 0}); assert 302 in transitions(s)
def test_04_short_motion_then_quiet_holds_120s():
    s,_=run(2000,lambda t:{"motion":.25 if 300<=t<380 else 0}); tr=transitions(s); assert tr[0]<=308 and tr[1]>=1580
def test_05_continuous_motion_extends_hold():
    s,_=run(4000,lambda t:{"motion":.25 if 300<=t<2500 else 0}); tr=transitions(s); assert tr[0]<=308 and tr[1]>=3690
def test_06_person_stops_moving_does_not_latch_forever():
    s,_=run(2500,lambda t:{"motion":.25 if 300<=t<380 else 0,"static":.15 if t>=380 else 0}); assert not s[-1]
def test_07_permanent_rf_scene_change_does_not_trigger():
    s,_=run(3000,lambda t:{"static":.25 if t>=300 else 0}); assert not any(s)
def test_08_new_motion_after_off_reactivates():
    s,_=run(3500,lambda t:{"motion":.25 if 300<=t<380 or 1800<=t<1880 else 0}); tr=transitions(s); assert len(tr)>=4 and tr[2]<=1808
def test_09_multiple_separate_events():
    s,_=run(6000,lambda t:{"motion":.25 if any(a<=t<a+80 for a in (300,1800,3300)) else 0}); assert len(transitions(s))==6
def test_10_many_hours_without_motion(): assert not any(run(8*60*60*10)[0])
def test_11_slow_channel_drift():
    s,_=run(10000,lambda t:{"drift":t*0.00001}); assert not any(s)
def test_12_common_mode_rf_disturbance():
    s,_=run(3000,lambda t:{"common":.4 if 300<=t<1000 else 0}); assert not any(s)
def test_13_real_event_after_previous_hold():
    s,_=run(3000,lambda t:{"motion":.25 if 300<=t<380 or 1590<=t<1670 else 0}); tr=transitions(s); assert len(tr)>=3 and tr[2]<=1598

def test_no_off_3s_on_loop_without_new_motion():
    s,_=run(8*60*60*10,lambda t:{"motion":.25 if 300<=t<380 else 0})
    assert len(transitions(s))==2

def test_cpp_uses_temporal_jitter_and_filter_not_absolute_baseline():
    src=Path("components/esp_wifi_sensing/mvs_algorithm.cpp").read_text()
    assert "temporal_jitter_" in src and "update_filter_" in src
    assert "baseline_" not in src and "score_window_" not in src

def test_only_public_motion_binary_sensor():
    assert not Path("components/esp_wifi_sensing/sensor.py").exists()
