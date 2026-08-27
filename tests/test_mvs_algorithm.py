import math

BINS = 48
BOOT = 20
WINDOW = 32
ENTER = 3
EXIT = 8
HOLD = 120_000

class MVS:
    def __init__(self, threshold=9.5):
        self.threshold=threshold; self.baseline=None; self.noise=[0.08]*BINS; self.samples=0
        self.hist=[]; self.motion=False; self.last=0; self.enter=0; self.exit=0
    def update_base(self, frame, alpha=None):
        if self.baseline is None:
            self.baseline=frame[:]; self.samples=1; return
        if self.samples < BOOT:
            for i,x in enumerate(frame):
                d=x-self.baseline[i]; self.baseline[i]+=d/(self.samples+1); self.noise[i]+=(abs(d)-self.noise[i])/(self.samples+1)
            self.samples+=1; return
        for i,x in enumerate(frame):
            d=x-self.baseline[i]; a=alpha if alpha is not None else 0.006
            self.baseline[i]+=a*d; self.noise[i]=max(0.02,self.noise[i]+a*(abs(d)-self.noise[i]))
    def step(self, frame, now):
        m=sum(frame)/BINS; frame=[x-m for x in frame]
        if self.samples < BOOT:
            self.update_base(frame); return 0,self.motion
        common=sum(frame[i]-self.baseline[i] for i in range(BINS))/BINS
        res=mad=rough=tot=0
        for i in range(BINS):
            n=max(self.noise[i],0.02); d=frame[i]-self.baseline[i]; nc=(d-common)/n
            res+=nc*nc; mad+=abs(nc); tot+=(d/n)*(d/n)
            if i: rough+=abs((d-(frame[i-1]-self.baseline[i-1]))/max((self.noise[i]+self.noise[i-1])/2,0.02))
        cr=(common*common*BINS/(0.02*0.02))/((common*common*BINS/(0.02*0.02))+tot+1e-3)
        f=(math.sqrt(res/BINS),mad/BINS,rough/(BINS-1),cr)
        self.hist.append(f); self.hist=self.hist[-WINDOW:]
        score=self.score()
        if score>self.threshold: self.enter=min(ENTER,self.enter+1); self.exit=0
        elif score<self.threshold*.62: self.exit=min(EXIT,self.exit+1); self.enter=0
        else: self.enter=self.exit=0
        if self.enter>=ENTER: self.motion=True; self.last=now
        if self.motion and now-self.last>=HOLD and self.exit>=EXIT: self.motion=False
        if not self.motion and (score<self.threshold*.45 or (cr>.78 and score<self.threshold*.85)):
            self.update_base(frame,0.0015 if cr>.78 else 0.006)
        return score,self.motion
    def score(self):
        if not self.hist: return 0
        residual=sum(x[0] for x in self.hist); mad=sum(x[1] for x in self.hist); rough=sum(x[2] for x in self.hist); pen=sum(x[3] for x in self.hist); temp=0
        active=sum(1 for x in self.hist if x[0]+x[1]+x[2] > 4.0)
        for a,b in zip(self.hist,self.hist[1:]): temp+=abs(b[0]-a[0])+.4*abs(b[2]-a[2])
        inv=1/len(self.hist); common=1-min(.70,pen*inv*.70)
        return max(0,((1.8*residual+2.4*mad+1.5*rough)*inv + 1.2*(temp/max(1,len(self.hist)-1)))*common*min(1,active/4))

def quiet(t=0): return [math.sin(i*.31)*.08 for i in range(BINS)]
def noise(t=0): return [math.sin(i*.31)*.08 + math.sin(t*.17+i*1.7)*.004 for i in range(BINS)]
def common(t=0): return [quiet(t)[i]+.28 for i in range(BINS)]
def spatial(t=0, amp=.42): return [quiet(t)[i]+amp*math.sin(i*.73) for i in range(BINS)]
def motion(t=0, amp=.44): return [quiet(t)[i]+amp*math.sin(i*.55+t*.37)+.11*math.sin(i*1.9+t*.21) for i in range(BINS)]

def run(seq, threshold=9.5):
    m=MVS(threshold); out=[]
    for t,frame in enumerate(seq): out.append(m.step(frame,t*1000))
    return out

def prime(n=40): return [noise(t) for t in range(n)]

def test_silence_and_normal_noise_stay_off():
    assert not any(x[1] for x in run(prime(140)))

def test_common_rf_change_is_down_weighted():
    assert not any(x[1] for x in run(prime()+[common(t) for t in range(60)]))

def test_spatial_and_temporal_motion_turn_on():
    assert any(x[1] for x in run(prime()+[motion(t) for t in range(35)]))
    assert any(x[1] for x in run(prime()+[spatial(t) for t in range(35)]))

def test_continuous_motion_does_not_immediately_contaminate_baseline():
    out=run(prime()+[motion(t) for t in range(160)]+[noise(t) for t in range(20)])
    assert any(x[1] for x in out[50:150])

def test_isolated_spike_rejected_and_false_entry_exit():
    seq=prime()+[motion(0,1.0)]+[noise(t) for t in range(40)]
    assert not any(x[1] for x in run(seq))

def test_hold_120s_and_retrigger_extends_hold():
    out=run(prime()+[motion(t) for t in range(8)]+[noise(t) for t in range(100)])
    first=next(i for i,x in enumerate(out) if x[1])
    assert all(x[1] for x in out[first:first+100])
    out=run(prime()+[motion(t) for t in range(8)]+[noise(t) for t in range(80)]+[motion(t) for t in range(8)]+[noise(t) for t in range(80)])
    second=40+8+80
    assert all(x[1] for x in out[second:second+60])

def test_prolonged_channel_change_recovers_without_motion():
    out=run(prime()+[common(t) for t in range(180)]+[noise(t) for t in range(40)])
    assert not any(x[1] for x in out)

def test_thresholds_have_continuous_scale_not_on_on_off():
    results=[]
    for th in (9,9.5,10):
        out=run(prime()+[motion(t, .30) for t in range(35)], th)
        results.append(any(x[1] for x in out))
        scores=[x[0] for x in out[-20:]]
        assert max(scores)-min(scores) > .05
    assert results != [True, True, False]
