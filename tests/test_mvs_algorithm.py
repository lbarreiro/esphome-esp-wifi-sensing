import math
import random

BINS = 48
BOOT = 20
WINDOW = 32
UPDATE_MS = 1000
ENTER = 3
EXIT = 3
HOLD = 120_000

class MVS:
    def __init__(self, threshold=9.5, hold_ms=HOLD):
        self.threshold=threshold; self.hold_ms=max(HOLD, hold_ms)
        self.acc=[0.0]*BINS; self.acc_count=0; self.timing=False; self.last_obs=0; self.obs_count=0
        self.baseline=None; self.noise=[0.08]*BINS; self.samples=0
        self.hist=[]; self.motion=False; self.last_motion=0; self.enter=0; self.exit=0; self.last_score=0.0
    def update_base(self, frame, channel=False):
        if self.baseline is None:
            self.baseline=frame[:]; self.samples=1; return
        if self.samples < BOOT:
            for i,x in enumerate(frame):
                d=x-self.baseline[i]; self.baseline[i]+=d/(self.samples+1); self.noise[i]=max(0.02,self.noise[i]+(abs(d)-self.noise[i])/(self.samples+1))
            self.samples+=1; return
        a=0.0015 if channel else 0.006
        for i,x in enumerate(frame):
            d=x-self.baseline[i]; self.baseline[i]+=a*d; self.noise[i]=max(0.02,self.noise[i]+a*(abs(d)-self.noise[i]))
    def observe(self, frame, now):
        if not self.timing: self.timing=True; self.last_obs=now
        for i,x in enumerate(frame): self.acc[i]+=x
        self.acc_count+=1
        if now-self.last_obs < UPDATE_MS: return None
        obs=[x/self.acc_count for x in self.acc]
        self.acc=[0.0]*BINS; self.acc_count=0; self.last_obs=now; self.obs_count+=1
        return obs
    def step(self, frame, now):
        obs=self.observe(frame, now)
        if obs is None: return self.last_score,self.motion,False,self.obs_count
        if self.samples < BOOT:
            self.update_base(obs); self.last_score=0; return self.last_score,self.motion,True,self.obs_count
        common=sum(obs[i]-self.baseline[i] for i in range(BINS))/BINS
        spatial=mad=rough=0.0
        for i in range(BINS):
            n=max(self.noise[i],0.02); d=obs[i]-self.baseline[i]; r=(d-common)/n
            spatial+=r*r; mad+=abs(r)
            if i: rough+=abs(((d-common)-((obs[i-1]-self.baseline[i-1])-common))/max((self.noise[i]+self.noise[i-1])/2,0.02))
        common_energy=common*common*BINS/(0.02*0.02)
        f=(math.sqrt(spatial/BINS),mad/BINS,rough/(BINS-1),common_energy/(common_energy+spatial+1e-3))
        self.hist.append(f); self.hist=self.hist[-WINDOW:]
        self.last_score=self.score()
        if self.last_score>self.threshold: self.enter=min(ENTER,self.enter+1); self.exit=0
        elif self.last_score<self.threshold*.62: self.exit=min(EXIT,self.exit+1); self.enter=0
        else: self.enter=self.exit=0
        if self.enter>=ENTER: self.motion=True; self.last_motion=now
        if self.motion and now-self.last_motion>=self.hold_ms and self.exit>=EXIT: self.motion=False
        if not self.motion and (self.last_score<self.threshold*.45 or (f[3]>.78 and self.last_score<self.threshold*.85)):
            self.update_base(obs, f[3]>.78)
        return self.last_score,self.motion,True,self.obs_count
    def score(self):
        residual=sum(x[0] for x in self.hist); mad=sum(x[1] for x in self.hist); rough=sum(x[2] for x in self.hist); pen=sum(x[3] for x in self.hist)
        active=sum(1 for x in self.hist if x[0]+x[1]+x[2] > 4.0); temp=0.0
        for a,b in zip(self.hist,self.hist[1:]): temp+=abs(b[0]-a[0])+.4*abs(b[2]-a[2])
        inv=1/len(self.hist); common=1-min(.75,pen*inv*.75)
        return max(0,((1.8*residual+2.4*mad+1.5*rough)*inv + 1.2*(temp/max(1,len(self.hist)-1)))*common*min(1,active/4))

def quiet(t=0): return [2.0+math.sin(i*.31)*.08 for i in range(BINS)]
def noise(t=0): return [2.0+math.sin(i*.31)*.08 + math.sin(t*.17+i*1.7)*.004 for i in range(BINS)]
def random_noise(t=0):
    rnd=random.Random(t)
    return [2.0+math.sin(i*.31)*.08 + rnd.uniform(-.012,.012) for i in range(BINS)]
def common(t=0): return [x+.28 for x in quiet(t)]
def spatial(t=0, amp=.42): return [quiet(t)[i]+amp*math.sin(i*.73) for i in range(BINS)]
def motion(t=0, amp=.44): return [quiet(t)[i]+amp*math.sin(i*.55+t*.37)+.11*math.sin(i*1.9+t*.21) for i in range(BINS)]

def run(seq, threshold=9.5, hold_ms=HOLD, step_ms=1000):
    m=MVS(threshold, hold_ms); out=[]
    for t,frame in enumerate(seq): out.append(m.step(frame,t*step_ms))
    return out

def prime(n=45): return [noise(t) for t in range(n)]

def test_silence_prolongado_e_ruido_aleatorio_nao_ligam():
    assert not any(x[1] for x in run(prime(160)))
    assert not any(x[1] for x in run([random_noise(t) for t in range(160)]))

def test_common_mode_forte_tem_score_menor_que_alteracao_espacial():
    common_out=run(prime()+[common(t) for t in range(40)])
    spatial_out=run(prime()+[spatial(t) for t in range(40)])
    assert max(x[0] for x in common_out) < max(x[0] for x in spatial_out) * 0.45
    assert not any(x[1] for x in common_out)

def test_movimento_temporal_persistente_supera_ruido_e_liga():
    quiet_out=run(prime(90))
    move_out=run(prime()+[motion(t) for t in range(40)])
    assert max(x[0] for x in move_out) > max(x[0] for x in quiet_out) + 9.5
    assert any(x[1] for x in move_out)

def test_pico_isolado_e_falsa_entrada_saida_rejeitados():
    seq=prime()+[motion(0,1.0)]+[noise(t) for t in range(60)]
    assert not any(x[1] for x in run(seq))

def test_thresholds_9_95_10_nao_sao_interruptor_de_escala():
    results=[]
    for th in (9,9.5,10):
        out=run(prime()+[motion(t, .30) for t in range(40)], th)
        results.append(any(x[1] for x in out))
        scores=[x[0] for x in out[-20:]]
        assert max(scores)-min(scores) > .05
    assert results != [True, True, False]

def test_janela_temporal_1hz_e_32_observacoes_em_32s():
    m=MVS()
    for i in range(10):
        score, state, added, count = m.step(noise(i), i * 100)
        assert not added
        assert count == 0
    for sec in range(1, 33):
        for sub in range(10):
            score, state, added, count = m.step(noise(sec*10+sub), 900 + sec * 1000 + sub * 50)
        assert count == sec
    assert m.obs_count == 32

def test_hold_minimo_120s_retrigger_e_saida_com_histerese():
    out=run(prime()+[motion(t) for t in range(8)]+[noise(t) for t in range(150)], hold_ms=1)
    first=next(i for i,x in enumerate(out) if x[1])
    assert all(x[1] for x in out[first:first+120])
    assert not out[-1][1]
    out=run(prime()+[motion(t) for t in range(8)]+[noise(t) for t in range(80)]+[motion(t) for t in range(8)]+[noise(t) for t in range(100)])
    second=40+8+80
    assert all(x[1] for x in out[second:second+100])

def test_alteracao_prolongada_canal_recupera_sem_motion_persistente():
    out=run(prime()+[common(t) for t in range(180)]+[noise(t) for t in range(40)])
    assert not any(x[1] for x in out)

def test_algorithm_mvs_nao_usa_average_variation_para_decidir_motion():
    source=open('components/esp_wifi_sensing/wifi_sensing.cpp', encoding='utf-8').read()
    mvs_block=source[source.index('if (this->selected_algorithm_ == CsiAlgorithm::MVS)'):source.index('if (this->new_csi_sample_)')]
    assert 'average_variation' not in mvs_block
    callback=source[source.index('if (self->selected_algorithm_ == CsiAlgorithm::MVS)'):source.index('} else if (self->selected_algorithm_ == CsiAlgorithm::VARIANCE)')]
    assert 'mvs_algorithm_.process' in callback

def test_sem_sensores_diagnostico_e_sem_gpio15_no_cpp():
    import pathlib
    assert not pathlib.Path('components/esp_wifi_sensing/sensor.py').exists()
    combined='\n'.join(p.read_text(encoding='utf-8') for p in pathlib.Path('components/esp_wifi_sensing').glob('*.cpp'))
    assert 'GPIO_NUM_15' not in combined
    assert 'gpio_set_level' not in combined
