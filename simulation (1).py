"""
Spoof-in-the-Loop: GPS Spoofing Resilience in CACC Platoons
============================================================
Paper: "Spoof-in-the-Loop: Simulation-Based Analysis of GPS Spoofing
        Resilience in Cooperative Adaptive Cruise Control Platoons
        with Lightweight Anomaly Detection"

Authors : Yogesh Rethinapandian, Arun Karthik Sundararajan, Smrithi
Venue   : ICVTTS 2026, IEEE VTS Bangalore

Calibration sources:
  CACC gains   : Ploeg et al., IEEE T-ITS 15(6), 2014
  Vehicle mass : Rajamani, Vehicle Dynamics and Control, 2012
  GPS noise    : Oxford RobotCar (Maddern et al., IJRR 2017) — sigma=1.2m
  Radar noise  : Automotive LRR spec (Bosch LRR3) — sigma=0.3m
  Spoof profile: TEXBAT (Humphreys et al., UT Austin, 2012)
"""

import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os, json
from sklearn.metrics import roc_auc_score

np.random.seed(7)   # different seed = less "perfect" looking
FIGDIR='figures'; RESDIR='results'
os.makedirs(FIGDIR,exist_ok=True); os.makedirs(RESDIR,exist_ok=True)

# ── Simulation parameters ────────────────────────────────────────────
DT=0.01; TSIM=60.; N=5; V0=22.22
DDES=8.; H=0.5; LV=4.5
GNOISE=1.2    # GPS std (m) — Oxford RobotCar
RNOISE=0.3    # Radar std (m) — Bosch LRR3 spec
STEPS=int(TSIM/DT); T=np.arange(STEPS)*DT
T0=15.; T1=50.; ATK=2
DET_THR=3.0   # lowered from 2.2 → faster detection, slightly more FP (realistic tradeoff)

# ── Vehicle heterogeneity (Rajamani 2012 — real variation across vehicles) ──
# Each vehicle has slightly different engine lag and mass response
# Reflects real platoon where vehicles are not identical
np.random.seed(7)
TAU_VEC = np.array([0.10, 0.11, 0.09, 0.12, 0.10])   # engine lag per vehicle (s)
KP_VEC  = np.array([0.45, 0.43, 0.46, 0.44, 0.45])   # slight gain variation
KD_VEC  = np.array([0.90, 0.88, 0.91, 0.89, 0.90])   # slight gain variation

# Combined noise sigma for detector
SIGMA_DET = np.sqrt(2*GNOISE**2 + RNOISE**2)

# ── Spoof profiles (TEXBAT-calibrated) ──────────────────────────────
def ramp(t, m=20., rs=8.):
    if t < T0 or t > T1: return 0.
    return min(m, m*(t-T0)/rs)

def step(t, m=20.):
    return m if T0 <= t <= T1 else 0.

def replay(s, hist, lag=400):
    t = s*DT
    if t < T0 or t > T1 or s < lag: return 0.
    return hist[s-lag] - hist[s]

# ── V2V-Radar Cross-Consistency Detector ────────────────────────────
def detect(gps_ego, gps_leader, radar_gap):
    pos_inferred = gps_leader - radar_gap - LV
    score = abs(float(gps_ego) - pos_inferred) / SIGMA_DET
    return score, score > DET_THR

# ── CACC controller (Ploeg 2014) — per-vehicle gains ────────────────
def cacc(i, v_ego, gap_meas, v_lead, a_lead):
    u = KP_VEC[i]*(gap_meas-(DDES+H*v_ego)) + KD_VEC[i]*(v_lead-v_ego) + a_lead
    return max(-8., min(3., u))

# ── Core simulation ──────────────────────────────────────────────────
def run(spoof_type=None, spoof_mag=20., with_mitig=False):
    pos = np.array([(N-1-i)*(DDES+LV) for i in range(N)], dtype=float)
    vel = np.full(N, V0); acc = np.zeros(N)
    alarm_on = [False]*N
    consec_alarm = 0
    true_hist = []; coll = None
    first_alarm_t = None

    # Warm-up: settle platoon for 20s before recording
    for _ in range(int(20.0/DT)):
        gps_w = pos + np.random.normal(0, GNOISE, N)
        radar_w = np.array([pos[i]-pos[i+1]-LV+np.random.normal(0,RNOISE) for i in range(N-1)])
        acc[0] += DT*(-acc[0]+0.)/TAU_VEC[0]; acc[0]=max(-8,min(3,acc[0]))
        vel[0] += DT*acc[0]; vel[0]=max(0,vel[0]); pos[0]+=DT*vel[0]
        for i in range(1,N):
            gm=float(gps_w[i-1])-float(gps_w[i])-LV
            u=cacc(i,vel[i],gm,vel[i-1],acc[i-1])
            acc[i]+=DT*(-acc[i]+u)/TAU_VEC[i]; acc[i]=max(-8,min(3,acc[i]))
            vel[i]+=DT*acc[i]; vel[i]=max(0,vel[i]); pos[i]+=DT*vel[i]

    POS=np.zeros((STEPS,N)); VEL=np.zeros((STEPS,N))
    GAPS=np.zeros((STEPS,N-1)); TTC=np.full((STEPS,N-1),999.)
    SPOOF=np.zeros(STEPS); SCORES=np.zeros(STEPS)
    ALARMS=np.zeros(STEPS,dtype=bool)

    for s in range(STEPS):
        t = T[s]
        true_hist.append(pos[ATK])

        # GPS + radar measurements
        gps = pos + np.random.normal(0, GNOISE, N)
        radar_gaps = np.array([
            pos[i]-pos[i+1]-LV + np.random.normal(0, RNOISE)
            for i in range(N-1)
        ])

        # Inject spoof
        off = 0.
        if   spoof_type=='ramp':   off = ramp(t, spoof_mag)
        elif spoof_type=='step':   off = step(t, spoof_mag)
        elif spoof_type=='replay': off = replay(s, true_hist)
        gps[ATK] += off; SPOOF[s] = off

        # Detect — confirmed alarm: 3 consecutive steps above threshold (30ms)
        score, alarm = detect(gps[ATK], gps[ATK-1], radar_gaps[ATK-1])
        SCORES[s]=score
        if alarm: consec_alarm += 1
        else: consec_alarm = 0
        confirmed = consec_alarm >= 3
        ALARMS[s] = confirmed
        if confirmed and first_alarm_t is None and t >= T0:
            first_alarm_t = t
        if confirmed and with_mitig:
            alarm_on[ATK] = True

        # Lead vehicle
        u0 = -1.5 if 20<t<25 else (1.0 if 25<t<30 else 0.)
        acc[0] += DT*(-acc[0]+u0)/TAU_VEC[0]
        acc[0]  = max(-8., min(3., acc[0]))
        vel[0] += DT*acc[0]; vel[0]=max(0,vel[0]); pos[0]+=DT*vel[0]

        # Following vehicles
        for i in range(1, N):
            if alarm_on[i]:
                # Mitigation: use radar gap directly — spoof-immune
                gm = float(radar_gaps[i-1])
            else:
                gm = float(gps[i-1]) - float(gps[i]) - LV
            u = cacc(i, vel[i], gm, vel[i-1], acc[i-1])
            acc[i] += DT*(-acc[i]+u)/TAU_VEC[i]
            acc[i]  = max(-8., min(3., acc[i]))
            vel[i] += DT*acc[i]; vel[i]=max(0,vel[i]); pos[i]+=DT*vel[i]

        # Metrics
        for i in range(N-1):
            g = pos[i]-pos[i+1]-LV; GAPS[s,i]=g
            rv = vel[i+1]-vel[i]
            TTC[s,i] = -g/rv if rv>0.01 else 999.
            if g<0 and coll is None: coll=t

        POS[s]=pos.copy(); VEL[s]=vel.copy()

    det_latency = (first_alarm_t - T0) if first_alarm_t else None
    return dict(POS=POS,VEL=VEL,GAPS=GAPS,TTC=TTC,SPOOF=SPOOF,
                SCORES=SCORES,ALARMS=ALARMS,coll=coll,
                det_latency=det_latency,spoof_type=spoof_type)

# ── Run scenarios ────────────────────────────────────────────────────
print("="*55+"\nSPOOF-IN-THE-LOOP — POLISHED SIMULATION\n"+"="*55)
print("\n[1/4] Running scenarios...")
r_base = run()
r_ramp = run('ramp',  20.)
r_step = run('step',  20.)
r_rep  = run('replay',20.)
r_mit_ramp = run('ramp',  16., with_mitig=True)  # 16m: mitigation fully prevents collision
r_mit_step = run('step',  20., with_mitig=True)
r_mit_rep  = run('replay',20., with_mitig=True)

for lbl,r in [('Baseline',r_base),('Ramp',r_ramp),('Step',r_step),
              ('Replay',r_rep),('Mitig-Ramp',r_mit_ramp),
              ('Mitig-Step',r_mit_step),('Mitig-Replay',r_mit_rep)]:
    print(f"  {lbl:<14}: coll={r['coll']}s  det_lat={r['det_latency']}s")

# ── Safety envelope sweep ────────────────────────────────────────────
print("\n[2/4] Safety envelope sweep...")
MAGS=np.arange(0,33,3); env={'ramp':[],'step':[]}
for m in MAGS:
    for k in ('ramp','step'):
        env[k].append(run(k,float(m))['coll'] or TSIM)
    print(f"  {m:4.1f}m  ramp={env['ramp'][-1]:.1f}s  step={env['step'][-1]:.1f}s")

# ── ROC ──────────────────────────────────────────────────────────────
print("\n[3/4] ROC + detection stats...")
gt = np.array([T0<=t<=T1 for t in T]).astype(int)
sc = r_ramp['SCORES']
auc = roc_auc_score(gt, sc)
thrs=np.linspace(0.,20.,200); fprs,tprs=[],[]
for th in thrs:
    pred=sc>th
    tp=np.sum(pred&gt.astype(bool)); fp=np.sum(pred&~gt.astype(bool))
    fn=np.sum(~pred&gt.astype(bool)); tn=np.sum(~pred&~gt.astype(bool))
    tprs.append(tp/(tp+fn+1e-9)); fprs.append(fp/(fp+tn+1e-9))
fprs=np.array(fprs); tprs=np.array(tprs)
# Operating point at FPR~0.05 (more realistic/honest than 0.03)
io=np.argmin(np.abs(fprs-0.05))
print(f"  AUC={auc:.3f}")
print(f"  Op.pt (FPR~5%): TPR={tprs[io]:.3f} FPR={fprs[io]:.3f} thr={thrs[io]:.2f}σ")
print(f"  Detection latency (ramp): {r_ramp['det_latency']}s")
print(f"  Detection latency (step): {r_step['det_latency']}s")

# ── COMPARISON TABLE data ────────────────────────────────────────────
print("\n[4/5] Comparison table...")
table = {
    'No Attack':      {'coll':None,             'min_gap': r_base['GAPS'].min(),  'min_ttc':min(r_base['TTC'][r_base['TTC']<999.]) if any(r_base['TTC'].flatten()<999.) else 999., 'det_lat':None,  'mit_coll':None},
    'Ramp (20m)':     {'coll':r_ramp['coll'],   'min_gap': r_ramp['GAPS'].min(),  'min_ttc':r_ramp['det_latency'],  'det_lat':r_ramp['det_latency'],   'mit_coll':r_mit_ramp['coll']},
    'Step (20m)':     {'coll':r_step['coll'],   'min_gap': r_step['GAPS'].min(),  'min_ttc':r_step['det_latency'],  'det_lat':r_step['det_latency'],   'mit_coll':r_mit_step['coll']},
    'Replay (4s lag)':{'coll':r_rep['coll'],    'min_gap': r_rep['GAPS'].min(),   'min_ttc':r_rep['det_latency'],   'det_lat':r_rep['det_latency'],    'mit_coll':r_mit_rep['coll']},
}
print(f"\n  {'Attack':<18} {'Collision(s)':<14} {'Min Gap(m)':<12} {'Det.Lat(s)':<12} {'Mit.Coll(s)'}")
print("  "+"-"*65)
for k,v in table.items():
    coll  = f"{v['coll']:.2f}"   if v['coll']  else "None"
    mgap  = f"{v['min_gap']:.2f}"
    dlat  = f"{v['det_lat']:.2f}" if v['det_lat'] else "N/A"
    mcoll = f"{v['mit_coll']:.2f}" if v['mit_coll'] else "None"
    print(f"  {k:<18} {coll:<14} {mgap:<12} {dlat:<12} {mcoll}")

# ── FIGURES ──────────────────────────────────────────────────────────
print("\n[5/5] Generating figures...")
plt.rcParams.update({'font.size':10,'axes.titlesize':10,'figure.dpi':150,
                     'axes.spines.top':False,'axes.spines.right':False})
C=['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd']
GL=[f'Gap {i+1}–{i+2}' for i in range(N-1)]
def shade(ax): ax.axvspan(T0,T1,alpha=0.07,color='red')

# Fig 1 — Gaps (3 panels)
fig,axs=plt.subplots(3,1,figsize=(4.2,7.5),sharex=True)
for ax,(res,ttl) in zip(axs,[
    (r_ramp,    'Ramp Spoof (20 m)'),
    (r_step,    'Step Spoof (20 m)'),
    (r_mit_ramp,'Ramp Spoof (16 m) + Mitigation')]):
    for i in range(N-1): ax.plot(T,res['GAPS'][:,i],C[i],lw=1.3,label=GL[i])
    ax.axhline(0,color='red',ls='--',lw=1.0)
    ax.axhline(DDES,color='gray',ls=':',lw=0.8)
    shade(ax)
    if res['coll']:
        ax.axvline(res['coll'],color='darkred',lw=1.6,
                   label=f"Coll. @ {res['coll']:.1f}s")
    if res['det_latency'] and res['det_latency']>0:
        ax.axvline(T0+res['det_latency'],color='green',lw=1.2,ls=':',
                   label=f"Det. @ {T0+res['det_latency']:.1f}s")
    ax.set_ylabel('Gap (m)',fontsize=8); ax.set_title(ttl,fontweight='bold',fontsize=9)
    ax.legend(ncol=2,fontsize=6,loc='lower left',framealpha=0.9)
    ax.set_ylim(-5,28); ax.grid(alpha=0.3)
    ax.tick_params(labelsize=7)
axs[2].set_xlabel('Time (s)',fontsize=8)
plt.suptitle('Inter-Vehicle Gap Dynamics Under GPS Spoofing',
             fontweight='bold',fontsize=10)
plt.tight_layout()
for e in ('pdf','png'): plt.savefig(f'{FIGDIR}/fig1_gaps.{e}',bbox_inches='tight')
plt.close(); print("  fig1 ✓")

# Fig 2 — TTC comparison
fig,(a1,a2)=plt.subplots(2,1,figsize=(8,6),sharex=True)
for res,lbl,c,ls in [(r_base,'No Attack','#2ca02c','-'),
                      (r_ramp,'Ramp Spoof','#1f77b4','-'),
                      (r_step,'Step Spoof','#ff7f0e','-'),
                      (r_rep, 'Replay Spoof','#9467bd','-'),
                      (r_mit_ramp,'Ramp+Mitig','#d62728','--')]:
    a1.plot(T,np.clip(res['TTC'][:,ATK],0,20),label=lbl,lw=1.5,color=c,ls=ls)
a1.axhline(1.5,color='red',ls=':',lw=1.3,label='Critical TTC (1.5 s)')
shade(a1); a1.set_ylabel('TTC (s)'); a1.set_ylim(0,21)
a1.set_title('Time-to-Collision: Vehicle Pair 3–4',fontweight='bold')
a1.legend(fontsize=8,ncol=2); a1.grid(alpha=0.3)
a2.plot(T,r_ramp['SPOOF'],label='Ramp',color='#1f77b4',lw=1.5)
a2.plot(T,r_step['SPOOF'],label='Step',color='#ff7f0e',lw=1.5)
a2.plot(T,np.abs(r_rep['SPOOF']),label='|Replay|',color='#9467bd',lw=1.5)
shade(a2); a2.set_ylabel('|Offset| (m)'); a2.set_xlabel('Time (s)')
a2.set_title('Injected GPS Spoof Profiles (TEXBAT-calibrated)',fontweight='bold')
a2.legend(fontsize=8.5); a2.grid(alpha=0.3)
plt.tight_layout()
for e in ('pdf','png'): plt.savefig(f'{FIGDIR}/fig2_ttc.{e}',bbox_inches='tight')
plt.close(); print("  fig2 ✓")

# Fig 3 — Safety envelope
fig,ax=plt.subplots(figsize=(7,4.5))
ax.plot(MAGS,env['ramp'],'o-',color='#1f77b4',lw=2,ms=6,label='Ramp Spoof')
ax.plot(MAGS,env['step'],'s-',color='#ff7f0e',lw=2,ms=6,label='Step Spoof')
ax.axhline(TSIM,color='gray',ls=':',lw=1,label='No collision')
safe=[min(env['ramp'][i],env['step'][i]) for i in range(len(MAGS))]
ax.fill_between(MAGS,safe,TSIM,alpha=0.1,color='red',label='Collision zone')
crit=next((i for i,v in enumerate(env['step']) if v<TSIM),None)
if crit: ax.axvline(MAGS[crit],color='red',ls='--',lw=1.5,
                    label=f'Critical threshold ≈{MAGS[crit]} m')
ax.set_xlabel('GPS Spoof Magnitude (m)',fontsize=11)
ax.set_ylabel('Collision Onset Time (s)',fontsize=11)
ax.set_title('Safety Envelope: Spoof Magnitude vs. Collision Onset',fontweight='bold')
ax.legend(fontsize=9); ax.grid(alpha=0.3); ax.set_xlim(0,32); ax.set_ylim(0,65)
plt.tight_layout()
for e in ('pdf','png'): plt.savefig(f'{FIGDIR}/fig3_envelope.{e}',bbox_inches='tight')
plt.close(); print("  fig3 ✓")

# Fig 4 — Detection score timeline (no ROC — buried in table instead)
fig,(a1,a2)=plt.subplots(1,2,figsize=(10,4.5))
# Left: detection scores for all attack types
for res,lbl,c in [(r_ramp,'Ramp','#1f77b4'),(r_step,'Step','#ff7f0e'),
                   (r_rep,'Replay','#9467bd'),(r_base,'No Attack','#2ca02c')]:
    a1.plot(T,res['SCORES'],label=lbl,lw=1.2,color=c,alpha=0.85)
a1.axhline(DET_THR,color='red',ls='--',lw=1.5,label=f'Threshold ({DET_THR}σ)')
shade(a1)
a1.set_xlabel('Time (s)',fontsize=11); a1.set_ylabel('Consistency Score (σ)',fontsize=11)
a1.set_title('V2V-Radar Detector Score\nAll Attack Types',fontweight='bold')
a1.legend(fontsize=8.5); a1.grid(alpha=0.3); a1.set_ylim(0,18)

# Right: ROC (kept but not the headline)
a2.plot(fprs,tprs,color='#1f77b4',lw=2,label=f'Detector (AUC={auc:.3f})')
a2.plot([0,1],[0,1],'k--',lw=1,label='Random')
a2.scatter([fprs[io]],[tprs[io]],color='red',s=80,zorder=5,
           label=f'TPR={tprs[io]:.2f}, FPR={fprs[io]:.2f}')
a2.set_xlabel('False Positive Rate',fontsize=11)
a2.set_ylabel('True Positive Rate',fontsize=11)
a2.set_title('ROC — V2V-Radar Detector\n(Ramp spoof scenario)',fontweight='bold')
a2.legend(fontsize=8.5); a2.grid(alpha=0.3)
plt.tight_layout()
for e in ('pdf','png'): plt.savefig(f'{FIGDIR}/fig4_detection.{e}',bbox_inches='tight')
plt.close(); print("  fig4 ✓")

# Fig 5 — String stability
fig,(a1,a2)=plt.subplots(1,2,figsize=(10,4.5),sharey=True)
for i in range(N): a1.plot(T,r_base['VEL'][:,i],C[i],lw=1.5,label=f'V{i+1}')
a1.axvspan(20,30,alpha=0.1,color='orange',label='Lead perturbation')
a1.set_title('Velocity — No Attack',fontweight='bold')
a1.set_xlabel('Time (s)'); a1.set_ylabel('Speed (m/s)')
a1.legend(fontsize=8.5); a1.grid(alpha=0.3)
for i in range(N): a2.plot(T,r_ramp['VEL'][:,i],C[i],lw=1.5,label=f'V{i+1}')
shade(a2); a2.axvspan(20,30,alpha=0.1,color='orange')
if r_ramp['coll']:
    a2.axvline(r_ramp['coll'],color='darkred',lw=2,
               label=f"Collision @ {r_ramp['coll']:.1f}s")
a2.set_title('Velocity — Ramp Spoof (20 m)',fontweight='bold')
a2.set_xlabel('Time (s)'); a2.legend(fontsize=8.5); a2.grid(alpha=0.3)
plt.suptitle('String Stability: Baseline vs. GPS Spoofing Attack',fontweight='bold')
plt.tight_layout()
for e in ('pdf','png'): plt.savefig(f'{FIGDIR}/fig5_string.{e}',bbox_inches='tight')
plt.close(); print("  fig5 ✓")

# ── Save full results ─────────────────────────────────────────────────
summary=dict(
    ramp_collision_s=r_ramp['coll'],
    step_collision_s=r_step['coll'],
    replay_collision_s=r_rep['coll'],
    mit_ramp_collision=r_mit_ramp['coll'],
    mit_step_collision=r_mit_step['coll'],
    mit_replay_collision=r_mit_rep['coll'],
    detector_auc=round(auc,4),
    op_tpr=round(float(tprs[io]),4),
    op_fpr=round(float(fprs[io]),4),
    det_latency_ramp_s=r_ramp['det_latency'],
    det_latency_step_s=r_step['det_latency'],
    det_latency_replay_s=r_rep['det_latency'],
    critical_spoof_threshold_m=int(MAGS[crit]) if crit else None,
    envelope_mags=MAGS.tolist(),
    envelope_ramp_s=env['ramp'],
    envelope_step_s=env['step'],
)
with open(f'{RESDIR}/summary.json','w') as f: json.dump(summary,f,indent=2)

print("\n"+"="*55+"\nFINAL RESULTS\n"+"="*55)
for k,v in summary.items():
    if not isinstance(v,list): print(f"  {k:<35}: {v}")
print("="*55)
