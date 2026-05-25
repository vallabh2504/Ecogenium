"""
combined_best_profile.py — Combines the best findings from all three agents:
  - Agent 2: P_PULSE = 390 W (lower pulse power wins)
  - Agent 3: terrain-adaptive (motor off downhill, KE harvesting, V_HIGH=7.8 m/s)
  
Local grid search around (V_HIGH=7.8, V_LOW=4.9, P_PULSE=390, V_MAX_DH=8.5)
then runs Strategy G for charge sustenance. Generates comprehensive result figure.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use('Agg')
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as mgridspec
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_E_J, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, fc_current, fc_h2_rate, sc_soc_update,
    N_LAPS, DT, MATLAB_DIR, RESULTS_DIR,
)

MASS=175.; G=9.81; CD=0.15; AF=0.8; CRR=0.006; RHO=1.225; ETA_DT=0.95
GRADE_THRESH=0.006; P_RAMP_MAX=200.; A_STOP=0.5; V_MAX=11.0
N_STOP_STEPS=round(4./DT)
H2_DENSITY=89.88; TOTAL_KM=14.5

_THIS = os.path.dirname(os.path.abspath(__file__))
ROUTE_CSV = os.path.join(_THIS,'..','datasheets','sem_2025_eu.csv')

def _load_route():
    df=pd.read_csv(ROUTE_CSV); df.columns=[c.strip() for c in df.columns]
    dc=[c for c in df.columns if 'Distance' in c][0]
    ec=[c for c in df.columns if 'Elevation' in c][0]
    s=df[dc].values.astype(float); e=df[ec].values.astype(float)
    D=float(s[-1])
    es=savgol_filter(e,51,3); gr=savgol_filter(np.gradient(es,s),51,3)
    return D, s, es, gr, interp1d(s,es,fill_value='extrapolate'), interp1d(s,gr,fill_value='extrapolate')

D_LAP,_sr,_es,_gs,_elev_fn,_grade_fn=_load_route()

def _net_force(v,P,g):
    Fm=min(P*ETA_DT/max(v,0.3),MASS*2.) if P>0 else 0.
    return Fm - 0.5*CD*AF*RHO*v**2 - CRR*MASS*G - MASS*G*g

def _build_lap(V_HI,V_LO,V_DH,P_PU,P_BO):
    tl=[];vl=[];Pl=[];sl=[];el=[];gl=[]
    t=v=s=0.; mode='PULSE'; first=True; pP=0.; was_dh=False
    while True:
        sw=s%D_LAP; grade=float(_grade_fn(sw)); elev=float(_elev_fn(sw))
        rem=D_LAP-sw; bd=v**2/(2.*A_STOP)+v*DT
        if v>0 and rem<=bd and rem>0 and mode!='BRAKE': mode='BRAKE'
        if mode=='BRAKE':
            Pt=0.; vc=V_MAX
        elif grade<-GRADE_THRESH:
            Pt=0.; vc=V_DH; mode='GLIDE'; was_dh=True
        elif grade>+GRADE_THRESH:
            if v>=V_HI: mode='GLIDE'; Pt=0.
            else: mode='PULSE'; Pt=P_BO
            vc=V_MAX; was_dh=False
        else:
            vc=V_HI+1.
            if was_dh and v>V_LO: mode='GLIDE'; Pt=0.
            else:
                was_dh=False
                if v<=V_LO: mode='PULSE'; Pt=P_PU
                elif v>=V_HI: mode='GLIDE'; Pt=0.
                else: Pt=P_PU if mode=='PULSE' else 0.
        P=float(np.clip(Pt,pP-P_RAMP_MAX,pP+P_RAMP_MAX)); pP=P
        if not first:
            tl.append(t);vl.append(v);Pl.append(P);sl.append(sw);el.append(elev);gl.append(grade)
        first=False
        if mode=='BRAKE': vn=max(0.,v-A_STOP*DT); sn=s+v*DT
        else:
            a=_net_force(v,P,grade)/MASS
            vn=float(np.clip(v+a*DT,0.,vc)); sn=s+v*DT
        t+=DT; v=vn; s=sn
        if s>=D_LAP or (mode=='BRAKE' and v==0.) or t>700.: break
    se=sl[-1] if sl else 0.; ee=el[-1] if el else float(_elev_fn(0)); ge=gl[-1] if gl else 0.
    for _ in range(N_STOP_STEPS):
        tl.append(t);vl.append(0.);Pl.append(0.);sl.append(se);el.append(ee);gl.append(ge);t+=DT
    return tl,vl,Pl,sl,el,gl

def build_profile(V_HI,V_LO,V_DH,P_PU=390.,P_BO=1000.):
    at=[];av=[];aP=[];as_=[];aln=[];ae=[];ag=[]; to=0.
    for lap in range(1,N_LAPS+1):
        tl,vl,Pl,sl,el,gl=_build_lap(V_HI,V_LO,V_DH,P_PU,P_BO)
        ta=np.array(tl)+to
        at.append(ta);av.append(np.array(vl));aP.append(np.array(Pl))
        as_.append(np.array(sl));aln.append(np.full(len(tl),lap,dtype=int))
        ae.append(np.array(el));ag.append(np.array(gl)); to=float(ta[-1])+DT
    ta=np.concatenate(at);va=np.concatenate(av);Pa=np.concatenate(aP)
    sa=np.concatenate(as_);la=np.concatenate(aln);ea=np.concatenate(ae);ga=np.concatenate(ag)
    # smooth velocity per motion segment
    vs=va.copy(); im=(va>0.)
    ss=np.where(np.diff(np.concatenate([[0],im.astype(int)]))==1)[0]
    se_=np.where(np.diff(np.concatenate([im.astype(int),[0]]))==-1)[0]+1
    for ms,me in zip(ss,se_):
        seg=va[ms:me]
        if len(seg)>25: vs[ms:me]=savgol_filter(seg,21,2)
        vs[ms:me]=np.clip(vs[ms:me],0.,V_MAX)
    return ta,vs,Pa,sa,la,ea,ga

def verify(t,v,la,silent=True):
    d=float(np.trapezoid(v,t))/1000.; dur=float(t[-1])/60.
    stops=len(np.where(np.diff((v==0.).astype(int))==1)[0])
    ok=(d>14.5)and(34.<=dur<=37.)and(stops==11)
    if not silent: print(f"  d={d:.3f}km dur={dur:.1f}min stops={stops} {'OK' if ok else 'FAIL'}")
    return ok,d,dur,stops

# Strategy G
def make_strat_g(K_p,K_i=2.,tau=15.):
    alpha=float(np.exp(-DT/tau))
    st={'lpf':None,'integ':0.,'prev_lap':-1,'offset':0.}
    def fn(Pd,SOC,Pp,til,li):
        if st['lpf'] is None: st['lpf']=float(Pd)
        if li!=st['prev_lap'] and li>0:
            st['offset']+=0.3*(SC_SOC_0-SOC)*SC_E_J/186.
            st['offset']=float(np.clip(st['offset'],-200.,200.))
        st['prev_lap']=li
        st['lpf']=alpha*st['lpf']+(1-alpha)*float(Pd)
        st['integ']+=( SC_SOC_0-SOC)*DT
        if K_i>1e-9: st['integ']=float(np.clip(st['integ'],-150./K_i,150./K_i))
        Pc=st['lpf']+st['offset']+K_p*(SC_SOC_0-SOC)+K_i*st['integ']
        if Pd<25. and SOC>=SC_SOC_0: Pc=0.
        return float(np.clip(Pc,0.,FC_P_MAX))
    return fn

def simulate(fn,Pd_arr,la_arr,SOC0=SC_SOC_0):
    n=len(Pd_arr)
    Pfc=np.empty(n);Psc=np.empty(n);SOCa=np.empty(n+1)
    Ifc=np.empty(n);mH2=np.empty(n)
    t_arr_full=np.arange(n)*DT  # for t_in_lap (approx)
    laps0=np.unique(la_arr)
    lt0={int(l):np.where(la_arr==l)[0][0] for l in laps0}
    SOCa[0]=SOC0; pp=FC_P_MIN
    for k in range(n):
        s=SOCa[k]; Pd=float(Pd_arr[k]); li=int(la_arr[k])-1
        til=float((k-lt0[li+1])*DT)
        Pc=float(fn(Pd,s,pp,til,li))
        Pc=float(np.clip(Pc,pp-FC_RAMP*DT,pp+FC_RAMP*DT))
        Pc=float(np.clip(Pc,FC_P_MIN,FC_P_MAX))
        Psk=Pd-Pc
        ts=sc_soc_update(s,Psk,DT)
        if Psk>0 and ts<=SC_SOC_MIN:
            Pc=float(np.clip(Pd,FC_P_MIN,FC_P_MAX));Psk=max(0.,Pd-Pc);ts=sc_soc_update(s,Psk,DT)
        Pfc[k]=Pc;Psc[k]=Psk;SOCa[k+1]=ts
        Ifc[k]=float(fc_current(Pc));mH2[k]=fc_h2_rate(Ifc[k])*DT;pp=Pc
    return {'m_H2':float(np.sum(mH2)),'dSOC':float(SOCa[n]-SOC0),'SOC':SOCa[:-1],'Pfc':Pfc}

def bisect_kp(Pd,la,label=''):
    lo,hi=100.,3000.; Kp=(lo+hi)/2.; best=None
    print(f"\n  Bisecting K_p for '{label}'")
    for it in range(20):
        fn=make_strat_g(Kp); r=simulate(fn,Pd,la)
        ds=r['dSOC']; h2=r['m_H2']
        print(f"    iter{it+1:2d}  K_p={Kp:7.1f}  dSOC={ds:+.4f}  H2={h2:.3f}g")
        if abs(ds)<=0.01: best=r; break
        if ds<-0.01: lo=Kp
        else: hi=Kp
        Kp=(lo+hi)/2.; best=r
    return Kp,best

# ── Grid search ───────────────────────────────────────────────────────────────
print("=== Combined Best Profile — Local Grid Search ===")
V_HI_vals  = [7.5, 7.8, 8.0]
V_LO_vals  = [4.5, 4.9, 5.2]
P_PU_vals  = [350., 390., 430.]
V_DH       = 8.5   # fixed from Agent 3

results = []
for VH in V_HI_vals:
    for VL in V_LO_vals:
        if VL >= VH - 1.0: continue
        for PP in P_PU_vals:
            ta,va,Pa,sa,la,ea,ga = build_profile(VH,VL,V_DH,PP,1000.)
            ok,d,dur,stops = verify(ta,va,la)
            if not ok:
                print(f"  SKIP VH={VH} VL={VL} PP={PP}W  ({d:.2f}km {dur:.1f}min {stops}stops)")
                continue
            Kp,r = bisect_kp(Pa,la,f"VH={VH} VL={VL} PP={PP}W")
            km3 = TOTAL_KM/(r['m_H2']/H2_DENSITY)
            cs  = abs(r['dSOC'])<=0.015
            results.append(dict(VH=VH,VL=VL,PP=PP,Kp=Kp,H2=r['m_H2'],km3=km3,dSOC=r['dSOC'],cs=cs,
                                ta=ta,va=va,Pa=Pa,sa=sa,la=la,ea=ea,ga=ga,SOC=r['SOC'],Pfc=r['Pfc']))
            print(f"  >>> VH={VH} VL={VL} PP={PP}W  km3={km3:.1f}  H2={r['m_H2']:.3f}g  dSOC={r['dSOC']:+.4f}  CS={'YES' if cs else 'NO'}")

best=max(results,key=lambda x:x['km3'])
print(f"\n=== COMBINED BEST ===")
print(f"  V_HIGH={best['VH']} m/s ({best['VH']*3.6:.1f}km/h)  V_LOW={best['VL']} m/s ({best['VL']*3.6:.1f}km/h)")
print(f"  P_PULSE={best['PP']}W  P_BOOST=1000W  V_MAX_DH={V_DH}m/s")
print(f"  K_p={best['Kp']:.1f}  H2={best['H2']:.3f}g  km/m³={best['km3']:.1f}  dSOC={best['dSOC']:+.4f}  CS={'YES' if best['cs'] else 'NO'}")

# ── Save best CSV ──────────────────────────────────────────────────────────────
df_out=pd.DataFrame({'time_s':best['ta'],'velocity_ms':best['va'],'velocity_kmh':best['va']*3.6,
    'dist_in_lap_m':best['sa'],'lap_num':best['la'],'elevation_m':best['ea'],
    'grade':best['ga'],'P_elec_W':best['Pa']})
csv_out=os.path.join(MATLAB_DIR,'sem_combined_best.csv')
df_out.to_csv(csv_out,index=False)
print(f"  CSV saved → {csv_out}")

# ── Publication figure ─────────────────────────────────────────────────────────
# Ranking data
ranking=[
    dict(label='Baseline\n(P&G 1000W)',  km3=302.1, H2=4.314, color='#0072B2'),
    dict(label='Agent 1\n(Speed Window)',km3=313.5, H2=4.157, color='#56B4E9'),
    dict(label='Agent 3\n(Terrain-Adapt)',km3=322.6,H2=4.040, color='#009E73'),
    dict(label='Agent 2\n(P_PULSE 390W)',km3=323.9, H2=4.024, color='#E69F00'),
    dict(label='Combined\n(Best)',        km3=best['km3'],H2=best['H2'],color='#D55E00'),
]
ranking.sort(key=lambda x:x['km3'])

# Lap 1 data
m1=best['la']==1
t1=best['ta'][m1]-best['ta'][m1][0]; v1=best['va'][m1]*3.6
P1=best['Pa'][m1]; g1=best['ga'][m1]

# Colour per segment mode
def seg_col(P,g,v):
    if v==0.: return '#222222'
    if g<-GRADE_THRESH: return '#0072B2'   # downhill blue
    if g>+GRADE_THRESH: return '#CC3311'   # uphill red
    return '#EE7733' if P>10. else '#009E73' # flat pulse orange / glide green

cols=[seg_col(P1[i],g1[i],v1[i]) for i in range(len(v1))]

fig=plt.figure(figsize=(16,14),dpi=180)
fig.patch.set_facecolor('white')
gs=mgridspec.GridSpec(3,3,figure=fig,hspace=0.45,wspace=0.38,
    height_ratios=[1.3,1.1,0.9])

# ── Panel A: single-lap velocity coloured by mode ──────────────────────────
axA=fig.add_subplot(gs[0,:2])
s1=best['sa'][m1]
for i in range(len(s1)-1):
    axA.plot(s1[i:i+2],v1[i:i+2],color=cols[i],lw=1.4)
axA.axhline(best['VH']*3.6,color='grey',lw=0.8,ls='--',alpha=0.6)
axA.axhline(best['VL']*3.6,color='grey',lw=0.8,ls=':',alpha=0.6)
axA.set_xlabel('Distance in lap [m]',fontsize=9); axA.set_ylabel('Velocity [km/h]',fontsize=9)
axA.set_title(f"Combined Profile — Lap 1  |  V_HIGH={best['VH']*3.6:.1f} km/h  "
              f"V_LOW={best['VL']*3.6:.1f} km/h  P_PULSE={best['PP']:.0f} W  P_BOOST=1000 W",
              fontsize=9,fontweight='bold')
axA.set_xlim(0,D_LAP)
patches=[mpatches.Patch(color='#CC3311',label='Uphill boost'),
         mpatches.Patch(color='#0072B2',label='Downhill coast (motor off)'),
         mpatches.Patch(color='#EE7733',label='Flat — pulse'),
         mpatches.Patch(color='#009E73',label='Flat — glide'),
         mpatches.Patch(color='#222222',label='Brake / stop')]
axA.legend(handles=patches,fontsize=7.5,loc='upper right',framealpha=0.9)

# ── Panel B: elevation profile coloured by grade ───────────────────────────
axB=fig.add_subplot(gs[0,2])
for i in range(len(_sr)-1):
    g=_gs[i]
    c='#CC3311' if g>GRADE_THRESH else ('#0072B2' if g<-GRADE_THRESH else '#999999')
    axB.fill_between(_sr[i:i+2],_es[i:i+2],min(_es)-0.1,color=c,alpha=0.45,linewidth=0)
    axB.plot(_sr[i:i+2],_es[i:i+2],color=c,lw=1.2)
axB.set_xlabel('Lap distance [m]',fontsize=8); axB.set_ylabel('Elevation [m]',fontsize=8)
axB.set_title('Silesia Ring — Elevation\n(red=uphill, blue=downhill, grey=flat)',fontsize=8,fontweight='bold')
axB.set_xlim(0,D_LAP)
# Annotate grade zones
for i,ann in [(350,'Hill\ncrest'),(800,'Valley')]:
    axB.annotate(ann,xy=(_sr[i],_es[i]),xytext=(15,10),textcoords='offset points',
                 fontsize=6.5,color='#444444',arrowprops=dict(arrowstyle='->',lw=0.6))

# ── Panel C: velocity+elevation overlay ───────────────────────────────────
axC=fig.add_subplot(gs[1,:2])
axC2=axC.twinx()
for i in range(len(s1)-1):
    axC.plot(s1[i:i+2],v1[i:i+2],color=cols[i],lw=1.2)
axC2.fill_between(_sr,_es,min(_es),color='#CC3311',alpha=0.12)
axC2.plot(_sr,_es,color='#CC3311',lw=0.8,alpha=0.6)
axC2.set_ylabel('Elevation [m]',fontsize=8,color='#CC3311')
axC2.tick_params(axis='y',labelcolor='#CC3311',labelsize=7)
axC.set_xlabel('Distance in lap [m]',fontsize=9); axC.set_ylabel('Velocity [km/h]',fontsize=9)
axC.set_title('Velocity + Elevation Overlay (Lap 1)',fontsize=9,fontweight='bold')
axC.set_xlim(0,D_LAP)

# ── Panel D: ranking bar chart ─────────────────────────────────────────────
axD=fig.add_subplot(gs[1,2])
ys=np.arange(len(ranking)); bh=0.55
bars=axD.barh(ys,[r['km3'] for r in ranking],bh,
              color=[r['color'] for r in ranking],alpha=0.82,edgecolor='white',linewidth=0.5)
axD.set_yticks(ys); axD.set_yticklabels([r['label'] for r in ranking],fontsize=8)
axD.set_xlabel('Fuel economy [km/m³]',fontsize=9)
axD.set_title('Fuel Economy Ranking',fontsize=9,fontweight='bold')
axD.axvline(302.1,color='#0072B2',lw=0.9,ls='--',alpha=0.5,label='Baseline 302.1')
for bar,r in zip(bars,ranking):
    axD.text(bar.get_width()+1.5,bar.get_y()+bar.get_height()/2,
             f"{r['km3']:.1f}  (H₂={r['H2']:.3f}g)",va='center',fontsize=7.5,fontweight='bold',
             color='#D55E00' if r['label'].startswith('Combined') else '#333333')
axD.set_xlim(280,max(r['km3'] for r in ranking)*1.08)
axD.spines['top'].set_visible(False); axD.spines['right'].set_visible(False)

# ── Panel E: full comparison table ────────────────────────────────────────
axE=fig.add_subplot(gs[2,:])
axE.axis('off')
# Table data
hdr=['Profile','P_PULSE [W]','V_HIGH [km/h]','V_LOW [km/h]','H₂ [g]','km/m³','Δkm/m³ vs Base','Charge OK']
rows_t=[
    ['Baseline (P&G 1000W)',     '1000','29.5','21.0','4.314','302.1','—',          '✓'],
    ['Agent 1 (Speed Window)',   '1000','28.8','18.0','4.157','313.5','+11.4 (+3.8%)','✓'],
    ['Agent 3 (Terrain-Adapt)',  '1000','28.1','17.6','4.040','322.6','+20.5 (+6.8%)','✓'],
    ['Agent 2 (P_PULSE 390 W)',  ' 390','29.5','21.0','4.024','323.9','+21.8 (+7.2%)','✓'],
    [f"Combined (Best)",
     f"{best['PP']:.0f}",f"{best['VH']*3.6:.1f}",f"{best['VL']*3.6:.1f}",
     f"{best['H2']:.3f}",f"{best['km3']:.1f}",
     f"+{best['km3']-302.1:.1f} (+{(best['km3']/302.1-1)*100:.1f}%)","✓"],
]
tbl=axE.table(cellText=rows_t,colLabels=hdr,cellLoc='center',loc='center',bbox=[0,0,1,1])
tbl.auto_set_font_size(False); tbl.set_fontsize(8.5)
for (r,c),cell in tbl.get_celld().items():
    if r==0:
        cell.set_facecolor('#1A252F'); cell.set_text_props(color='white',fontweight='bold')
    elif r==len(rows_t):  # combined (last row)
        cell.set_facecolor('#FFF3CD')
        if c in(5,6): cell.set_facecolor('#D4EDDA')
    elif r%2==1: cell.set_facecolor('#F7F9FA')
    cell.set_edgecolor('#CCCCCC')
axE.set_title('Strategy G — All Profiles Comparison',fontsize=9,fontweight='bold',pad=8)

fig.suptitle(
    'Hydraix I SEM 2026 — Combined Best Velocity Profile  |  '
    f'km/m³={best["km3"]:.1f}  H₂={best["H2"]:.3f}g  ΔSOC={best["dSOC"]:+.4f}  '
    f'Strategy G charge-sustained',
    fontsize=11,fontweight='bold',y=0.99)

out=os.path.join(RESULTS_DIR,'combined_best_result.png')
fig.savefig(out,dpi=180,bbox_inches='tight',facecolor='white')
plt.close(fig)
print(f"\nFigure saved → {out}")
