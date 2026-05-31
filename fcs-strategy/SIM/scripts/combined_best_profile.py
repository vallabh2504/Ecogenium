"""
combined_best_profile.py — 35-min race constraint, natural coast-to-stop,
4-panel publication figure with multi-strategy EMS comparison.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use('Agg')
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as mgridspec
import matplotlib.colors as mcolors
import matplotlib.cm as mcm
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d, RegularGridInterpolator

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP, LHV_H2, ETA_DCDC,
    SC_C, SC_V_MAX, SC_V_MIN, SC_E_J, SC_ETA, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, ECMS_DENOM, fc_current, fc_h2_rate, sc_soc_update, sc_voltage,
    sc_terminal_voltage, motor_lookup_2d, motor_power_2d, MOTOR_VARIANT, R_WHEEL,
    N_LAPS, DT, MATLAB_DIR, RESULTS_DIR,
)

MASS=180.; G=9.81; CD=0.15; AF=1.35; CRR=0.006; RHO=1.225; ETA_DT=0.95
GRADE_THRESH=0.006; P_RAMP_MAX=200.; V_MAX=14.0
TAU_MOTOR_MAX=35.0   # Nm — BAFANG RM G060.1000 peak wheel torque (LUT ceiling)
V_COAST_STOP=7./3.6   # coast, then brake from 7 km/h (swept optimum: max km/m³;
                      # hard-brake distance ≈0.63 m @ A_HARD. Lower→long crawl forces fast cruise; higher→more KE lost)
A_HARD=3.0
N_STOP_STEPS=round(3./DT)
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
    return D,s,es,gr,interp1d(s,es,fill_value='extrapolate'),interp1d(s,gr,fill_value='extrapolate')

D_LAP,_sr,_es,_gs,_elev_fn,_grade_fn=_load_route()

def _net_force(v,P,g):
    # Tractive force is limited by BOTH available power (P·η/v) AND the motor's
    # peak wheel torque. The BAFANG LUT tops out at TAU_MOTOR_MAX (35 Nm); at the
    # wheel that is TAU_MOTOR_MAX/R_WHEEL ≈ 119 N. The old MASS*2 = 360 N cap
    # (≈106 Nm) was ~3× the motor's real limit and let the FSM fabricate
    # accelerations the motor cannot supply.
    F_max = TAU_MOTOR_MAX / R_WHEEL
    Fm=min(P*ETA_DT/max(v,0.3), F_max) if P>0 else 0.
    return Fm - 0.5*CD*AF*RHO*v**2 - CRR*MASS*G - MASS*G*g

def _coast_dist(v0,s0):
    v=v0; s=s0; dist=0.
    while v>V_COAST_STOP and dist<800.:
        sw=s%D_LAP; grade=float(_grade_fn(sw))
        a=_net_force(v,0.,grade)/MASS
        vn=max(0.,v+a*DT)
        ds=v*DT; dist+=ds; s+=ds; v=vn
        if vn==0. and v0>V_COAST_STOP: break
    return dist

def _hold_power(v,grade):
    # Electrical power to hold speed v against road load (no accel) — used in the
    # terminal zone so the car cruises (no new pulses) right up to the coast point.
    F = 0.5*CD*AF*RHO*v*v + CRR*MASS*G + MASS*G*grade
    return max(0., F*v/ETA_DT)

def _build_lap(V_HI,V_LO,V_DH,P_PU,P_BO,k_grade=0.):
    tl=[];vl=[];Pl=[];sl=[];el=[];gl=[];coastl=[]
    t=v=s=0.; mode='PULSE'; first=True; pP=0.; was_dh=False; term=False
    while True:
        sw=s%D_LAP; grade=float(_grade_fn(sw)); elev=float(_elev_fn(sw))
        # Terrain-adaptive P&G (EMS rec #2): shift the speed band by local grade.
        # k_grade>0 lowers the target speed on climbs (don't fight gravity at high
        # speed → less drag·time and less motor work) and raises it on descents
        # (harvest gravity). k_grade=0 reproduces the fixed-band behaviour.
        V_HI_e=float(np.clip(V_HI-k_grade*grade, V_LO+0.5, V_MAX))
        V_LO_e=float(np.clip(V_LO-k_grade*grade, 3.0,      V_HI_e-0.3))
        rem=D_LAP-sw; Pt=0.; vc=V_MAX
        # Terminal stop logic (no regen → coast, then brake from V_COAST_STOP):
        #   • latch COAST when remaining distance == natural coast distance from v;
        #   • before that, once within the worst-case (V_HI) coast distance, stop
        #     pulsing and HOLD speed so we don't overshoot the line still fast.
        if mode not in ('COAST_TO_STOP','HARD_BRAKE') and v>0 and rem<800.:
            if rem<=_coast_dist(v,s): mode='COAST_TO_STOP'; term=False
            elif rem<=_coast_dist(V_HI_e,s): term=True
        if mode=='COAST_TO_STOP' and v<=V_COAST_STOP: mode='HARD_BRAKE'
        if mode in ('COAST_TO_STOP','HARD_BRAKE'):
            Pt=0.; vc=V_MAX
        elif term:
            mode='CRUISE_HOLD'; Pt=_hold_power(v,grade); vc=v   # hold speed until coast point
        elif grade<-GRADE_THRESH:
            Pt=0.; vc=V_DH; mode='GLIDE'; was_dh=True
        elif grade>+GRADE_THRESH:
            if v>=V_HI_e: mode='GLIDE'; Pt=0.
            else: mode='PULSE'; Pt=P_BO
            vc=V_MAX; was_dh=False
        else:
            vc=V_HI_e+1.
            if was_dh and v>V_LO_e: mode='GLIDE'; Pt=0.
            else:
                was_dh=False
                if v<=V_LO_e: mode='PULSE'; Pt=P_PU
                elif v>=V_HI_e: mode='GLIDE'; Pt=0.
                else: Pt=P_PU if mode=='PULSE' else 0.
        P=float(np.clip(Pt,pP-P_RAMP_MAX,pP+P_RAMP_MAX)); pP=P
        if not first:
            tl.append(t);vl.append(v);Pl.append(P);sl.append(sw)
            el.append(elev);gl.append(grade)
            coastl.append(mode in ('COAST_TO_STOP','HARD_BRAKE'))
        first=False
        if mode=='HARD_BRAKE':
            vn=max(0.,v-A_HARD*DT); sn=s+v*DT
        else:
            a=_net_force(v,P,grade)/MASS
            vn=float(np.clip(v+a*DT,0.,vc)); sn=s+v*DT
        t+=DT; v=vn; s=sn
        # In normal driving the lap ends at the line. In the terminal stop sequence we
        # let the car finish coasting to V_COAST_STOP and brake (stopping a few m past
        # the line) so the kinetic energy dumped to the brakes is minimised (no regen).
        if (s>=D_LAP and mode not in ('COAST_TO_STOP','HARD_BRAKE','CRUISE_HOLD')) \
           or (mode=='HARD_BRAKE' and v==0.) or t>700.: break
    se=sl[-1] if sl else 0.; ee=el[-1] if el else float(_elev_fn(0)); ge=gl[-1] if gl else 0.
    for _ in range(N_STOP_STEPS):
        tl.append(t);vl.append(0.);Pl.append(0.);sl.append(se)
        el.append(ee);gl.append(ge);coastl.append(True);t+=DT
    return tl,vl,Pl,sl,el,gl,coastl

def build_profile(V_HI,V_LO,V_DH,P_PU=500.,P_BO=1000.,k_grade=0.):
    at=[];av=[];aP=[];as_=[];aln=[];ae=[];ag=[];ac=[]; to=0.
    for lap in range(1,N_LAPS+1):
        tl,vl,Pl,sl,el,gl,coastl=_build_lap(V_HI,V_LO,V_DH,P_PU,P_BO,k_grade)
        ta=np.array(tl)+to
        at.append(ta);av.append(np.array(vl));aP.append(np.array(Pl))
        as_.append(np.array(sl));aln.append(np.full(len(tl),lap,dtype=int))
        ae.append(np.array(el));ag.append(np.array(gl))
        ac.append(np.array(coastl,dtype=bool)); to=float(ta[-1])+DT
    ta=np.concatenate(at);va=np.concatenate(av);Pa=np.concatenate(aP)
    sa=np.concatenate(as_);la=np.concatenate(aln);ea=np.concatenate(ae)
    ga=np.concatenate(ag);ca=np.concatenate(ac)
    vs=va.copy(); im=(va>0.)
    ss=np.where(np.diff(np.concatenate([[0],im.astype(int)]))==1)[0]
    se_=np.where(np.diff(np.concatenate([im.astype(int),[0]]))==-1)[0]+1
    for ms,me in zip(ss,se_):
        seg=va[ms:me]
        if len(seg)>25: vs[ms:me]=savgol_filter(seg,21,2)
        vs[ms:me]=np.clip(vs[ms:me],0.,V_MAX)
    return ta,vs,Pa,sa,la,ea,ga,ca

def compute_motor_signals(va, ga, variant=None):
    """Vehicle dynamics → electrical demand via 2D BAFANG LUT.
    Returns P_elec [W], V_eff [V], Torque_Nm [Nm] (arrays, length=len(va)).

    P_elec is the DC bus power drawn by the motor controller, taken from the
    LUT's P_in column for the selected iron-loss variant (default 'v1', the
    most optimistic / lowest-iron-loss model — see ems_core.MOTOR_VARIANT).
    For variant 'v1' this equals I_dc × V_eff exactly. Use variant 'v2'/'v3'
    to study sensitivity to the iron-loss assumption.

    No regenerative braking is modelled: zeros where P_wheel <= 0
    (glide, coast, downhill braking)."""
    accel       = np.gradient(va, DT)
    P_wheel     = (CRR*MASS*G*va + 0.5*CD*AF*RHO*va**3
                   + MASS*accel*va + MASS*G*ga*va)
    P_wheel_pos = np.maximum(P_wheel, 0.)
    v_safe      = np.maximum(va, 0.3)
    Torque_Nm   = (P_wheel_pos / v_safe) * R_WHEEL
    Speed_kmh   = va * 3.6
    _, V_eff, _ = motor_lookup_2d(Speed_kmh, Torque_Nm)
    P_elec      = motor_power_2d(Speed_kmh, Torque_Nm, variant)  # DC bus power [W]
    return P_elec, V_eff, Torque_Nm

def verify(t,v,la,silent=True):
    d=float(np.trapezoid(v,t))/1000.; dur=float(t[-1])/60.
    stops=len(np.where(np.diff((v==0.).astype(int))==1)[0])
    ok=(d>14.5)and(28.<=dur<=35.5)and(stops==11)
    if not silent: print(f"  d={d:.3f}km dur={dur:.1f}min stops={stops} {'OK' if ok else 'FAIL'}")
    return ok,d,dur,stops

# ── FC efficiency ──────────────────────────────────────────────────────────────
def fc_eta(P):
    """Efficiency of LC52.30 at electrical output P [W]."""
    if P < FC_P_MIN: return 0.
    I = float(fc_current(np.array([P]))[0])
    rate = float(fc_h2_rate(np.array([I]))[0])
    return P / (rate * LHV_H2) if rate > 1e-12 else 0.

# Build efficiency table once
_P_ETA_TAB = np.linspace(FC_P_MIN, FC_P_MAX, 500)
_ETA_TAB   = np.array([fc_eta(p) for p in _P_ETA_TAB])
ETA_MAX    = float(np.max(_ETA_TAB))
ETA_MIN    = float(np.min(_ETA_TAB[_ETA_TAB > 0]))
# FC operating point that maximises electrical efficiency (η = P_net / (ṁH2 × LHV))
P_PEAK_ETA = float(_P_ETA_TAB[np.argmax(_ETA_TAB)])
# Upper edge of the high-efficiency FC band (system η ≥ 55 %). Strategy G caps
# its FC command here so the stack stays in its efficient region and the SC
# buffers demand peaks above it — EMS recommendation #3. (Falls back to FC_P_MAX
# if the band can't be resolved.) For the LC 52.30 this is ≈ 388 W.
_BAND_THRESH   = 0.55
_band_mask     = _ETA_TAB >= _BAND_THRESH
FC_ETA_BAND_HI = float(_P_ETA_TAB[_band_mask].max()) if _band_mask.any() else FC_P_MAX

def fc_eta_arr(Pfc_arr):
    return np.interp(Pfc_arr, _P_ETA_TAB, _ETA_TAB, left=0., right=_ETA_TAB[-1])

def physics_power_demand(va, ga):
    """Physics-based electrical power demand from velocity [m/s] and grade [-] arrays.
    Uses the 2D motor LUT to convert wheel power to DC electrical power [W].
    Returns zeros where vehicle is not under power (coast/stop)."""
    accel       = np.gradient(va, DT)
    P_wheel     = (CRR*MASS*G*va + 0.5*CD*AF*RHO*va**3
                   + MASS*accel*va + MASS*G*ga*va)
    P_wheel_pos = np.maximum(P_wheel, 0.)
    v_safe      = np.maximum(va, 0.3)
    Torque_Nm   = (P_wheel_pos / v_safe) * R_WHEEL
    Speed_kmh   = va * 3.6
    I_dc, V_eff, _ = motor_lookup_2d(Speed_kmh, Torque_Nm)
    P_elec      = I_dc * V_eff
    return P_elec

# ── EMS strategies ─────────────────────────────────────────────────────────────
def make_strat_g(K_p,K_i=2.,tau=15.,p_eta_cap=None):
    """LPF feedforward + SOC-PI + lap-offset + always-on FC floor.

    p_eta_cap: soft upper limit on the FC FEEDFORWARD baseline [W] (EMS rec #3 —
    keep the FC in its high-efficiency band, let the SC buffer peaks). The SOC-PI
    correction may still push the command above the cap up to FC_P_MAX, so charge
    sustenance is never sacrificed. Default None → FC_P_MAX (cap OFF): for the
    Hydraix duty cycle the average demand is already inside the η-band, so capping
    only routes peak energy through the SC (≈3 % round-trip loss) with no net gain.
    Pass FC_ETA_BAND_HI (≈388 W) to enable it for low-demand vehicles/tracks.
    """
    alpha=float(np.exp(-DT/tau))
    # FC glide floor = membrane-safe FC_P_MIN (100 W). Must not exceed the race-
    # average FC demand or the SC overcharges; FC_P_MIN is safe for all regimes.
    G_FC_MIN = float(FC_P_MIN)
    cap = float(FC_P_MAX if p_eta_cap is None else p_eta_cap)
    cap = min(cap, FC_P_MAX)
    st={'lpf':None,'integ':0.,'prev_lap':-1,'offset':0.}
    def fn(I_motor,U_sc,Pp,til,li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if st['lpf'] is None: st['lpf']=float(P_motor)
        if li!=st['prev_lap'] and li>0:
            st['offset']+=0.3*(SC_SOC_0-SOC)*SC_E_J/186.
            st['offset']=float(np.clip(st['offset'],-400.,400.))  # wider: needs ~200W more headroom
        st['prev_lap']=li
        st['integ']+=(SC_SOC_0-SOC)*DT
        if K_i>1e-9: st['integ']=float(np.clip(st['integ'],-400./K_i,400./K_i))  # wider: allow ~400W integral correction
        if I_motor<0.1: return G_FC_MIN
        st['lpf']=alpha*st['lpf']+(1-alpha)*float(P_motor)
        # η-band cap (#3) applies to the FEEDFORWARD baseline only. The SOC-PI
        # correction may push beyond it (up to FC_P_MAX) so that on high-demand
        # vehicles the FC can still recharge the SC — charge sustenance is never
        # sacrificed for efficiency. On low-demand vehicles the baseline stays
        # below the cap and the FC sits in its efficient band.
        base=min(st['lpf']+st['offset'], cap)
        Pc=base+K_p*(SC_SOC_0-SOC)+K_i*st['integ']
        return float(np.clip(Pc,G_FC_MIN,FC_P_MAX))
    return fn

def make_strat_constant(P_set):
    """FC at fixed setpoint when motor is on, 0 when motor is off."""
    def fn(I_motor,U_sc,Pp,til,li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if I_motor<0.1: return 0.
        return float(P_set)
    return fn

def make_strat_pi(K_p,K_i=2.):
    """Pure SOC-PI, no LPF feedforward."""
    st={'integ':0.}
    def fn(I_motor,U_sc,Pp,til,li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if I_motor<0.1: return 0.
        st['integ']+=(SC_SOC_0-SOC)*DT
        if K_i>1e-9: st['integ']=float(np.clip(st['integ'],-150./K_i,150./K_i))
        return float(np.clip(K_p*(SC_SOC_0-SOC)+K_i*st['integ'],0.,FC_P_MAX))
    return fn

def make_strat_rule(P_hi,band=0.05):
    """Rule-based hysteresis: high FC when SOC<SOC_ref-band, low when SOC>SOC_ref+band."""
    st={'state':'neutral'}
    def fn(I_motor,U_sc,Pp,til,li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if I_motor<0.1: return 0.
        if SOC<SC_SOC_0-band: st['state']='charge'
        elif SOC>SC_SOC_0+band: st['state']='hold'
        Pout=float(np.clip(P_hi,FC_P_MIN,FC_P_MAX)) if st['state']=='charge' else FC_P_MIN
        return Pout
    return fn

def make_strat_h(P_set):
    """
    Strategy H — Large-SC Optimal Constant Dispatch.

    With the HyCap 91 Wh SC bank the capacitor SOC barely changes per lap, so
    SOC-feedback correction terms (LPF, PI, hysteresis) have near-zero effect.
    The optimal solution is a FIXED FC setpoint that balances energy over the race;
    the large SC absorbs every pulse/glide transient passively.

    P_set: charge-sustaining FC power [W] (found by bisection).
    K_SOFT: small fixed SOC stabiliser (±50 W per 10 % SOC drift).
    """
    K_SOFT = 500.     # gentle disturbance-rejection only; not the primary balance term
    def fn(I_motor,U_sc,Pp,til,li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if I_motor<0.1: return 0.
        P = P_set + K_SOFT*(SC_SOC_0-SOC)
        return float(np.clip(P,FC_P_MIN,FC_P_MAX))
    return fn

def make_strat_a(K_soc, P_base=200.):
    """
    Strategy A — 2D Lookup Table EMS.

    Maps (u = (P_motor − P_fc_prev) / SC_E_J, SOC) → P_fc [W] via bilinear
    interpolation on a 21×18 pre-built table.

    K_soc  : SOC feedback gain [W per unit SOC], bisected for charge sustenance.
    P_base : mean motor-on demand [W], feedforward term (sets FC at SOC=SOC_0).
    K_rate : rate feedforward for |u| > deadband (rarely activates with large SC).
    """
    u_grid   = np.linspace(-0.050, 0.050, 21)
    soc_grid = np.linspace(0.10, 0.95, 18)
    K_rate   = 500.0
    deadband = 0.020
    TABLE = np.zeros((21, 18))
    for i, u_i in enumerate(u_grid):
        for j, soc_j in enumerate(soc_grid):
            delta_soc  = K_soc * (SC_SOC_0 - soc_j)
            delta_rate = (K_rate * (abs(u_i) - deadband) * np.sign(u_i)
                          if abs(u_i) > deadband else 0.0)
            TABLE[i, j] = float(np.clip(P_base + delta_soc + delta_rate,
                                        FC_P_MIN, FC_P_MAX))
    _interp = RegularGridInterpolator(
        (u_grid, soc_grid), TABLE,
        method='linear', bounds_error=False, fill_value=None
    )
    def fn(I_motor, U_sc, Pp, til, li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if I_motor < 0.1:
            return 0.
        u     = float(np.clip((P_motor - Pp) / SC_E_J, u_grid[0], u_grid[-1]))
        soc_c = float(np.clip(SOC, soc_grid[0], soc_grid[-1]))
        return float(_interp([[u, soc_c]])[0])
    return fn

def make_strat_dems(s0, w=0., K_s=4.0, P_lo=150., P_hi=600.):
    """Strategy D-ECMS — Durability-weighted ECMS with damped SOC-PI co-state.

    Per step minimises  H(P_fc) = mH2(P_fc) + s_eff*(P_dem - P_fc*ETA_DCDC)/ECMS_DENOM
                                  + w*c_deg(P_fc, P_fc_prev)
    The equivalence factor is closed-loop on SOC (damped A-ECMS):
        s_eff = clip( s0 + K_s*(SOC0-SOC) + K_si*∫(SOC0-SOC)dt , 0.3, 3.0 )
    The integral (anti-wound) gives charge-sustaining robustness to imperfect/varying
    laps (uses MEASURED demand + SOC, no stored plan); the clamp + modest gains keep
    it off the twitchy ECMS breakeven so it doesn't overshoot like a P-only co-state.
    c_deg is convex (penalises idle <P_lo, high power >P_hi, and ramping): it parks
    the FC at a near-constant high-η / low-stress point — which both smooths the stack
    AND lowers H2 (constant operation has lower mean current than floor+pulse on the
    convex I-P curve). w=0 → plain ECMS. Drop-in fn(...)->P_fc, same interface as G.
    """
    Pg   = np.concatenate([[0.0], np.linspace(FC_P_MIN, FC_P_MAX, 90)])
    mdot = np.where(Pg>=FC_P_MIN, K_H2*fc_current(np.clip(Pg,FC_P_MIN,FC_P_MAX)), 0.0)
    cdeg0= (np.maximum(0.,P_lo-Pg)/100.)**2 + (np.maximum(0.,Pg-P_hi)/100.)**2
    def fn(I_motor,U_sc,Pp,til,li):
        SOC   = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        P_dem = float(I_motor*U_sc)
        s_eff = float(np.clip(s0 + K_s*(SC_SOC_0-SOC), 0.3, 3.0))
        P_sc  = P_dem - Pg*ETA_DCDC
        H     = mdot + s_eff*P_sc/ECMS_DENOM + w*(cdeg0 + ((Pg-Pp)/150.)**2)
        return float(Pg[int(np.argmin(H))])
    return fn

def make_strat_cc(P_set, K_p=3000., K_i=2.):
    """Strategy CC — Constant-Continuous FC at the efficient point + SOC-PI trim.

    Commands a near-CONSTANT FC power P_set the whole race (never returns 0, even
    motor-off — keeps charging the SC), so the FC sits at one high-efficiency point
    instead of G's floor↔pulse swing. On the convex I-P curve, constant operation
    has lower mean current → less H2 for the SAME FC energy (~+6% km/m³). The outer
    SOC-PI (clamped) provides robust charge-sustenance under imperfect/varying laps;
    the 91 Wh SC buffers all pulse/glide transients passively. Drop-in fn(...)->P_fc.
    """
    st={'integ':0.}
    def fn(I_motor,U_sc,Pp,til,li):
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        st['integ'] = float(np.clip(st['integ'] + (SC_SOC_0-SOC)*DT, -300./max(K_i,1e-9), 300./max(K_i,1e-9)))
        return float(np.clip(P_set + K_p*(SC_SOC_0-SOC) + K_i*st['integ'], FC_P_MIN, FC_P_MAX))
    return fn

def durability_metrics(Pfc):
    """FC-stress metrics for a P_fc trace: ramp activity, idle dwell, FC-η, spread."""
    on=Pfc[Pfc>=FC_P_MIN]
    if len(on)==0: return dict(sigma=0.,ramp=0.,idle=0.,eta=0.,pmean=0.)
    dP=np.diff(Pfc)/DT
    return dict(sigma=float(np.std(on)),
                ramp=float(np.mean(np.abs(dP))),
                idle=100.*float(np.mean(on<=FC_P_MIN+5.)),
                eta=float(np.mean(fc_eta_arr(on))*100.),
                pmean=float(np.mean(on)))

def simulate(fn,P_elec_arr,la_arr,coast_arr=None,SOC0=SC_SOC_0):
    """
    fn signature: fn(I_motor [A], U_sc [V], P_fc_prev, t_in_lap, lap_idx) -> P_fc [W]
    P_elec_arr : precomputed electrical demand = I_dc_LUT × V_eff (from compute_motor_signals)
    I_motor and U_sc are derived each step from P_elec and SOC, matching Simulink signals.
    """
    n=len(P_elec_arr)
    Pfc=np.empty(n);Psc=np.empty(n);SOCa=np.empty(n+1)
    Ifc=np.empty(n);mH2=np.empty(n)
    laps0=np.unique(la_arr)
    lt0={int(l):np.where(la_arr==l)[0][0] for l in laps0}
    SOCa[0]=SOC0; pp=FC_P_MIN
    v_min = float(SC_V_MIN)
    for k in range(n):
        s=SOCa[k]; P_elec=float(P_elec_arr[k]); li=int(la_arr[k])-1
        til=float((k-lt0[li+1])*DT)
        # Derive I_motor and U_sc: actual bus current = P_elec / U_bus (2-step ESR correction)
        U_oc = float(sc_voltage(s))                          # open-circuit voltage
        I_m  = P_elec / max(U_oc, v_min)                    # first estimate of bus current
        U_sc = sc_terminal_voltage(s, I_m)                  # terminal voltage with ESR sag
        I_m  = P_elec / max(U_sc, v_min)                    # corrected bus current
        P_demand = P_elec                                    # actual electrical demand [W]
        Pc=float(fn(I_m,U_sc,pp,til,li))
        Pc=float(np.clip(Pc,pp-FC_RAMP*DT,pp+FC_RAMP*DT))
        is_coast=(bool(coast_arr[k]) if coast_arr is not None else I_m<0.1)
        fc_min=0. if is_coast else FC_P_MIN
        Pc=float(np.clip(Pc,fc_min,FC_P_MAX))
        # Pc = FC NET output (stack side, sets current/H2). The boost converter
        # delivers Pc × ETA_DCDC to the bus, so the SC covers the remainder.
        Psk=P_demand-Pc*ETA_DCDC
        ts=sc_soc_update(s,Psk,DT)
        if Psk>0 and ts<=SC_SOC_MIN:
            Pc=float(np.clip(P_demand/ETA_DCDC,FC_P_MIN,FC_P_MAX));Psk=max(0.,P_demand-Pc*ETA_DCDC)
            ts=sc_soc_update(s,Psk,DT)
        Pfc[k]=Pc;Psc[k]=Psk;SOCa[k+1]=ts
        Ifc[k]=float(fc_current(Pc)) if Pc>=FC_P_MIN else 0.
        mH2[k]=fc_h2_rate(Ifc[k])*DT; pp=Pc
    m_H2=float(np.sum(mH2)); dSOC=float(SOCa[n]-SOC0)
    E_fc_J=float(np.sum(Pfc)*DT)
    m_H2_norm=_normalize_h2(m_H2,dSOC,E_fc_J)
    return {'m_H2':m_H2,'m_H2_norm':m_H2_norm,'dSOC':dSOC,'E_fc_J':E_fc_J,
            'SOC':SOCa[:-1],'Pfc':Pfc,'Psc':Psc}

def _normalize_h2(m_H2, dSOC, E_fc_J):
    """Charge-sustaining-normalised H2 [g]: remove (or add back) the H2 that
    produced the residual SC energy change, so strategies at slightly different
    end-SOC are compared on an equal-energy basis (dSOC=0).

    The SC stored-energy change dSOC×E_sc was deposited from the bus, which the
    FC supplied through the boost converter and (charge-side) SC efficiency.
    Convert that FC-net energy to H2 at the run's average H2 intensity
    (m_H2 / E_fc_J) and subtract it.
    """
    if E_fc_J <= 0.:
        return m_H2
    eta_sc_ow = SC_ETA ** 0.5
    dE_fc_J   = dSOC * SC_E_J / (eta_sc_ow * ETA_DCDC)   # FC-net J behind the dSOC
    return m_H2 - (dE_fc_J / E_fc_J) * m_H2

def km_per_m3(result, total_km=TOTAL_KM, h2_density=H2_DENSITY):
    """km/m³ from the charge-sustaining-normalised H2 (preferred for ranking)."""
    return total_km / (result['m_H2_norm'] / h2_density)

def _bisect_param(make_fn,lo,hi,P_e,la,ca,param_name='param'):
    best=None
    p=(lo+hi)/2.
    for it in range(25):
        fn=make_fn(p); r=simulate(fn,P_e,la,ca)
        ds=r['dSOC']
        print(f"    iter{it+1:2d}  {param_name}={p:7.1f}  dSOC={ds:+.4f}  H2={r['m_H2']:.3f}g")
        if abs(ds)<=0.015: best=r; break
        if ds<-0.015: lo=p
        else: hi=p
        if hi-lo<0.5: break
        p=(lo+hi)/2.; best=r
    if best is None: best=r
    return p,best

def bisect_kp(P_e,la,ca,label=''):
    print(f"\n  Strategy G bisect K_p — '{label}'")
    return _bisect_param(lambda kp: make_strat_g(kp), 100.,8000.,P_e,la,ca,'K_p')

def bisect_const(P_e,la,ca):
    print(f"\n  Constant FC bisect P_set")
    return _bisect_param(lambda ps: make_strat_constant(ps), FC_P_MIN,FC_P_MAX,P_e,la,ca,'P_set')

def bisect_pi(P_e,la,ca):
    print(f"\n  PI-only bisect K_p")
    return _bisect_param(lambda kp: make_strat_pi(kp), 100.,3000.,P_e,la,ca,'K_p')

def bisect_rule(P_e,la,ca):
    print(f"\n  Rule-based bisect P_hi")
    return _bisect_param(lambda ph: make_strat_rule(ph), FC_P_MIN,FC_P_MAX,P_e,la,ca,'P_hi')

def bisect_h(P_e,la,ca):
    print(f"\n  Strategy H bisect P_set — large-SC optimal constant dispatch "
          f"(FC peak-η at {P_PEAK_ETA:.0f}W = {fc_eta(P_PEAK_ETA)*100:.1f}%)")
    return _bisect_param(make_strat_h, FC_P_MIN,FC_P_MAX,P_e,la,ca,'P_set')

def bisect_a(P_e,la,ca):
    print(f"\n  Strategy A bisect K_soc (2D LUT)")
    # P_base must account for the FC_P_MIN=100W floor that simulate() applies
    # during glide phases (ca=False, I_motor≈0): energy balance requires
    #   P_base × T_motor_on + FC_P_MIN × T_floor = E_motor_total
    ca_bool = ca.astype(bool)
    T_motor_on = float(np.sum(P_e > 10.) * DT)
    T_floor    = float(np.sum((P_e <= 10.) & ~ca_bool) * DT)
    E_motor    = float(np.sum(P_e) * DT)
    P_base     = float(np.clip((E_motor - FC_P_MIN * T_floor) / max(T_motor_on, 1.),
                               FC_P_MIN, FC_P_MAX))
    print(f"    P_base={P_base:.1f}W  (motor-on {T_motor_on:.0f}s, glide-floor {T_floor:.0f}s × {FC_P_MIN:.0f}W)")
    return _bisect_param(lambda ks: make_strat_a(ks, P_base), 50.,600.,P_e,la,ca,'K_soc')

# ── Grid search — target ≤ 35 min ─────────────────────────────────────────────
if __name__ == '__main__':
    print("=== Combined Best Profile — 35-min constraint grid search ===")
    # High-drag vehicle, physical 35 Nm torque cap, AND glide-to-~1km/h stops (no
    # regen). The slow final crawl adds ~2.5 min/race, so the car must cruise faster
    # to stay under 35.5 min: feasible band is now VH≈11–12 m/s with high pulse power.
    # Brake-from-7km/h (swept optimum). Shorter coast than glide-to-1km/h, so the
    # feasible cruise band drops back to VH≈9.5–10.5 m/s.
    V_HI_vals = [9.5, 10.0, 10.5]
    V_LO_vals = [6.0, 6.5, 7.0]
    P_PU_vals = [1200., 1400.]
    V_DH      = 10.5

    results = []
    for VH in V_HI_vals:
        for VL in V_LO_vals:
            if VL >= VH-1.5: continue
            for PP in P_PU_vals:
                ta,va,Pa,sa,la,ea,ga,ca = build_profile(VH,VL,V_DH,PP,1200.)
                ok,d,dur,stops = verify(ta,va,la)
                if not ok:
                    print(f"  SKIP VH={VH} VL={VL} PP={PP}W  ({d:.2f}km {dur:.1f}min {stops}stops)")
                    continue
                P_e, V_eff, _ = compute_motor_signals(va, ga)
                Kp,r = bisect_kp(P_e,la,ca,f"VH={VH} VL={VL} PP={PP}W")
                km3  = km_per_m3(r)
                cs   = abs(r['dSOC'])<=0.015
                results.append(dict(VH=VH,VL=VL,PP=PP,Kp=Kp,k_grade=0.,H2=r['m_H2'],
                                    H2n=r['m_H2_norm'],E_fc_J=r['E_fc_J'],km3=km3,
                                    dSOC=r['dSOC'],cs=cs,
                                    ta=ta,va=va,Pa=Pa,sa=sa,la=la,ea=ea,ga=ga,ca=ca,
                                    SOC=r['SOC'],Pfc=r['Pfc'],Psc=r['Psc']))
                print(f"  >>> VH={VH} VL={VL} PP={PP}W  km3={km3:.1f}  H2={r['m_H2']:.3f}g  dSOC={r['dSOC']:+.4f}  CS={'YES' if cs else 'NO'}")

    if not results:
        raise RuntimeError("All profiles failed verify. Adjust grid or verify bounds.")

    # Prefer charge-sustaining profiles; only fall back to all if none qualify.
    _cs = [r for r in results if r['cs']]
    if not _cs:
        print("\n  WARNING: no charge-sustaining flat profile found — ranking among all.")
    best_flat = max(_cs or results, key=lambda x: x['km3'])

    # ── Terrain-adaptive P&G sweep (EMS rec #2) on the best flat profile ────────
    print(f"\n=== Terrain-adaptive P&G sweep (k_grade) on best flat profile ===")
    VHb,VLb,PPb = best_flat['VH'],best_flat['VL'],best_flat['PP']
    for kg in [20.,40.,60.,80.,120.]:
        ta,va,Pa,sa,la,ea,ga,ca = build_profile(VHb,VLb,V_DH,PPb,1200.,k_grade=kg)
        ok,d,dur,stops = verify(ta,va,la)
        if not ok:
            print(f"  SKIP k_grade={kg:.0f}  ({d:.2f}km {dur:.1f}min {stops}stops)")
            continue
        P_e,_,_ = compute_motor_signals(va,ga)
        Kp,r = bisect_kp(P_e,la,ca,f"k_grade={kg:.0f}")
        km3 = km_per_m3(r); cs = abs(r['dSOC'])<=0.015
        print(f"  >>> k_grade={kg:.0f}  km/m³={km3:.1f}  H2={r['m_H2']:.3f}g  dSOC={r['dSOC']:+.4f}  CS={'YES' if cs else 'NO'}")
        if cs:
            results.append(dict(VH=VHb,VL=VLb,PP=PPb,Kp=Kp,k_grade=kg,H2=r['m_H2'],
                                H2n=r['m_H2_norm'],E_fc_J=r['E_fc_J'],km3=km3,
                                dSOC=r['dSOC'],cs=cs,
                                ta=ta,va=va,Pa=Pa,sa=sa,la=la,ea=ea,ga=ga,ca=ca,
                                SOC=r['SOC'],Pfc=r['Pfc'],Psc=r['Psc']))

    _cs_all = [r for r in results if r['cs']]
    best = max(_cs_all or results, key=lambda x: x['km3'])
    _,dur_best,_,_ = verify(best['ta'],best['va'],best['la'],silent=False)
    print(f"\n=== COMBINED BEST ===")
    print(f"  V_HIGH={best['VH']} m/s ({best['VH']*3.6:.1f}km/h)  V_LOW={best['VL']} m/s  PP={best['PP']}W  k_grade={best.get('k_grade',0.):.0f}")
    print(f"  K_p={best['Kp']:.1f}  H2(raw)={best['H2']:.3f}g  H2(CS-norm)={best['H2n']:.3f}g  "
          f"km/m³={best['km3']:.1f}  dSOC={best['dSOC']:+.4f}")
    print(f"  (FC→bus converter η={ETA_DCDC:.2f}, SC round-trip η={SC_ETA:.2f}, motor iron-loss '{MOTOR_VARIANT}')")

    # ── Motor iron-loss sensitivity band (v1 optimistic … v3 pessimistic) ──────
    print(f"\n=== Motor iron-loss sensitivity (Strategy G on best profile) ===")
    for var in ('v1', 'v2', 'v3'):
        P_e_v, _, _ = compute_motor_signals(best['va'], best['ga'], variant=var)
        _, r_v = bisect_kp(P_e_v, best['la'], best['ca'], f"iron-loss {var}")
        print(f"  {var}: km/m³={km_per_m3(r_v):6.1f}  H2(norm)={r_v['m_H2_norm']:.3f}g  "
              f"dSOC={r_v['dSOC']:+.4f}")

    # ── EMS rec #3: FC η-band cap comparison on the best profile ───────────────
    print(f"\n=== EMS rec #3: FC η-band cap comparison (Strategy G, best profile) ===")
    P_e_cap,_,_ = compute_motor_signals(best['va'], best['ga'])
    for label, capv in [('cap OFF (default)', FC_P_MAX),
                        (f'cap ON ({FC_ETA_BAND_HI:.0f} W)', FC_ETA_BAND_HI)]:
        _, rc = _bisect_param(lambda kp: make_strat_g(kp, p_eta_cap=capv),
                              100., 8000., P_e_cap, best['la'], best['ca'], 'K_p')
        print(f"  {label}: km/m³={km_per_m3(rc):.1f}  dSOC={rc['dSOC']:+.4f}  "
              f"CS={abs(rc['dSOC'])<=0.015}")

    # ── Save CSV ───────────────────────────────────────────────────────────────────
    df_out=pd.DataFrame({'time_s':best['ta'],'velocity_ms':best['va'],
        'velocity_kmh':best['va']*3.6,'dist_in_lap_m':best['sa'],
        'lap_num':best['la'],'elevation_m':best['ea'],'grade':best['ga'],
        'P_elec_W':best['Pa']})
    csv_out=os.path.join(MATLAB_DIR,'sem_combined_best.csv')
    df_out.to_csv(csv_out,index=False)
    print(f"  CSV → {csv_out}")

    # ── Multi-strategy comparison on best velocity profile ─────────────────────────
    print("\n=== Running additional EMS strategies on best velocity profile ===")
    la_b=best['la']; ca_b=best['ca']
    P_e_b, V_eff_b, _ = compute_motor_signals(best['va'], best['ga'])

    Kp_g=best['Kp']; r_g={'m_H2':best['H2'],'m_H2_norm':best['H2n'],
                           'E_fc_J':best['E_fc_J'],'dSOC':best['dSOC'],
                           'SOC':best['SOC'],'Pfc':best['Pfc'],'Psc':best['Psc']}
    km3_g=best['km3']

    Ps_c, r_c = bisect_const(P_e_b,la_b,ca_b)
    km3_c = km_per_m3(r_c)
    print(f"  Constant FC  P_set={Ps_c:.0f}W  H2={r_c['m_H2']:.3f}g  km/m³={km3_c:.1f}  dSOC={r_c['dSOC']:+.4f}")

    Kp_pi, r_pi = bisect_pi(P_e_b,la_b,ca_b)
    km3_pi = km_per_m3(r_pi)
    print(f"  PI-only  K_p={Kp_pi:.1f}  H2={r_pi['m_H2']:.3f}g  km/m³={km3_pi:.1f}  dSOC={r_pi['dSOC']:+.4f}")

    Ph_r, r_r = bisect_rule(P_e_b,la_b,ca_b)
    km3_r = km_per_m3(r_r)
    print(f"  Rule-based  P_hi={Ph_r:.0f}W  H2={r_r['m_H2']:.3f}g  km/m³={km3_r:.1f}  dSOC={r_r['dSOC']:+.4f}")

    Ps_h, r_h = bisect_h(P_e_b,la_b,ca_b)
    km3_h = km_per_m3(r_h)
    print(f"  Strategy H  P_set={Ps_h:.0f}W  H2={r_h['m_H2']:.3f}g  km/m³={km3_h:.1f}  dSOC={r_h['dSOC']:+.4f}  "
          f"(FC peak-η at {P_PEAK_ETA:.0f}W, η={fc_eta(P_PEAK_ETA)*100:.1f}%)")

    Ks_a, r_a = bisect_a(P_e_b,la_b,ca_b)
    km3_a = km_per_m3(r_a)
    print(f"  Strategy A  K_soc={Ks_a:.1f}  H2={r_a['m_H2']:.3f}g  km/m³={km3_a:.1f}  dSOC={r_a['dSOC']:+.4f}")

    # H2 shown is charge-sustaining-normalised (m_H2_norm) so the H₂ and km/m³
    # columns are mutually consistent and the ranking is dSOC-fair.
    strategies = [
        dict(name='Strategy H\n(Large-SC Fixed-P)', param=f'P={Ps_h:.0f}W',
             H2=r_h['m_H2_norm'], km3=km3_h, dSOC=r_h['dSOC'],
             sc_swing=float(np.max(r_h['SOC'])-np.min(r_h['SOC'])),
             cs=abs(r_h['dSOC'])<=0.015, color='#56B4E9', Pfc=r_h['Pfc'], SOC=r_h['SOC']),
        dict(name='Strategy G\n(LPF+SOC-PI)', param=f'K_p={Kp_g:.0f}',
             H2=r_g['m_H2_norm'], km3=km3_g, dSOC=r_g['dSOC'],
             sc_swing=float(np.max(r_g['SOC'])-np.min(r_g['SOC'])),
             cs=abs(r_g['dSOC'])<=0.015, color='#D55E00', Pfc=r_g['Pfc'], SOC=r_g['SOC']),
        dict(name='Constant FC\n(Fixed P_FC)', param=f'P={Ps_c:.0f}W',
             H2=r_c['m_H2_norm'], km3=km3_c, dSOC=r_c['dSOC'],
             sc_swing=float(np.max(r_c['SOC'])-np.min(r_c['SOC'])),
             cs=abs(r_c['dSOC'])<=0.015, color='#CC79A7', Pfc=r_c['Pfc'], SOC=r_c['SOC']),
        dict(name='Rule-Based\n(Hysteresis)', param=f'P_hi={Ph_r:.0f}W',
             H2=r_r['m_H2_norm'], km3=km3_r, dSOC=r_r['dSOC'],
             sc_swing=float(np.max(r_r['SOC'])-np.min(r_r['SOC'])),
             cs=abs(r_r['dSOC'])<=0.015, color='#009E73', Pfc=r_r['Pfc'], SOC=r_r['SOC']),
        dict(name='PI Only\n(SOC-PI)', param=f'K_p={Kp_pi:.0f}',
             H2=r_pi['m_H2_norm'], km3=km3_pi, dSOC=r_pi['dSOC'],
             sc_swing=float(np.max(r_pi['SOC'])-np.min(r_pi['SOC'])),
             cs=abs(r_pi['dSOC'])<=0.015, color='#0072B2', Pfc=r_pi['Pfc'], SOC=r_pi['SOC']),
        dict(name='Strategy A\n(2D LUT)', param=f'K_soc={Ks_a:.0f}',
             H2=r_a['m_H2_norm'], km3=km3_a, dSOC=r_a['dSOC'],
             sc_swing=float(np.max(r_a['SOC'])-np.min(r_a['SOC'])),
             cs=abs(r_a['dSOC'])<=0.015, color='#E69F00', Pfc=r_a['Pfc'], SOC=r_a['SOC']),
    ]

    # ── Figure ─────────────────────────────────────────────────────────────────────
    # Lap 1 mask
    m1   = best['la']==1
    t1   = best['ta'][m1]-best['ta'][m1][0]
    v1   = best['va'][m1]*3.6
    P1   = best['Pa'][m1]
    g1   = best['ga'][m1]
    e1   = best['ea'][m1]
    ca1  = best['ca'][m1]
    Pfc1 = best['Pfc'][m1]

    # Segment colours
    GRADE_THRESH=0.006
    def seg_col(P,g,v,is_coast):
        if v==0.: return '#222222'
        if is_coast: return '#9467BD'
        if g<-GRADE_THRESH: return '#0072B2'
        if g>+GRADE_THRESH: return '#CC3311'
        return '#EE7733' if P>10. else '#009E73'

    cols1=[seg_col(P1[i],g1[i],v1[i],bool(ca1[i])) for i in range(len(v1))]

    # FC efficiency colourmap (red→green)
    _cmap_rg = mcolors.LinearSegmentedColormap.from_list('rg',['#D62728','#FF7F0E','#2CA02C'])
    _norm_eta = mcolors.Normalize(vmin=ETA_MIN*0.9, vmax=ETA_MAX)

    def eta_color(P):
        e = fc_eta(P)
        return _cmap_rg(_norm_eta(e)) if P>=FC_P_MIN else (0.6,0.6,0.6,0.4)

    fig=plt.figure(figsize=(18,24),dpi=150)
    fig.patch.set_facecolor('white')
    gs=mgridspec.GridSpec(4,1,figure=fig,hspace=0.52,
        height_ratios=[2.2,1.6,2.0,1.8])

    # ══════════════════════════════════════════════════════════════════════════════
    # Panel 1 — Lap 1 detail: velocity+elevation (top) + motor power (bottom)
    # ══════════════════════════════════════════════════════════════════════════════
    gs1=mgridspec.GridSpecFromSubplotSpec(2,1,subplot_spec=gs[0],hspace=0.08,
        height_ratios=[1.6,1.0])

    ax1v=fig.add_subplot(gs1[0])   # velocity + elevation
    ax1e=ax1v.twinx()              # elevation right axis
    ax1p=fig.add_subplot(gs1[1],sharex=ax1v)  # motor power

    # Velocity coloured by mode
    for i in range(len(t1)-1):
        ax1v.plot(t1[i:i+2],v1[i:i+2],color=cols1[i],lw=1.8)
    ax1v.axhline(best['VH']*3.6,color='grey',lw=0.9,ls='--',alpha=0.5,label=f"V_HI={best['VH']*3.6:.1f}")
    ax1v.axhline(best['VL']*3.6,color='grey',lw=0.9,ls=':',alpha=0.5,label=f"V_LO={best['VL']*3.6:.1f}")
    ax1v.set_ylabel('Velocity [km/h]',fontsize=10)
    ax1v.set_title(
        f"Lap 1 — Velocity, Elevation & Motor Power  |  "
        f"V_HI={best['VH']*3.6:.1f} km/h  V_LO={best['VL']*3.6:.1f} km/h  "
        f"P_PULSE={best['PP']:.0f} W  V_DH={V_DH*3.6:.1f} km/h",
        fontsize=10,fontweight='bold')
    # Elevation fill on right axis
    ax1e.fill_between(t1,e1,np.min(e1),color='#BB3311',alpha=0.12)
    ax1e.plot(t1,e1,color='#BB3311',lw=0.9,alpha=0.55)
    ax1e.set_ylabel('Elevation [m]',fontsize=9,color='#BB3311')
    ax1e.tick_params(axis='y',labelcolor='#BB3311',labelsize=8)
    ax1v.set_zorder(ax1e.get_zorder()+1); ax1v.patch.set_visible(False)
    patches_leg=[
        mpatches.Patch(color='#CC3311',label='Uphill boost'),
        mpatches.Patch(color='#0072B2',label='Downhill glide'),
        mpatches.Patch(color='#EE7733',label='Flat pulse'),
        mpatches.Patch(color='#009E73',label='Flat glide'),
        mpatches.Patch(color='#9467BD',label='Coast-to-stop'),
        mpatches.Patch(color='#222222',label='Stop')]
    ax1v.legend(handles=patches_leg,fontsize=8,loc='upper right',ncol=3,framealpha=0.9)
    plt.setp(ax1v.get_xticklabels(),visible=False)

    # Motor power (electrical demand) vs time
    ax1p.fill_between(t1,P1,0,where=(~ca1),color='#EE7733',alpha=0.55,label='Motor on')
    ax1p.fill_between(t1,P1,0,where=ca1,color='#9467BD',alpha=0.4,label='Coast/stop')
    ax1p.axhline(FC_P_MIN,color='#009E73',lw=0.8,ls='--',alpha=0.7,label=f'FC_MIN={FC_P_MIN:.0f}W')
    ax1p.set_xlabel('Time in lap [s]',fontsize=10)
    ax1p.set_ylabel('Motor power = I_motor × U_sc [W]',fontsize=10)
    ax1p.legend(fontsize=8,loc='upper right',framealpha=0.9)
    ax1p.set_xlim(t1[0],t1[-1])
    ax1p.spines['top'].set_visible(False); ax1p.spines['right'].set_visible(False)

    # ══════════════════════════════════════════════════════════════════════════════
    # Panel 2 — EMS strategy comparison table
    # ══════════════════════════════════════════════════════════════════════════════
    ax2=fig.add_subplot(gs[1])
    ax2.axis('off')
    hdr=['EMS Strategy','Tuning','H₂ [g]','km/m³',
         'Δkm/m³ vs H','ΔSOC','SC Swing','CS?']
    ref_km3=km3_h
    rows_t=[]
    for s in strategies:
        delta=s['km3']-ref_km3
        sgn='+' if delta>=0 else ''
        rows_t.append([
            s['name'], s['param'],
            f"{s['H2']:.3f}",
            f"{s['km3']:.1f}",
            f"{sgn}{delta:.1f}" if abs(delta)>0.05 else '—',
            f"{s['dSOC']:+.4f}",
            f"{s['sc_swing']:.3f}",
            '✓' if s['cs'] else '✗',
        ])
    tbl=ax2.table(cellText=rows_t,colLabels=hdr,cellLoc='center',loc='center',
                  bbox=[0,0,1,1])
    tbl.auto_set_font_size(False); tbl.set_fontsize(9.5)
    col_widths=[0.20,0.12,0.09,0.09,0.13,0.10,0.10,0.07]
    for (r,c),cell in tbl.get_celld().items():
        cell.set_width(col_widths[c] if c<len(col_widths) else 0.1)
        if r==0:
            cell.set_facecolor('#1A252F'); cell.set_text_props(color='white',fontweight='bold')
        elif r==1:  # Strategy H (new large-SC best, first row)
            cell.set_facecolor('#E3F2FD')
            if c in(3,4): cell.set_facecolor('#D4EDDA')
        elif r%2==0: cell.set_facecolor('#F7F9FA')
        cell.set_edgecolor('#CCCCCC')
    ax2.set_title(
        f'EMS Strategy Comparison — Same Velocity Profile '
        f'(VH={best["VH"]*3.6:.1f} km/h, VL={best["VL"]*3.6:.1f} km/h, PP={best["PP"]:.0f}W)  '
        f'SC: HyCap 20P×16S 3.8V/250F = {SC_E_J/3600:.0f} Wh',
        fontsize=10,fontweight='bold',pad=10)

    # ══════════════════════════════════════════════════════════════════════════════
    # Panel 3 — Physics-based road load + FC efficiency (Lap 1)
    # ══════════════════════════════════════════════════════════════════════════════
    ax3=fig.add_subplot(gs[2])

    # Road-load power computed from actual v(t) and grade(t) — this is what
    # vehicle dynamics truly demand to maintain current speed at each instant.
    va1   = best['va'][m1]          # velocity [m/s]
    ga1   = best['ga'][m1]          # grade [-]
    P_rl1 = physics_power_demand(va1, ga1)   # physics-based [W electrical]
    Psc1  = best['Psc'][m1]

    # Road-load power (smooth, velocity-dependent) — the physical truth
    ax3.fill_between(t1, P_rl1, 0, alpha=0.18, color='#222222',
                     label='Vehicle dynamics $P_{elec}(v,a,g)$\n(rolling+aero+accel+grad+motor η)')
    ax3.plot(t1, P_rl1, color='#222222', lw=1.3, alpha=0.8)

    # Commanded motor electrical power (P&G step — what EMS actually sees as demand)
    ax3.plot(t1, P1, color='#888888', lw=1.0, ls='--', alpha=0.7, label='Commanded $P_d$ (P&G step)')

    # SC discharge (+) / charge (−) shading
    ax3.fill_between(t1, Psc1, 0, where=(Psc1>0), color='#0072B2', alpha=0.22, label='SC discharge')
    ax3.fill_between(t1, Psc1, 0, where=(Psc1<0), color='#CC79A7', alpha=0.22, label='SC charge (FC surplus)')

    # FC power colour-coded by efficiency (red=low η → green=peak η)
    for i in range(len(t1)-1):
        c = eta_color(Pfc1[i])
        ax3.plot(t1[i:i+2], Pfc1[i:i+2], color=c, lw=2.4)

    # Colourbar
    sm=mcm.ScalarMappable(cmap=_cmap_rg,norm=_norm_eta)
    sm.set_array([])
    cbar=fig.colorbar(sm,ax=ax3,orientation='vertical',fraction=0.018,pad=0.01)
    cbar.set_label('FC efficiency η [—]',fontsize=9)
    cbar.ax.tick_params(labelsize=8)

    ax3.axhline(FC_P_MIN,color='#009E73',lw=0.8,ls=':',alpha=0.7,label=f'FC_MIN={FC_P_MIN:.0f}W')
    ax3.axhline(0,color='#333333',lw=0.6,ls='-',alpha=0.3)
    ax3.set_xlabel('Time in lap [s]',fontsize=10)
    ax3.set_ylabel('Power [W]',fontsize=10)
    ax3.set_title(
        'Power Breakdown — Lap 1  |  Vehicle-dynamics $P_{elec}(v,a,g)$ (rolling+aero+accel+grad+motor η) '
        'vs commanded $P_d$ (P&G)  |  FC colour = efficiency',
        fontsize=9,fontweight='bold')
    ax3.legend(fontsize=8,loc='upper right',framealpha=0.9,ncol=2)
    ax3.set_xlim(t1[0],t1[-1])
    ax3.spines['top'].set_visible(False); ax3.spines['right'].set_visible(False)

    eta1=fc_eta_arr(Pfc1); eta_on=eta1[Pfc1>=FC_P_MIN]
    if len(eta_on):
        ax3.annotate(f"FC η: {eta_on.min()*100:.1f}–{eta_on.max()*100:.1f}%",
                     xy=(0.02,0.93),xycoords='axes fraction',fontsize=9,
                     color='#333333',fontweight='bold')

    # ══════════════════════════════════════════════════════════════════════════════
    # Panel 4 — Full-race motor power (road-load) with FC overlay, all 11 laps
    # ══════════════════════════════════════════════════════════════════════════════
    ax4=fig.add_subplot(gs[3])

    ta_min  = best['ta']/60.
    ca_b_   = best['ca']
    P_rl_all = physics_power_demand(best['va'], best['ga'])   # physics, full race

    ax4.fill_between(ta_min, P_rl_all, 0, color='#222222', alpha=0.15,
                     label='Vehicle dynamics $P_{elec}(v,a,g)$')
    ax4.plot(ta_min, P_rl_all, color='#333333', lw=0.9, alpha=0.6)

    # Commanded motor power (P&G square wave)
    ax4.fill_between(ta_min, best['Pa'], 0, where=(~ca_b_), color='#EE7733', alpha=0.45, label='Motor cmd $P_d$ (on)')
    ax4.fill_between(ta_min, best['Pa'], 0, where=ca_b_,   color='#9467BD', alpha=0.35, label='Coast/stop')

    # FC power (Strategy H — large-SC peak-η)
    ax4.plot(ta_min, r_h['Pfc'], color='#56B4E9', lw=0.9, alpha=0.85, label='FC output (Strat H, large-SC)')

    # Lap markers
    for lap in range(1,N_LAPS+1):
        idx=np.where(best['la']==lap)[0][0]
        lt=best['ta'][idx]/60.
        ax4.axvline(lt,color='#444444',lw=0.6,ls=':',alpha=0.4)
        ax4.text(lt+0.05,max(P_rl_all)*1.02,f'L{lap}',fontsize=7,color='#555555',va='bottom')

    ax4.set_xlabel('Race time [min]',fontsize=10)
    ax4.set_ylabel('Power [W]',fontsize=10)
    ax4.set_title(
        f'Full-Race Power — Road-load (physics) vs Motor command (P&G) vs FC output  '
        f'({best["ta"][-1]/60.:.1f} min, 11 laps)',
        fontsize=10,fontweight='bold')
    ax4.legend(fontsize=8.5,loc='upper right',framealpha=0.9,ncol=2)
    ax4.set_xlim(0,best['ta'][-1]/60.)
    ax4.spines['top'].set_visible(False); ax4.spines['right'].set_visible(False)

    fig.suptitle(
        f'Hydraix I SEM 2026 — Best Velocity Profile  |  '
        f'Strategy H km/m³={km3_h:.1f}  H₂(CS-norm)={r_h["m_H2_norm"]:.3f}g  ΔSOC={r_h["dSOC"]:+.4f}  '
        f'Race: {best["ta"][-1]/60.:.1f} min  |  SC: HyCap 20P×16S {SC_E_J/3600:.0f} Wh',
        fontsize=12,fontweight='bold',y=0.995)

    out=os.path.join(RESULTS_DIR,'combined_best_result.png')
    fig.savefig(out,dpi=150,bbox_inches='tight',facecolor='white')
    plt.close(fig)
    print(f"\nFigure → {out}")
