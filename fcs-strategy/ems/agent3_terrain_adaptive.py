"""
agent3_terrain_adaptive.py — Terrain-Adaptive Velocity Profile for Hydraix I SEM 2026.

Fundamentally different from standard P&G: segment-specific operating modes based
on real route gradient, with parameter sweep to maximise km/m³ fuel economy.

Approach:
  - Flat  (|grade| <= GRADE_THRESH): Standard P&G with tunable V_HIGH_FLAT / V_LOW_FLAT
  - Downhill (grade < -GRADE_THRESH): Motor OFF, let gravity accelerate, cap at V_MAX_DOWNHILL
  - Uphill   (grade > +GRADE_THRESH): Full 1000 W always (no glide on uphill)
  - After downhill: if v > V_LOW_FLAT already, skip straight to glide (harvest free KE)

Baseline to beat: Strategy G on elevation-aware P&G → 302.1 km/m³, H2 = 4.314 g
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_E_J, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, fc_current, fc_h2_rate, sc_soc_update, motor_eta,
    N_LAPS, DT, MATLAB_DIR, RESULTS_DIR,
)

# ── Constants ─────────────────────────────────────────────────────────────────
MASS         = 175.0
G            = 9.81
CD           = 0.15
AF           = 0.8
CRR          = 0.006
RHO          = 1.225
ETA_DT       = 0.95
GRADE_THRESH = 0.006
P_RAMP_MAX   = 200.0   # W/step (per DT=0.2s)
A_STOP       = 0.5     # m/s²  gentle deceleration at lap end
V_MAX        = 11.0    # m/s   hard speed cap (39.6 km/h)
V_STALL      = 10.0 / 3.6  # ≈ 2.78 m/s  (10 km/h) — keep motor on regardless
N_STOP_STEPS = round(4.0 / DT)   # 4 s stop = 20 steps

H2_DENSITY  = 89.88   # g/m³ at STP
TOTAL_KM    = 14.5
BASELINE_KM_M3 = 302.1

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROUTE_CSV = os.path.join(_THIS_DIR, '..', 'datasheets', 'sem_2025_eu.csv')

# ── Load and cache route ──────────────────────────────────────────────────────
def _load_route():
    df = pd.read_csv(ROUTE_CSV)
    df.columns = [c.strip() for c in df.columns]
    dist_col = [c for c in df.columns if 'Distance' in c][0]
    elev_col = [c for c in df.columns if 'Elevation' in c][0]

    s_raw = df[dist_col].values.astype(float)
    e_raw = df[elev_col].values.astype(float)

    D_LAP = float(s_raw[-1])

    # Smooth elevation (window 51, poly 3)
    e_smooth = savgol_filter(e_raw, window_length=51, polyorder=3)

    # Gradient, then smooth again
    grad_raw    = np.gradient(e_smooth, s_raw)
    grad_smooth = savgol_filter(grad_raw, window_length=51, polyorder=3)

    elev_fn  = interp1d(s_raw, e_smooth,    kind='linear', fill_value='extrapolate')
    grade_fn = interp1d(s_raw, grad_smooth, kind='linear', fill_value='extrapolate')

    return D_LAP, s_raw, e_smooth, grad_smooth, elev_fn, grade_fn


D_LAP, _s_raw, _e_smooth, _grad_smooth, _elev_fn, _grade_fn = _load_route()


# ── Physics ───────────────────────────────────────────────────────────────────
def _net_force(v, P_elec, grade):
    """Net traction force [N], forward-Euler compatible."""
    if P_elec > 0:
        F_motor = P_elec * ETA_DT / max(v, 0.3)
        F_motor = min(F_motor, MASS * 2.0)
    else:
        F_motor = 0.0
    F_drag    = 0.5 * CD * AF * RHO * v**2
    F_roll    = CRR * MASS * G
    F_gravity = MASS * G * grade
    return F_motor - F_drag - F_roll - F_gravity


# ── One-lap terrain-adaptive builder ─────────────────────────────────────────
def _build_one_lap(V_HIGH_FLAT, V_LOW_FLAT, V_MAX_DOWNHILL,
                   P_PULSE=1000.0, P_BOOST=1000.0):
    """
    Forward-Euler one-lap simulation with terrain-adaptive logic.

    Returns lists: t, v, P_elec, s, elev, grade
    """
    t_lst     = []
    v_lst     = []
    P_lst     = []
    s_lst     = []
    elev_lst  = []
    grade_lst = []

    t          = 0.0
    v          = 0.0
    s          = 0.0
    mode       = 'PULSE'   # start in pulse to accelerate from stop
    first_step = True
    prev_P     = 0.0

    MAX_LAP_TIME = 700.0   # safety cap

    while True:
        s_wrap = s % D_LAP
        grade  = float(_grade_fn(s_wrap))
        elev   = float(_elev_fn(s_wrap))

        # ── Brake check ──────────────────────────────────────────────────────
        remaining  = D_LAP - s_wrap
        brake_dist = v**2 / (2.0 * A_STOP) + v * DT

        if v > 0 and remaining <= brake_dist and remaining > 0 and mode != 'BRAKE':
            mode = 'BRAKE'

        # ── Determine target P_elec based on mode and terrain ─────────────
        if mode == 'BRAKE':
            P_target = 0.0
            v_cap    = V_MAX
        elif grade < -GRADE_THRESH:
            # Downhill: motor fully OFF, harvest gravity
            P_target = 0.0
            v_cap    = V_MAX_DOWNHILL
            # Force mode to GLIDE so when grade returns to flat, we check
            # if v > V_LOW_FLAT before deciding to pulse or glide
            mode = 'GLIDE'
        elif grade > +GRADE_THRESH:
            # Uphill: always boost, no glide on uphill
            P_target = P_BOOST
            v_cap    = V_MAX
            mode     = 'PULSE'
        else:
            # Flat: standard P&G with terrain-harvesting innovation
            v_cap = V_HIGH_FLAT + 1.0   # slight overshoot allowed before cap
            if v <= V_LOW_FLAT:
                mode     = 'PULSE'
                P_target = P_PULSE
            elif v >= V_HIGH_FLAT:
                mode     = 'GLIDE'
                P_target = 0.0
            else:
                # Maintain current mode (hysteresis)
                if mode == 'PULSE':
                    P_target = P_PULSE
                else:
                    # GLIDE or coming off downhill with extra speed
                    P_target = 0.0

        # Apply power ramp limit
        P_elec = float(np.clip(P_target,
                               prev_P - P_RAMP_MAX,
                               prev_P + P_RAMP_MAX))
        prev_P = P_elec

        # Record state BEFORE velocity update (skip first step: initial v=0)
        if not first_step:
            t_lst.append(t)
            v_lst.append(v)
            P_lst.append(P_elec)
            s_lst.append(s_wrap)
            elev_lst.append(elev)
            grade_lst.append(grade)
        first_step = False

        # ── Advance state ────────────────────────────────────────────────
        if mode == 'BRAKE':
            v_new = max(0.0, v - A_STOP * DT)
            s_new = s + v * DT
        else:
            F_net = _net_force(v, P_elec, grade)
            a     = F_net / MASS
            v_new = float(np.clip(v + a * DT, 0.0, v_cap))
            s_new = s + v * DT

        t += DT
        v  = v_new
        s  = s_new

        # ── Lap completion check ─────────────────────────────────────────
        if s >= D_LAP or (mode == 'BRAKE' and v == 0.0) or t > MAX_LAP_TIME:
            break

    # ── 4-second stop at lap line ────────────────────────────────────────────
    s_end     = float(s_lst[-1])
    elev_end  = float(elev_lst[-1])
    grade_end = float(grade_lst[-1])

    for _ in range(N_STOP_STEPS):
        t_lst.append(t)
        v_lst.append(0.0)
        P_lst.append(0.0)
        s_lst.append(s_end)
        elev_lst.append(elev_end)
        grade_lst.append(grade_end)
        t += DT

    return t_lst, v_lst, P_lst, s_lst, elev_lst, grade_lst


# ── Full 11-lap profile builder ───────────────────────────────────────────────
def build_terrain_profile(V_HIGH_FLAT, V_LOW_FLAT, V_MAX_DOWNHILL,
                          P_PULSE=1000.0, P_BOOST=1000.0):
    """
    Build full 11-lap terrain-adaptive profile.

    Returns arrays: t, v, P_elec, s, lap_num, elev, grade
    """
    all_t     = []
    all_v     = []
    all_P     = []
    all_s     = []
    all_lap   = []
    all_elev  = []
    all_grade = []

    t_offset = 0.0

    for lap in range(1, N_LAPS + 1):
        t_l, v_l, P_l, s_l, e_l, g_l = _build_one_lap(
            V_HIGH_FLAT, V_LOW_FLAT, V_MAX_DOWNHILL, P_PULSE, P_BOOST
        )
        t_arr_lap = np.array(t_l) + t_offset

        all_t.append(t_arr_lap)
        all_v.append(np.array(v_l))
        all_P.append(np.array(P_l))
        all_s.append(np.array(s_l))
        all_lap.append(np.full(len(t_l), lap, dtype=int))
        all_elev.append(np.array(e_l))
        all_grade.append(np.array(g_l))

        t_offset = float(t_arr_lap[-1]) + DT

    t_arr       = np.concatenate(all_t)
    v_arr       = np.concatenate(all_v)
    P_arr       = np.concatenate(all_P)
    s_arr       = np.concatenate(all_s)
    lap_num_arr = np.concatenate(all_lap)
    elev_arr    = np.concatenate(all_elev)
    grade_arr   = np.concatenate(all_grade)

    # Smooth velocity on motion segments (preserve v=0 stops)
    v_smooth    = v_arr.copy()
    in_motion   = (v_arr > 0.0)
    seg_starts  = np.where(np.diff(np.concatenate([[0], in_motion.astype(int)])) == 1)[0]
    seg_ends    = np.where(np.diff(np.concatenate([in_motion.astype(int), [0]])) == -1)[0] + 1
    for ms, me in zip(seg_starts, seg_ends):
        seg = v_arr[ms:me]
        if len(seg) > 25:
            v_smooth[ms:me] = savgol_filter(seg, window_length=21, polyorder=2)
        v_smooth[ms:me] = np.clip(v_smooth[ms:me], 0.0, V_MAX)
    v_arr = v_smooth

    return t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr


# ── Profile verification ──────────────────────────────────────────────────────
def verify_profile(t_arr, v_arr, lap_num_arr, silent=False):
    """
    Returns (ok, total_km, duration_min, n_stops) and optionally prints.
    Race constraints: total_dist > 14.5 km, 34-37 min, exactly 11 stops.
    """
    total_dist_m  = float(np.trapezoid(v_arr, t_arr))
    total_dist_km = total_dist_m / 1000.0
    duration_s    = float(t_arr[-1])
    duration_min  = duration_s / 60.0

    v_zero      = (v_arr == 0.0)
    stop_starts = np.where(np.diff(v_zero.astype(int)) == 1)[0] + 1
    n_stops     = len(stop_starts)

    ok_dist  = total_dist_km > 14.5
    ok_time  = 34.0 <= duration_min <= 37.0
    ok_stops = n_stops == 11

    ok = ok_dist and ok_time and ok_stops

    if not silent:
        print(f"  dist={total_dist_km:.3f} km  dur={duration_min:.1f} min  stops={n_stops}  "
              f"{'OK' if ok else 'FAIL'}")

    return ok, total_dist_km, duration_min, n_stops


# ── Demand array from physics profile ────────────────────────────────────────
def profile_to_demand(v_arr, P_elec_arr, grade_arr):
    """
    Convert motor power + velocity into electrical demand at motor controller.

    For each step: eta_m = motor_eta(P_shaft); P_elec_demand = P_shaft / eta_m
    But here P_elec_arr is already the motor ELECTRICAL input from the physics sim.
    We return it clipped to [0, 1000].
    """
    return np.clip(P_elec_arr, 0.0, 1000.0)


# ── Strategy G (FF + SOC-PI) ──────────────────────────────────────────────────
def make_strategy_g(K_p, K_i=2.0, tau=15.0, SC_SOC_0_ref=SC_SOC_0, T_LAP=None):
    """
    Feedforward + SOC-PI (Strategy G variant for custom profiles).

    Returns a callable: fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx) → (P_fc, P_filt)
    """
    alpha = float(np.exp(-DT / tau))
    K_LAP = 0.3

    state = {
        'P_filt':       None,
        'integrator':   0.0,
        'P_lap_offset': 0.0,
        'prev_lap_idx': -1,
    }

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        if state['P_filt'] is None:
            state['P_filt'] = float(P_dem)

        # Lap supervisor: adjust offset at lap boundaries
        t_lap_eff = T_LAP if T_LAP is not None else 190.0   # fallback ~3.2 min
        if lap_idx != state['prev_lap_idx'] and lap_idx > 0:
            E_deficit = (SC_SOC_0_ref - SOC) * SC_E_J
            state['P_lap_offset'] += K_LAP * E_deficit / t_lap_eff
            state['P_lap_offset'] = float(np.clip(state['P_lap_offset'], -200.0, 200.0))
        state['prev_lap_idx'] = lap_idx

        # Low-pass filter on demand
        state['P_filt'] = alpha * state['P_filt'] + (1.0 - alpha) * float(P_dem)
        P_filt = state['P_filt']

        # SOC-PI
        soc_err = SC_SOC_0_ref - SOC
        state['integrator'] += soc_err * DT
        if K_i > 1e-9:
            state['integrator'] = float(np.clip(
                state['integrator'], -150.0 / K_i, +150.0 / K_i))
        P_pi = K_p * soc_err + K_i * state['integrator']

        P_cmd = P_filt + P_pi + state['P_lap_offset']

        # FC suppressed during glide when SOC >= reference
        if P_dem < 25.0 and SOC >= SC_SOC_0_ref:
            P_cmd = 0.0

        P_fc = float(np.clip(P_cmd, 0.0, FC_P_MAX))
        return (P_fc, P_filt)

    return strategy_fn


# ── Simulator for custom demand profile ──────────────────────────────────────
def simulate_custom(strategy_fn, P_dem_arr, lap_num_arr, t_arr=None, SOC_0=SC_SOC_0):
    """
    Run EMS simulation on a pre-built demand array.

    Parameters
    ----------
    strategy_fn  : callable (P_dem, SOC, P_fc_prev, t_in_lap, lap_idx) → (P_fc, P_filt)
    P_dem_arr    : ndarray — motor electrical demand [W] at each timestep
    lap_num_arr  : ndarray (int) — 1-indexed lap number at each timestep
    t_arr        : ndarray — time [s] (optional, synthesised if None)
    SOC_0        : float — initial SC SOC

    Returns dict with standard fields.
    """
    n = len(P_dem_arr)
    lap_idx_arr = (lap_num_arr - 1).astype(int)   # 0-indexed

    if t_arr is None:
        t_arr = np.arange(n) * DT

    P_fc_ar  = np.empty(n)
    P_sc_ar  = np.empty(n)
    SOC_ar   = np.empty(n + 1)
    I_fc_ar  = np.empty(n)
    m_H2_ar  = np.empty(n)
    lap_SOC  = np.zeros(N_LAPS + 1)

    SOC_ar[0]  = SOC_0
    lap_SOC[0] = SOC_0
    P_fc_prev  = 0.0

    # Lap start times (for t_in_lap calculation)
    lap_start_t = {}
    for k in range(n):
        li = int(lap_idx_arr[k])
        if li not in lap_start_t:
            lap_start_t[li] = float(t_arr[k])

    for k in range(n):
        lap_idx  = int(lap_idx_arr[k])
        t_in_lap = float(t_arr[k]) - lap_start_t[lap_idx]
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem_arr[k])

        res = strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx)
        P_fc_cmd = float(res[0]) if isinstance(res, tuple) else float(res)

        # Ramp limiting: only limit upward ramp; instant shutdown allowed
        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k    = Pd - P_fc_cmd
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        # SC floor protection
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = float(np.clip(Pd, 0.0, FC_P_MAX))
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(P_fc_cmd)) if P_fc_cmd >= FC_P_MIN else 0.0
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev     = P_fc_cmd

        # Record SOC at lap boundaries
        nxt_lap = int(lap_idx_arr[k + 1]) if k + 1 < n else lap_idx + 1
        if nxt_lap != lap_idx:
            lap_SOC[lap_idx + 1] = SOC_ar[k + 1]

    lap_SOC[N_LAPS] = SOC_ar[n]

    return {
        't':           t_arr,
        'P_dem':       P_dem_arr,
        'P_fc':        P_fc_ar,
        'P_sc':        P_sc_ar,
        'SOC':         SOC_ar[:-1],
        'I_fc':        I_fc_ar,
        'm_H2_total':  float(np.sum(m_H2_ar)),
        'delta_SOC':   float(SOC_ar[n] - SOC_0),
        'SOC_final':   float(SOC_ar[n]),
        'lap_SOC':     lap_SOC,
    }


# ── Bisect K_p for charge sustenance ─────────────────────────────────────────
def bisect_kp(P_dem_arr, lap_num_arr, t_arr, T_LAP, tolerance=0.01, max_iter=20):
    """
    Binary-search K_p so that |ΔSOC| < tolerance.

    Returns (K_p_best, result_dict)
    """
    lo, hi = 100.0, 3000.0
    K_p    = (lo + hi) / 2.0
    best   = None

    for i in range(max_iter):
        strat  = make_strategy_g(K_p=K_p, K_i=2.0, tau=15.0, T_LAP=T_LAP)
        result = simulate_custom(strat, P_dem_arr, lap_num_arr, t_arr)
        d      = result['delta_SOC']
        best   = result

        if abs(d) <= tolerance:
            break

        # d > 0: SOC drifted high → FC produced too much → raise K_p (more FF correction)
        # d < 0: SOC drifted low  → FC produced too little → lower K_p
        # Actually: K_p raises P_cmd when SOC < ref, so higher K_p corrects negative drift
        if d < -tolerance:
            lo = K_p   # need more FC output → raise K_p
        else:
            hi = K_p   # need less FC output → lower K_p
        K_p = (lo + hi) / 2.0

    return K_p, best


# ── Parameter sweep ───────────────────────────────────────────────────────────
def run_sweep():
    """
    Coarse grid sweep over terrain-adaptive parameters.

    Returns list of result dicts, sorted by km/m³ descending.
    """
    V_HIGH_FLAT_GRID   = [7.0, 7.5, 8.0, 8.5, 9.0]
    V_LOW_FLAT_GRID    = [4.5, 5.0, 5.5, 6.0]
    V_MAX_DOWNHILL_GRID = [9.0, 10.0, 11.0]

    results = []

    total_combos = sum(
        1
        for vhi in V_HIGH_FLAT_GRID
        for vlo in V_LOW_FLAT_GRID
        for vdh in V_MAX_DOWNHILL_GRID
        if vlo < vhi - 1.0
    )

    print(f"\n=== AGENT 3 PARAMETER SWEEP: {total_combos} valid combinations ===\n")

    combo_idx = 0
    for V_HIGH_FLAT in V_HIGH_FLAT_GRID:
        for V_LOW_FLAT in V_LOW_FLAT_GRID:
            # Constraint: V_LOW_FLAT < V_HIGH_FLAT - 1.0
            if V_LOW_FLAT >= V_HIGH_FLAT - 1.0:
                continue
            for V_MAX_DOWNHILL in V_MAX_DOWNHILL_GRID:
                combo_idx += 1
                tag = (f"V_HI={V_HIGH_FLAT:.1f} V_LO={V_LOW_FLAT:.1f} "
                       f"V_DH={V_MAX_DOWNHILL:.1f}")
                print(f"[{combo_idx:2d}/{total_combos}] {tag} ... ", end='', flush=True)

                # Build profile
                try:
                    t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr = \
                        build_terrain_profile(
                            V_HIGH_FLAT, V_LOW_FLAT, V_MAX_DOWNHILL,
                            P_PULSE=1000.0, P_BOOST=1000.0
                        )
                except Exception as exc:
                    print(f"BUILD ERROR: {exc}")
                    continue

                # Verify
                ok, total_km, dur_min, n_stops = verify_profile(
                    t_arr, v_arr, lap_num_arr, silent=True
                )
                if not ok:
                    print(f"SKIP (dist={total_km:.2f}km dur={dur_min:.1f}min stops={n_stops})")
                    continue

                # Demand array
                P_dem = profile_to_demand(v_arr, P_arr, grade_arr)

                # Mean lap duration for lap supervisor
                T_LAP = float(t_arr[-1]) / N_LAPS

                # Bisect K_p
                K_p, sim_result = bisect_kp(P_dem, lap_num_arr, t_arr, T_LAP)

                m_H2   = sim_result['m_H2_total']
                dSOC   = sim_result['delta_SOC']
                km_m3  = TOTAL_KM / (m_H2 / H2_DENSITY)
                charge_ok = abs(dSOC) < 0.01

                print(f"H2={m_H2:.3f}g  km/m³={km_m3:.1f}  ΔSOC={dSOC:+.4f}  "
                      f"Kp={K_p:.0f}  {'OK' if charge_ok else 'NO-CHARGE'}")

                results.append({
                    'V_HIGH_FLAT':    V_HIGH_FLAT,
                    'V_LOW_FLAT':     V_LOW_FLAT,
                    'V_MAX_DOWNHILL': V_MAX_DOWNHILL,
                    'K_p':            K_p,
                    'H2_g':           m_H2,
                    'km_m3':          km_m3,
                    'delta_SOC':      dSOC,
                    'charge_ok':      charge_ok,
                    'total_km':       total_km,
                    'dur_min':        dur_min,
                    'sim_result':     sim_result,
                    't_arr':          t_arr,
                    'v_arr':          v_arr,
                    'P_arr':          P_arr,
                    's_arr':          s_arr,
                    'lap_num_arr':    lap_num_arr,
                    'elev_arr':       elev_arr,
                    'grade_arr':      grade_arr,
                })

    # Sort by km/m³ descending (charge-sustained first)
    results_cs = [r for r in results if r['charge_ok']]
    results_nc = [r for r in results if not r['charge_ok']]
    results_cs.sort(key=lambda r: r['km_m3'], reverse=True)
    results_nc.sort(key=lambda r: r['km_m3'], reverse=True)

    return results_cs + results_nc


# ── Fine-tuning sweep around best result ─────────────────────────────────────
def run_fine_sweep(best):
    """
    Fine-tune around the best coarse result.
    """
    v_hi_base = best['V_HIGH_FLAT']
    v_lo_base = best['V_LOW_FLAT']
    v_dh_base = best['V_MAX_DOWNHILL']

    V_HI_FINE   = sorted(set([
        round(v_hi_base - 0.25, 2), v_hi_base, round(v_hi_base + 0.25, 2)
    ]))
    V_LO_FINE   = sorted(set([
        round(v_lo_base - 0.25, 2), v_lo_base, round(v_lo_base + 0.25, 2)
    ]))
    V_DH_FINE   = sorted(set([
        round(v_dh_base - 0.5, 1), v_dh_base, round(v_dh_base + 0.5, 1)
    ]))

    results = []
    combos  = [
        (vhi, vlo, vdh)
        for vhi in V_HI_FINE
        for vlo in V_LO_FINE
        for vdh in V_DH_FINE
        if vlo < vhi - 1.0 and vlo > 0 and vhi > 0 and vdh > 0
    ]

    print(f"\n=== FINE SWEEP: {len(combos)} combinations around best ===\n")

    for idx, (V_HIGH_FLAT, V_LOW_FLAT, V_MAX_DOWNHILL) in enumerate(combos):
        tag = (f"V_HI={V_HIGH_FLAT:.2f} V_LO={V_LOW_FLAT:.2f} "
               f"V_DH={V_MAX_DOWNHILL:.2f}")
        print(f"[{idx+1:2d}/{len(combos)}] {tag} ... ", end='', flush=True)

        try:
            t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr = \
                build_terrain_profile(
                    V_HIGH_FLAT, V_LOW_FLAT, V_MAX_DOWNHILL,
                    P_PULSE=1000.0, P_BOOST=1000.0
                )
        except Exception as exc:
            print(f"BUILD ERROR: {exc}")
            continue

        ok, total_km, dur_min, n_stops = verify_profile(
            t_arr, v_arr, lap_num_arr, silent=True
        )
        if not ok:
            print(f"SKIP (dist={total_km:.2f}km dur={dur_min:.1f}min stops={n_stops})")
            continue

        P_dem  = profile_to_demand(v_arr, P_arr, grade_arr)
        T_LAP  = float(t_arr[-1]) / N_LAPS
        K_p, sim_result = bisect_kp(P_dem, lap_num_arr, t_arr, T_LAP)

        m_H2   = sim_result['m_H2_total']
        dSOC   = sim_result['delta_SOC']
        km_m3  = TOTAL_KM / (m_H2 / H2_DENSITY)
        charge_ok = abs(dSOC) < 0.01

        print(f"H2={m_H2:.3f}g  km/m³={km_m3:.1f}  ΔSOC={dSOC:+.4f}  "
              f"Kp={K_p:.0f}  {'OK' if charge_ok else 'NO-CHARGE'}")

        results.append({
            'V_HIGH_FLAT':    V_HIGH_FLAT,
            'V_LOW_FLAT':     V_LOW_FLAT,
            'V_MAX_DOWNHILL': V_MAX_DOWNHILL,
            'K_p':            K_p,
            'H2_g':           m_H2,
            'km_m3':          km_m3,
            'delta_SOC':      dSOC,
            'charge_ok':      charge_ok,
            'total_km':       total_km,
            'dur_min':        dur_min,
            'sim_result':     sim_result,
            't_arr':          t_arr,
            'v_arr':          v_arr,
            'P_arr':          P_arr,
            's_arr':          s_arr,
            'lap_num_arr':    lap_num_arr,
            'elev_arr':       elev_arr,
            'grade_arr':      grade_arr,
        })

    results_cs = [r for r in results if r['charge_ok']]
    results_nc = [r for r in results if not r['charge_ok']]
    results_cs.sort(key=lambda r: r['km_m3'], reverse=True)
    results_nc.sort(key=lambda r: r['km_m3'], reverse=True)

    return results_cs + results_nc


# ── Save best profile CSV ─────────────────────────────────────────────────────
def save_best_csv(best):
    out_path = os.path.join(MATLAB_DIR, 'sem_agent3_best.csv')
    sim      = best['sim_result']
    P_dem    = profile_to_demand(best['v_arr'], best['P_arr'], best['grade_arr'])

    df = pd.DataFrame({
        'time_s':        best['t_arr'],
        'velocity_ms':   best['v_arr'],
        'velocity_kmh':  best['v_arr'] * 3.6,
        'dist_in_lap_m': best['s_arr'],
        'lap_num':       best['lap_num_arr'],
        'elevation_m':   best['elev_arr'],
        'grade':         best['grade_arr'],
        'P_elec_W':      P_dem,
    })
    df.to_csv(out_path, index=False)
    print(f"  Best profile CSV saved → {out_path}")
    return out_path


# ── Result figure ─────────────────────────────────────────────────────────────
def save_result_figure(best):
    """
    3-panel figure:
      Top:          Single-lap velocity coloured by terrain
      Bottom-left:  Elevation profile overlaid on velocity
      Bottom-right: Bar chart comparing agent3 best vs baseline km/m³
    """
    out_path = os.path.join(RESULTS_DIR, 'agent3_terrain_adaptive_result.png')

    t_arr       = best['t_arr']
    v_arr       = best['v_arr']
    P_arr       = best['P_arr']
    s_arr       = best['s_arr']
    grade_arr   = best['grade_arr']
    elev_arr    = best['elev_arr']
    lap_num_arr = best['lap_num_arr']
    km_m3       = best['km_m3']

    # Lap 1 mask
    mask1    = lap_num_arr == 1
    s1       = s_arr[mask1]
    v1       = v_arr[mask1] * 3.6   # km/h
    g1       = grade_arr[mask1]
    P1       = P_arr[mask1]
    e1       = elev_arr[mask1]

    def _seg_color(i):
        if g1[i] < -GRADE_THRESH:
            return 'steelblue'      # downhill
        elif g1[i] > +GRADE_THRESH:
            return 'tomato'         # uphill
        elif v1[i] == 0.0:
            return 'black'          # brake/stop
        elif P1[i] > 0:
            return 'darkorange'     # flat-pulse
        else:
            return 'mediumseagreen' # flat-glide

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(
        "Agent 3 — Terrain-Adaptive Velocity Profile | Hydraix I | Silesia Ring SEM 2026\n"
        f"V_HIGH_FLAT={best['V_HIGH_FLAT']:.1f} m/s  "
        f"V_LOW_FLAT={best['V_LOW_FLAT']:.1f} m/s  "
        f"V_MAX_DH={best['V_MAX_DOWNHILL']:.1f} m/s  "
        f"→ {km_m3:.1f} km/m³  H2={best['H2_g']:.3f} g",
        fontsize=12, fontweight='bold', y=0.99
    )

    gs = fig.add_gridspec(2, 2, hspace=0.40, wspace=0.30,
                          height_ratios=[1.4, 1.0])
    ax_top  = fig.add_subplot(gs[0, :])     # full width top
    ax_bl   = fig.add_subplot(gs[1, 0])     # bottom left
    ax_br   = fig.add_subplot(gs[1, 1])     # bottom right

    # ── Top panel: velocity vs distance, coloured by terrain ──────────────
    for i in range(len(s1) - 1):
        c = _seg_color(i)
        ax_top.plot(s1[i:i+2], v1[i:i+2], color=c, linewidth=1.2)

    ax_top.axhline(best['V_HIGH_FLAT'] * 3.6, color='gray',
                   linestyle='--', linewidth=0.9,
                   label=f"V_HIGH_FLAT = {best['V_HIGH_FLAT']*3.6:.1f} km/h")
    ax_top.axhline(best['V_LOW_FLAT']  * 3.6, color='gray',
                   linestyle=':',  linewidth=0.9,
                   label=f"V_LOW_FLAT  = {best['V_LOW_FLAT']*3.6:.1f} km/h")
    ax_top.axhline(best['V_MAX_DOWNHILL'] * 3.6, color='steelblue',
                   linestyle='--', linewidth=0.7, alpha=0.6,
                   label=f"V_MAX_DH    = {best['V_MAX_DOWNHILL']*3.6:.1f} km/h")

    legend_handles = [
        mpatches.Patch(color='tomato',         label='Uphill (full power)'),
        mpatches.Patch(color='steelblue',      label='Downhill (motor off)'),
        mpatches.Patch(color='darkorange',     label='Flat — pulse'),
        mpatches.Patch(color='mediumseagreen', label='Flat — glide'),
        mpatches.Patch(color='black',          label='Brake / stop'),
    ]
    ax_top.legend(handles=legend_handles, fontsize=8, loc='upper right', ncol=2)
    ax_top.set_xlabel('Distance in lap [m]', fontsize=10)
    ax_top.set_ylabel('Velocity [km/h]', fontsize=10)
    ax_top.set_title('Terrain-Adaptive Velocity Profile — Lap 1', fontsize=11, fontweight='bold')
    ax_top.set_xlim(0, D_LAP)
    ax_top.grid(True, lw=0.3, alpha=0.5)

    # ── Bottom-left: elevation overlaid ───────────────────────────────────
    ax_bl2 = ax_bl.twinx()
    ax_bl2.plot(s1, e1, color='saddlebrown', linewidth=1.2,
                alpha=0.7, label='Elevation')
    ax_bl2.fill_between(s1, e1, float(np.min(e1)), color='saddlebrown',
                        alpha=0.12)
    ax_bl2.set_ylabel('Elevation [m]', fontsize=9, color='saddlebrown')
    ax_bl2.tick_params(axis='y', labelcolor='saddlebrown')

    for i in range(len(s1) - 1):
        c = _seg_color(i)
        ax_bl.plot(s1[i:i+2], v1[i:i+2], color=c, linewidth=1.0, alpha=0.8)

    ax_bl.axhline(best['V_HIGH_FLAT'] * 3.6, color='gray',
                  linestyle='--', linewidth=0.7)
    ax_bl.axhline(best['V_LOW_FLAT']  * 3.6, color='gray',
                  linestyle=':',  linewidth=0.7)
    ax_bl.set_xlabel('Distance in lap [m]', fontsize=9)
    ax_bl.set_ylabel('Velocity [km/h]', fontsize=9)
    ax_bl.set_title('Velocity + Elevation Overlay (Lap 1)', fontsize=9, fontweight='bold')
    ax_bl.set_xlim(0, D_LAP)
    ax_bl.grid(True, lw=0.3, alpha=0.4)

    # ── Bottom-right: bar chart comparison ────────────────────────────────
    labels  = ['Baseline\n(Elev P&G\nStrategy G)', 'Agent 3\n(Terrain-Adaptive)']
    values  = [BASELINE_KM_M3, km_m3]
    colors  = ['#4a90d9', '#e05a3a' if km_m3 < BASELINE_KM_M3 else '#2ecc71']

    bars = ax_br.bar(labels, values, color=colors, width=0.5, edgecolor='white',
                     linewidth=1.5)

    for bar, val in zip(bars, values):
        ax_br.text(bar.get_x() + bar.get_width() / 2,
                   bar.get_height() + 2,
                   f"{val:.1f}", ha='center', va='bottom',
                   fontsize=12, fontweight='bold')

    ax_br.axhline(BASELINE_KM_M3, color='gray', linestyle='--', linewidth=1.0,
                  alpha=0.6, label=f'Baseline {BASELINE_KM_M3:.1f}')
    ax_br.set_ylabel('Fuel Economy [km/m³]', fontsize=10)
    ax_br.set_title('Fuel Economy Comparison', fontsize=10, fontweight='bold')
    ax_br.set_ylim(0, max(values) * 1.15)
    ax_br.grid(True, axis='y', lw=0.3, alpha=0.5)

    delta = km_m3 - BASELINE_KM_M3
    sign  = '+' if delta >= 0 else ''
    ax_br.text(0.5, 0.05,
               f"Delta: {sign}{delta:.1f} km/m³\n"
               f"({'BEATS' if delta > 0 else 'BELOW'} baseline)",
               transform=ax_br.transAxes,
               ha='center', va='bottom',
               fontsize=10, fontweight='bold',
               color='#006400' if delta > 0 else '#8b0000')

    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Figure saved → {out_path}")
    return out_path


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # 1. Coarse sweep
    coarse_results = run_sweep()

    if not coarse_results:
        print("\nERROR: No valid results from coarse sweep. Exiting.")
        sys.exit(1)

    best_coarse = coarse_results[0]
    print(f"\nBest coarse: V_HI={best_coarse['V_HIGH_FLAT']:.1f}  "
          f"V_LO={best_coarse['V_LOW_FLAT']:.1f}  "
          f"V_DH={best_coarse['V_MAX_DOWNHILL']:.1f}  "
          f"km/m³={best_coarse['km_m3']:.1f}  ΔSOC={best_coarse['delta_SOC']:+.4f}")

    # 2. Fine sweep
    fine_results = run_fine_sweep(best_coarse)

    # 3. Pick overall best (prefer charge-sustained)
    all_results = coarse_results + fine_results
    cs_results  = [r for r in all_results if r['charge_ok']]
    if cs_results:
        cs_results.sort(key=lambda r: r['km_m3'], reverse=True)
        best = cs_results[0]
    else:
        all_results.sort(key=lambda r: r['km_m3'], reverse=True)
        best = all_results[0]

    # 4. Print final result
    km_m3_best = best['km_m3']
    beats = km_m3_best > BASELINE_KM_M3

    print("\n=== AGENT 3 BEST RESULT ===")
    print(f"V_HIGH_FLAT    = {best['V_HIGH_FLAT']:.1f} m/s ({best['V_HIGH_FLAT']*3.6:.1f} km/h)")
    print(f"V_LOW_FLAT     = {best['V_LOW_FLAT']:.1f} m/s ({best['V_LOW_FLAT']*3.6:.1f} km/h)")
    print(f"V_MAX_DOWNHILL = {best['V_MAX_DOWNHILL']:.1f} m/s ({best['V_MAX_DOWNHILL']*3.6:.1f} km/h)")
    print(f"K_p            = {best['K_p']:.0f}")
    print(f"H2             = {best['H2_g']:.3f} g")
    print(f"km/m³          = {km_m3_best:.1f}")
    print(f"ΔSOC           = {best['delta_SOC']:+.4f}")
    print(f"Charge sustained: {'YES' if best['charge_ok'] else 'NO'}")
    print(f"Beats baseline ({BASELINE_KM_M3:.1f} km/m³): {'YES' if beats else 'NO'}")
    print("===========================")

    # 5. Save outputs
    print("\nSaving outputs ...")
    save_best_csv(best)
    save_result_figure(best)

    print("\nDone.")
