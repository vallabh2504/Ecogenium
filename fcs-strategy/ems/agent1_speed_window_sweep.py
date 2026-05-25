"""
agent1_speed_window_sweep.py — Agent 1: V_HIGH / V_LOW Speed-Window Sweep
for Hydraix I at Silesia Ring SEM 2026.

Sweeps P&G speed window parameters to find the km/m³-optimal combination:
  V_HIGH: 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5 m/s
  V_LOW:  4.5, 5.0, 5.5, 6.0, 6.5, 7.0 m/s
  Constraint: V_LOW < V_HIGH - 1.0

Baseline: V_HIGH=8.2, V_LOW=5.83 → 302.1 km/m³
"""

import matplotlib
matplotlib.use('Agg')

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

# ── Paths ─────────────────────────────────────────────────────────────────────
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
ROUTE_CSV   = os.path.join(_THIS_DIR, '..', 'datasheets', 'sem_2025_eu.csv')
MATLAB_DIR  = os.path.join(_THIS_DIR, '..', 'matlab')
RESULTS_DIR = os.path.join(_THIS_DIR, '..', 'results', 'ems')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(MATLAB_DIR, exist_ok=True)

# Import ems_core components
from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_E_J, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, fc_current, fc_h2_rate, sc_soc_update, motor_eta,
    N_LAPS, DT, MATLAB_DIR as _CORE_MATLAB_DIR, RESULTS_DIR as _CORE_RESULTS_DIR,
)

# ── Vehicle constants ──────────────────────────────────────────────────────────
MASS    = 175.0
G       = 9.81
CD      = 0.15
AF      = 0.8
CRR     = 0.006
RHO     = 1.225
ETA_DT  = 0.95

# ── Race / profile constants ───────────────────────────────────────────────────
V_MAX         = 10.0      # m/s  absolute speed cap
GRADE_THRESH  = 0.006     # 0.6% grade threshold
A_STOP        = 0.5       # m/s² coast-to-stop deceleration
P_RAMP_MAX    = 200.0     # W/step motor ramp limit
N_STOP_STEPS  = round(4.0 / DT)   # 4-second stop = 20 steps

# ── H2 / economy constants ─────────────────────────────────────────────────────
H2_DENSITY = 89.88        # g/m³  at STP
TOTAL_KM   = 14.5

# ── Sweep parameters ──────────────────────────────────────────────────────────
V_HIGH_LIST = [6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5]
V_LOW_LIST  = [4.5, 5.0, 5.5, 6.0, 6.5, 7.0]
P_PULSE     = 1000.0      # W  motor electrical power — fixed
P_BOOST     = 1000.0      # W  motor electrical power on uphill — fixed

# ── Baseline ──────────────────────────────────────────────────────────────────
BASELINE_KM_M3  = 302.1
BASELINE_V_HIGH = 8.2
BASELINE_V_LOW  = 5.83

# ── Load and smooth route ──────────────────────────────────────────────────────
def _load_route():
    df = pd.read_csv(ROUTE_CSV)
    df.columns = [c.strip() for c in df.columns]
    dist_col = [c for c in df.columns if 'Distance' in c][0]
    elev_col = [c for c in df.columns if 'Elevation' in c][0]
    s_raw = df[dist_col].values
    e_raw = df[elev_col].values
    D_LAP = float(s_raw[-1])
    e_smooth    = savgol_filter(e_raw,   window_length=51, polyorder=3)
    grad_raw    = np.gradient(e_smooth, s_raw)
    grad_smooth = savgol_filter(grad_raw, window_length=51, polyorder=3)
    elev_fn  = interp1d(s_raw, e_smooth,    kind='linear', fill_value='extrapolate')
    grade_fn = interp1d(s_raw, grad_smooth, kind='linear', fill_value='extrapolate')
    return D_LAP, s_raw, e_smooth, grad_smooth, elev_fn, grade_fn

D_LAP, _s_raw, _e_smooth, _grad_smooth, _elev_fn, _grade_fn = _load_route()

# ── Physics helpers ────────────────────────────────────────────────────────────
def _net_force(v, P_elec, grade):
    if P_elec > 0:
        F_motor = P_elec * ETA_DT / max(v, 0.3)
        F_motor = min(F_motor, MASS * 2.0)
    else:
        F_motor = 0.0
    F_drag    = 0.5 * CD * AF * RHO * v**2
    F_roll    = CRR * MASS * G
    F_gravity = MASS * G * grade
    return F_motor - F_drag - F_roll - F_gravity

# ── One-lap P&G builder ────────────────────────────────────────────────────────
def _build_one_lap(V_HIGH, V_LOW, P_PULSE=1000.0, P_BOOST=1000.0):
    """
    Forward-Euler simulation of one P&G lap with grade awareness.
    Returns lists: t, v, P_elec, s (dist in lap), elev, grade
    """
    t_lst     = []
    v_lst     = []
    P_lst     = []
    s_lst     = []
    elev_lst  = []
    grade_lst = []

    t     = 0.0
    v     = 0.0
    s     = 0.0
    mode  = 'PULSE'
    first_step   = True
    prev_P_elec  = 0.0
    MAX_LAP_TIME = 600.0

    while True:
        s_wrap = s % D_LAP
        grade  = float(_grade_fn(s_wrap))
        elev   = float(_elev_fn(s_wrap))

        # Brake check: begin coasting to stop at lap end
        remaining  = D_LAP - s_wrap
        brake_dist = v**2 / (2.0 * A_STOP) + v * DT

        if v > 0 and remaining <= brake_dist and remaining > 0 and mode != 'BRAKE':
            mode = 'BRAKE'

        # Determine P_elec for this step
        if mode == 'BRAKE':
            P_elec = 0.0
        elif grade < -GRADE_THRESH:
            if v < V_LOW and grade > -0.005:
                P_elec = P_PULSE
                mode   = 'PULSE'
            else:
                P_elec = 0.0
        elif grade > +GRADE_THRESH:
            if v >= V_HIGH:
                mode   = 'GLIDE'
                P_elec = 0.0
            else:
                mode   = 'PULSE'
                P_elec = P_BOOST
        else:
            # Flat: standard P&G
            if v <= V_LOW:
                mode   = 'PULSE'
                P_elec = P_PULSE
            elif v >= V_HIGH:
                mode   = 'GLIDE'
                P_elec = 0.0
            else:
                if mode == 'PULSE':
                    P_elec = P_PULSE
                elif mode == 'GLIDE':
                    P_elec = 0.0
                else:
                    P_elec = P_PULSE

        # Ramp limit
        P_elec = float(np.clip(P_elec,
                               prev_P_elec - P_RAMP_MAX,
                               prev_P_elec + P_RAMP_MAX))
        prev_P_elec = P_elec

        # Record (skip first step)
        if not first_step:
            t_lst.append(t)
            v_lst.append(v)
            P_lst.append(P_elec)
            s_lst.append(s_wrap)
            elev_lst.append(elev)
            grade_lst.append(grade)
        first_step = False

        # Advance state
        if mode == 'BRAKE':
            P_elec = 0.0
            v_new  = max(0.0, v - A_STOP * DT)
            s_new  = s + v * DT
        else:
            F_net = _net_force(v, P_elec, grade)
            a     = F_net / MASS
            v_new = max(0.0, v + a * DT)
            if grade < -GRADE_THRESH:
                v_new = min(v_new, min(V_MAX, V_HIGH + 3.0))
            else:
                v_new = min(v_new, V_MAX)
            s_new = s + v * DT

        t += DT
        v  = v_new
        s  = s_new

        if s >= D_LAP or (mode == 'BRAKE' and v == 0.0) or t > MAX_LAP_TIME:
            break

    # Append 4-second stop
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


# ── Full 11-lap profile builder ────────────────────────────────────────────────
def build_full_profile(V_HIGH, V_LOW, P_PULSE=1000.0, P_BOOST=1000.0):
    """Build 11-lap race profile and return as arrays."""
    all_t     = []
    all_v     = []
    all_P     = []
    all_s     = []
    all_lap   = []
    all_elev  = []
    all_grade = []
    t_offset  = 0.0

    for lap in range(1, N_LAPS + 1):
        t_l, v_l, P_l, s_l, e_l, g_l = _build_one_lap(V_HIGH, V_LOW, P_PULSE, P_BOOST)
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

    # Smooth velocity (preserve v=0 stops)
    v_smooth = v_arr.copy()
    in_motion = (v_arr > 0.0)
    motion_starts = np.where(np.diff(np.concatenate([[0], in_motion.astype(int)])) == 1)[0]
    motion_ends   = np.where(np.diff(np.concatenate([in_motion.astype(int), [0]])) == -1)[0] + 1
    for ms, me in zip(motion_starts, motion_ends):
        seg = v_arr[ms:me]
        if len(seg) > 25:
            v_smooth[ms:me] = savgol_filter(seg, window_length=21, polyorder=2)
        v_smooth[ms:me] = np.clip(v_smooth[ms:me], 0.0, V_MAX)
    v_arr = v_smooth

    return t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr


# ── Verification ───────────────────────────────────────────────────────────────
def verify_profile(t_arr, v_arr):
    """
    Check profile constraints. Returns (ok, total_dist_km, duration_min, n_stops).
    """
    total_dist_m  = float(np.trapezoid(v_arr, t_arr))
    total_dist_km = total_dist_m / 1000.0
    duration_min  = float(t_arr[-1]) / 60.0

    v_zero       = (v_arr == 0.0)
    stop_starts  = np.where(np.diff(v_zero.astype(int)) == 1)[0] + 1
    n_stops      = len(stop_starts)

    ok_dist  = total_dist_km > 14.5
    ok_time  = 34.0 <= duration_min <= 37.0
    ok_stops = n_stops == 11

    ok = ok_dist and ok_time and ok_stops
    return ok, total_dist_km, duration_min, n_stops


# ── Demand (P_elec) from velocity profile ─────────────────────────────────────
def compute_demand(v_arr, grade_arr):
    """
    Compute electrical motor demand from velocity and grade arrays.
    Uses motor_eta from ems_core (BAFANG lookup).
    """
    accel = np.gradient(v_arr, DT)
    P_wheel = (CRR * MASS * G * v_arr
               + 0.5 * CD * AF * RHO * v_arr**3
               + MASS * accel * v_arr
               + MASS * G * grade_arr * v_arr)
    P_motor = np.where(P_wheel > 0, P_wheel / ETA_DT, 0.0)
    eta_m   = motor_eta(P_motor)
    P_elec  = np.where(P_motor > 0, P_motor / eta_m, 0.0)
    P_elec  = np.clip(np.where(np.isfinite(P_elec), P_elec, 0.0), 0.0, None)
    return P_elec


# ── Strategy G factory ─────────────────────────────────────────────────────────
def make_strategy_g(K_p, K_i=2.0, tau=15.0, SC_SOC_0_ref=SC_SOC_0):
    """Feedforward LPF + SOC-PI (Strategy G). Returns callable strategy_fn."""
    alpha = np.exp(-DT / tau)

    state = {'P_lpf': 0.0, 'integral': 0.0}

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        state['P_lpf'] = alpha * state['P_lpf'] + (1 - alpha) * P_dem
        err = SC_SOC_0_ref - SOC
        state['integral'] += err * DT
        P_cmd = state['P_lpf'] + K_p * err + K_i * state['integral']
        # Glide override: FC off when demand low AND SOC at/above target
        if P_dem < 25.0 and SOC >= SC_SOC_0_ref:
            P_cmd = 0.0
        return float(np.clip(P_cmd, 0.0, FC_P_MAX))

    return strategy_fn


# ── Custom race simulator ──────────────────────────────────────────────────────
def simulate_custom(strategy_fn, P_dem_arr, lap_num_arr, t_arr, SOC_0=SC_SOC_0):
    """
    Simulate Strategy G on a custom demand array.
    lap boundaries come from the lap_num column of the generated profile.
    fc_can_off=True semantics (instant shutdown, ramp-limited upward only).
    """
    n = len(P_dem_arr)

    P_fc_ar  = np.empty(n)
    P_sc_ar  = np.empty(n)
    SOC_ar   = np.empty(n + 1)
    I_fc_ar  = np.empty(n)
    m_H2_ar  = np.empty(n)
    lap_SOC  = np.zeros(N_LAPS + 1)

    SOC_ar[0]  = SOC_0
    lap_SOC[0] = SOC_0
    P_fc_prev  = 0.0

    # Build per-lap start times
    lap_start_t = {}
    for k in range(n):
        li = int(lap_num_arr[k])
        if li not in lap_start_t:
            lap_start_t[li] = float(t_arr[k])

    for k in range(n):
        lap_idx  = int(lap_num_arr[k])
        t_in_lap = float(t_arr[k]) - lap_start_t[lap_idx]
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem_arr[k])

        P_fc_cmd = float(strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx))

        # fc_can_off=True: only limit upward ramp
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

        # Record lap boundary SOC
        nxt_lap = int(lap_num_arr[k + 1]) if k + 1 < n else lap_idx + 1
        if nxt_lap != lap_idx:
            lap_SOC[lap_idx] = SOC_ar[k + 1]  # store at end of this lap

    lap_SOC[N_LAPS] = float(SOC_ar[n])

    return {
        't':          t_arr,
        'P_dem':      P_dem_arr,
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'SOC_final':  float(SOC_ar[n]),
        'lap_SOC':    lap_SOC,
    }


# ── Bisect K_p for charge sustenance ──────────────────────────────────────────
def bisect_kp(sim_fn, tolerance=0.01, max_iters=20, verbose=False):
    """
    Bisect K_p in [100, 3000] for |ΔSOC| < tolerance.
    Returns (K_p, result_dict) or (None, None) if not converged.
    """
    lo, hi = 100.0, 3000.0
    K_p    = (lo + hi) / 2.0
    best_result = None
    best_Kp     = K_p

    for i in range(max_iters):
        strat  = make_strategy_g(K_p)
        result = sim_fn(strat)
        d      = result['delta_SOC']
        if verbose:
            print(f"    bisect iter {i+1:2d}  K_p={K_p:7.1f}  ΔSOC={d:+.4f}  "
                  f"H2={result['m_H2_total']:.3f} g")
        best_result = result
        best_Kp     = K_p
        if abs(d) <= tolerance:
            break
        # d > 0 → SOC rising → FC too active → raise K_p to push P_cmd up
        # Wait: if SOC drifts positive (d>0), the FC is OVER-producing,
        # meaning K_p is TOO LOW (correction P_pi is positive but SOC > ref).
        # Actually: err = SC_SOC_0 - SOC; if SOC > SC_SOC_0 → err < 0 → P_pi < 0
        # → FC commanded lower → SOC drift should self-correct.
        # delta_SOC = SOC_end - SOC_0.
        # d > 0 → SOC ended higher than start → FC over-produced → need to
        # reduce average FC output → increase K_p (larger negative correction
        # when SOC > target, so FC ramps down more aggressively).
        # d < 0 → SOC ended lower → FC under-produced → decrease K_p.
        if d > tolerance:
            lo = K_p   # SOC drifting high → increase K_p
        else:
            hi = K_p   # SOC drifting low  → decrease K_p
        K_p = (lo + hi) / 2.0

    return best_Kp, best_result


# ── km/m³ calculation ──────────────────────────────────────────────────────────
def compute_km_m3(H2_g):
    return TOTAL_KM / (H2_g / H2_DENSITY)


# ── Main sweep ────────────────────────────────────────────────────────────────
def main():
    print("=" * 65)
    print("  Agent 1 — V_HIGH / V_LOW Speed-Window Sweep")
    print(f"  Baseline: V_HIGH={BASELINE_V_HIGH}, V_LOW={BASELINE_V_LOW} → {BASELINE_KM_M3} km/m³")
    print("=" * 65)

    results = []

    for V_HIGH in V_HIGH_LIST:
        for V_LOW in V_LOW_LIST:
            # Constraint: V_LOW < V_HIGH - 1.0
            if V_LOW >= V_HIGH - 1.0:
                continue

            print(f"\n[V_HIGH={V_HIGH:.1f}, V_LOW={V_LOW:.1f}]  building profile ...", end=' ', flush=True)

            # 1. Build 11-lap profile
            try:
                t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr = \
                    build_full_profile(V_HIGH, V_LOW, P_PULSE, P_BOOST)
            except Exception as e:
                print(f"SKIPPED (build error: {e})")
                continue

            # 2. Verify profile
            ok, total_km, dur_min, n_stops = verify_profile(t_arr, v_arr)
            if not ok:
                print(f"SKIPPED (dist={total_km:.2f}km, dur={dur_min:.1f}min, stops={n_stops})")
                continue
            print(f"OK ({total_km:.2f}km, {dur_min:.1f}min, {n_stops} stops)")

            # 3. Use the P&G command array as the demand signal (binary 0/P_PULSE)
            #    This matches how strategy_g_elev_compare.py uses P_elec_W from CSV:
            #    the EMS sees the motor's requested electrical power, not a physics
            #    reconstruction from smoothed velocity.
            P_dem_arr = P_arr

            # 4. Bisect K_p
            def sim_fn(strat_fn):
                return simulate_custom(strat_fn, P_dem_arr, lap_num_arr, t_arr)

            print(f"  Bisecting K_p ...", end=' ', flush=True)
            K_p, result = bisect_kp(sim_fn, verbose=False)

            if result is None:
                print("SKIPPED (bisect failed)")
                continue

            H2_g      = result['m_H2_total']
            delta_soc = result['delta_SOC']
            km_m3     = compute_km_m3(H2_g)
            charge_ok = abs(delta_soc) < 0.01

            print(f"K_p={K_p:.0f}  H2={H2_g:.3f}g  km/m³={km_m3:.1f}  ΔSOC={delta_soc:+.4f}  "
                  f"{'CS:OK' if charge_ok else 'CS:FAIL'}")

            results.append({
                'V_HIGH':    V_HIGH,
                'V_LOW':     V_LOW,
                'H2_g':      H2_g,
                'km_m3':     km_m3,
                'delta_soc': delta_soc,
                'K_p':       K_p,
                'charge_ok': charge_ok,
                'total_km':  total_km,
                'dur_min':   dur_min,
                # Store profile for potential saving
                '_t_arr':    t_arr,
                '_v_arr':    v_arr,
                '_P_arr':    P_arr,
                '_s_arr':    s_arr,
                '_lap_num':  lap_num_arr,
                '_elev_arr': elev_arr,
                '_grade_arr':grade_arr,
                '_P_dem':    P_dem_arr,
            })

    if not results:
        print("\nNo valid results found!")
        return

    # ── Find best (charge-sustained only, by km/m³) ───────────────────────────
    valid = [r for r in results if r['charge_ok']]
    if not valid:
        print("\nNo charge-sustained results; picking best by km/m³ regardless.")
        valid = results

    best = max(valid, key=lambda r: r['km_m3'])

    # ── Print summary table ───────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print(f"{'V_HIGH':>7} {'V_LOW':>6} {'K_p':>7} {'H2(g)':>8} {'km/m³':>8} {'ΔSOC':>8} {'CS':>4}")
    print("-" * 65)
    for r in sorted(results, key=lambda x: x['km_m3'], reverse=True):
        cs  = 'YES' if r['charge_ok'] else 'NO'
        mrk = ' *' if r is best else ''
        print(f"{r['V_HIGH']:>7.1f} {r['V_LOW']:>6.1f} {r['K_p']:>7.0f} "
              f"{r['H2_g']:>8.3f} {r['km_m3']:>8.1f} {r['delta_soc']:>+8.4f} {cs:>4}{mrk}")
    print("=" * 65)

    # ── Save best profile CSV ─────────────────────────────────────────────────
    best_csv = os.path.join(MATLAB_DIR, 'sem_agent1_best.csv')
    df_best = pd.DataFrame({
        'time_s':        best['_t_arr'],
        'velocity_ms':   best['_v_arr'],
        'velocity_kmh':  best['_v_arr'] * 3.6,
        'dist_in_lap_m': best['_s_arr'],
        'lap_num':       best['_lap_num'],
        'elevation_m':   best['_elev_arr'],
        'grade':         best['_grade_arr'],
        'P_elec_W':      best['_P_dem'],
    })
    df_best.to_csv(best_csv, index=False)
    print(f"\nBest profile saved → {best_csv}  ({len(df_best)} rows)")

    # ── Load baseline lap-1 velocity for comparison ───────────────────────────
    baseline_csv = os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv')
    try:
        df_base      = pd.read_csv(baseline_csv)
        base_lap1    = df_base[df_base['lap_num'] == 1]
        base_t_lap1  = base_lap1['time_s'].values - base_lap1['time_s'].values[0]
        base_v_lap1  = base_lap1['velocity_kmh'].values
    except Exception:
        base_t_lap1 = None
        base_v_lap1 = None

    # ── Result figure ─────────────────────────────────────────────────────────
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle(
        "Agent 1 — V_HIGH / V_LOW Speed-Window Sweep | Hydraix I SEM 2026\n"
        f"Best: V_HIGH={best['V_HIGH']:.1f} m/s, V_LOW={best['V_LOW']:.1f} m/s  →  "
        f"{best['km_m3']:.1f} km/m³  (baseline {BASELINE_KM_M3} km/m³)",
        fontsize=11, fontweight='bold'
    )

    # Top panel: heatmap of km/m³ vs (V_HIGH, V_LOW)
    all_vh = sorted(set(r['V_HIGH'] for r in results))
    all_vl = sorted(set(r['V_LOW']  for r in results))
    heat   = np.full((len(all_vl), len(all_vh)), np.nan)
    for r in results:
        if not r['charge_ok']:
            continue
        ri = all_vl.index(r['V_LOW'])
        ci = all_vh.index(r['V_HIGH'])
        heat[ri, ci] = r['km_m3']

    im = ax_top.imshow(heat, aspect='auto', origin='lower',
                       cmap='RdYlGn', interpolation='nearest')
    ax_top.set_xticks(range(len(all_vh)))
    ax_top.set_xticklabels([f"{v:.1f}" for v in all_vh])
    ax_top.set_yticks(range(len(all_vl)))
    ax_top.set_yticklabels([f"{v:.1f}" for v in all_vl])
    ax_top.set_xlabel('V_HIGH [m/s]', fontsize=10)
    ax_top.set_ylabel('V_LOW [m/s]',  fontsize=10)
    ax_top.set_title('km/m³ Heatmap (charge-sustained combos only)', fontsize=10)
    cbar = fig.colorbar(im, ax=ax_top)
    cbar.set_label('km/m³', fontsize=9)

    # Annotate cells
    for r in results:
        if not r['charge_ok']:
            continue
        ri = all_vl.index(r['V_LOW'])
        ci = all_vh.index(r['V_HIGH'])
        ax_top.text(ci, ri, f"{r['km_m3']:.0f}", ha='center', va='center',
                    fontsize=7, color='black', fontweight='bold')

    # Mark best cell
    best_ri = all_vl.index(best['V_LOW'])
    best_ci = all_vh.index(best['V_HIGH'])
    ax_top.add_patch(plt.Rectangle(
        (best_ci - 0.5, best_ri - 0.5), 1.0, 1.0,
        fill=False, edgecolor='blue', linewidth=2.5, label='Best'
    ))
    ax_top.legend(fontsize=8, loc='upper right')

    # Bottom panel: single-lap velocity profile of best vs baseline
    best_lap1_mask = best['_lap_num'] == 1
    best_t_lap1    = best['_t_arr'][best_lap1_mask] - best['_t_arr'][best_lap1_mask][0]
    best_v_lap1    = best['_v_arr'][best_lap1_mask] * 3.6

    ax_bot.plot(best_t_lap1, best_v_lap1,
                color='tomato', lw=1.2, label=f"Best: V_H={best['V_HIGH']:.1f}, V_L={best['V_LOW']:.1f}")
    if base_t_lap1 is not None:
        ax_bot.plot(base_t_lap1, base_v_lap1,
                    color='royalblue', lw=1.0, alpha=0.8,
                    label=f"Baseline: V_H={BASELINE_V_HIGH}, V_L={BASELINE_V_LOW}")

    ax_bot.axhline(best['V_HIGH'] * 3.6, color='tomato', ls='--', lw=0.8,
                   label=f"Best V_HIGH={best['V_HIGH']*3.6:.1f} km/h")
    ax_bot.axhline(best['V_LOW']  * 3.6, color='tomato', ls=':',  lw=0.8,
                   label=f"Best V_LOW={best['V_LOW']*3.6:.1f} km/h")
    ax_bot.axhline(BASELINE_V_HIGH * 3.6, color='royalblue', ls='--', lw=0.8, alpha=0.6,
                   label=f"Baseline V_HIGH={BASELINE_V_HIGH*3.6:.1f} km/h")
    ax_bot.axhline(BASELINE_V_LOW  * 3.6, color='royalblue', ls=':',  lw=0.8, alpha=0.6,
                   label=f"Baseline V_LOW={BASELINE_V_LOW*3.6:.1f} km/h")

    ax_bot.set_xlabel('Time in lap [s]', fontsize=10)
    ax_bot.set_ylabel('Velocity [km/h]', fontsize=10)
    ax_bot.set_title('Lap 1 Velocity Profile: Best vs Baseline', fontsize=10)
    ax_bot.legend(fontsize=8, loc='upper right', ncol=2)
    ax_bot.grid(True, lw=0.25)

    fig.tight_layout(rect=[0, 0, 1, 0.93])
    result_png = os.path.join(RESULTS_DIR, 'agent1_speed_window_result.png')
    fig.savefig(result_png, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Result figure saved → {result_png}")

    # ── Final output ──────────────────────────────────────────────────────────
    charge_ok_str = "YES" if best['charge_ok'] else "NO"
    beats_str     = "YES" if best['km_m3'] > BASELINE_KM_M3 else "NO"

    print()
    print("=== AGENT 1 BEST RESULT ===")
    print(f"V_HIGH = {best['V_HIGH']:.1f} m/s ({best['V_HIGH']*3.6:.1f} km/h)")
    print(f"V_LOW  = {best['V_LOW']:.1f} m/s ({best['V_LOW']*3.6:.1f} km/h)")
    print(f"K_p    = {best['K_p']:.0f}")
    print(f"H2     = {best['H2_g']:.3f} g")
    print(f"km/m³  = {best['km_m3']:.1f}")
    print(f"ΔSOC   = {best['delta_soc']:+.4f}")
    print(f"Charge sustained: {charge_ok_str}")
    print(f"Beats baseline ({BASELINE_KM_M3} km/m³): {beats_str}")
    print("===========================")


if __name__ == '__main__':
    main()
