"""
agent2_power_sweep.py — Agent 2: Optimal motor pulse power sweep for Hydraix I.

Sweeps P_PULSE in [400, 500, 600, 700, 800, 900, 1000] W (with P_BOOST fixed at
1000 W for uphill), builds physics-based 11-lap profiles, runs Strategy G
(FF+SOC-PI) on each, and reports km/m³ fuel economy to find the optimal P_PULSE.

Vehicle: MASS=175 kg, CD=0.15, AF=0.8 m², CRR=0.006, RHO=1.225 kg/m³, ETA_DT=0.95
Race:    Silesia Ring SEM 2026, 11 laps, distance > 14.5 km, 34–37 min
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_E_J, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, fc_current, fc_h2_rate, sc_soc_update, motor_eta,
    _M_POUT_S, _M_ETA_S,
    N_LAPS, DT, MATLAB_DIR, RESULTS_DIR,
)

# ── Constants ─────────────────────────────────────────────────────────────────
H2_DENSITY   = 89.88   # g/m³ at STP
TOTAL_KM     = 14.5
TOLERANCE    = 0.01    # |ΔSOC| target for charge sustenance
K_I_FIXED    = 2.0
TAU          = 15.0

# Vehicle constants
MASS    = 175.0
G_GRAV  = 9.81
CD      = 0.15
AF      = 0.8
CRR     = 0.006
RHO     = 1.225
ETA_DT  = 0.95

# P&G fixed params
V_HIGH        = 8.2     # m/s glide trigger
V_LOW         = 5.83    # m/s pulse trigger
V_MAX         = 10.0    # m/s absolute speed cap
P_BOOST_FIXED = 1000.0  # W for uphill (always rated)
A_STOP        = 0.5     # m/s² gentle coast-to-stop
GRADE_THRESH  = 0.006   # 0.6% grade threshold
P_RAMP_MAX    = 200.0   # W/step motor power ramp
N_STOP_STEPS  = round(4.0 / DT)  # 4-second stop = 20 steps

# ── Load route elevation data ─────────────────────────────────────────────────
_ROUTE_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'datasheets', 'sem_2025_eu.csv')

def _load_route():
    df = pd.read_csv(_ROUTE_CSV)
    df.columns = [c.strip() for c in df.columns]
    dist_col = [c for c in df.columns if 'Distance' in c][0]
    elev_col = [c for c in df.columns if 'Elevation' in c][0]

    s_raw = df[dist_col].values
    e_raw = df[elev_col].values
    D_LAP = float(s_raw[-1])

    e_smooth   = savgol_filter(e_raw, window_length=51, polyorder=3)
    grad_raw   = np.gradient(e_smooth, s_raw)
    grad_smooth = savgol_filter(grad_raw, window_length=51, polyorder=3)

    elev_fn  = interp1d(s_raw, e_smooth,     kind='linear', fill_value='extrapolate')
    grade_fn = interp1d(s_raw, grad_smooth,  kind='linear', fill_value='extrapolate')

    return D_LAP, s_raw, e_smooth, grad_smooth, elev_fn, grade_fn

D_LAP, _s_raw, _e_smooth, _grad_smooth, _elev_fn, _grade_fn = _load_route()


# ── Physics helpers ───────────────────────────────────────────────────────────

def _net_force(v, P_elec, grade):
    if P_elec > 0:
        F_motor = P_elec * ETA_DT / max(v, 0.3)
        F_motor = min(F_motor, MASS * 2.0)
    else:
        F_motor = 0.0
    F_drag    = 0.5 * CD * AF * RHO * v**2
    F_roll    = CRR * MASS * G_GRAV
    F_gravity = MASS * G_GRAV * grade
    return F_motor - F_drag - F_roll - F_gravity


# ── Motor η iterative solve for electrical input ──────────────────────────────

def _motor_pout_from_pelec(P_elec_W):
    """
    Solve P_elec = P_out / eta(P_out) iteratively.
    Returns P_out (shaft power) given P_elec (electrical input).
    """
    if P_elec_W <= 0:
        return 0.0
    # Initial guess: use η at midpoint
    p_out = P_elec_W * 0.8  # start guess
    for _ in range(20):
        eta = float(np.interp(p_out, _M_POUT_S, _M_ETA_S,
                              left=_M_ETA_S[0], right=_M_ETA_S[-1]))
        p_out_new = P_elec_W * eta
        if abs(p_out_new - p_out) < 0.01:
            p_out = p_out_new
            break
        p_out = p_out_new
    return p_out


def _motor_eta_from_pelec(P_elec_W):
    """Return motor η given electrical input power."""
    if P_elec_W <= 0:
        return 0.0
    p_out = _motor_pout_from_pelec(P_elec_W)
    return float(np.interp(p_out, _M_POUT_S, _M_ETA_S,
                           left=_M_ETA_S[0], right=_M_ETA_S[-1]))


# ── One-lap builder (parameterised) ──────────────────────────────────────────

def _build_one_lap(P_PULSE, P_BOOST=P_BOOST_FIXED):
    """
    Forward-Euler single-lap simulation with elevation-aware P&G.

    Returns lists: t, v, P_elec, s, elev, grade
    """
    t_lst = []; v_lst = []; P_lst = []; s_lst = []; elev_lst = []; grade_lst = []

    t = 0.0; v = 0.0; s = 0.0
    mode = 'PULSE'
    first_step = True
    prev_P_elec = 0.0
    MAX_LAP_TIME = 600.0

    while True:
        s_wrap = s % D_LAP
        grade  = float(_grade_fn(s_wrap))
        elev   = float(_elev_fn(s_wrap))

        # Brake check
        remaining  = D_LAP - s_wrap
        brake_dist = v**2 / (2.0 * A_STOP) + v * DT

        if v > 0 and remaining <= brake_dist and remaining > 0 and mode != 'BRAKE':
            mode = 'BRAKE'

        # Determine P_elec
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

        if not first_step:
            t_lst.append(t); v_lst.append(v); P_lst.append(P_elec)
            s_lst.append(s_wrap); elev_lst.append(elev); grade_lst.append(grade)
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

        t += DT; v = v_new; s = s_new

        if s >= D_LAP or (mode == 'BRAKE' and v == 0.0) or t > MAX_LAP_TIME:
            break

    # Append 4-second stop
    s_end     = float(s_lst[-1])
    elev_end  = float(elev_lst[-1])
    grade_end = float(grade_lst[-1])
    for _ in range(N_STOP_STEPS):
        t_lst.append(t); v_lst.append(0.0); P_lst.append(0.0)
        s_lst.append(s_end); elev_lst.append(elev_end); grade_lst.append(grade_end)
        t += DT

    return t_lst, v_lst, P_lst, s_lst, elev_lst, grade_lst


def build_full_profile(P_PULSE, P_BOOST=P_BOOST_FIXED):
    """Build 11-lap profile. Returns (t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr)."""
    all_t = []; all_v = []; all_P = []; all_s = []
    all_lap = []; all_elev = []; all_grade = []
    t_offset = 0.0

    for lap in range(1, N_LAPS + 1):
        t_l, v_l, P_l, s_l, e_l, g_l = _build_one_lap(P_PULSE, P_BOOST)
        t_arr_lap = np.array(t_l) + t_offset
        all_t.append(t_arr_lap); all_v.append(np.array(v_l))
        all_P.append(np.array(P_l)); all_s.append(np.array(s_l))
        all_lap.append(np.full(len(t_l), lap, dtype=int))
        all_elev.append(np.array(e_l)); all_grade.append(np.array(g_l))
        t_offset = float(t_arr_lap[-1]) + DT

    t_arr       = np.concatenate(all_t)
    v_arr       = np.concatenate(all_v)
    P_arr       = np.concatenate(all_P)
    s_arr       = np.concatenate(all_s)
    lap_num_arr = np.concatenate(all_lap)
    elev_arr    = np.concatenate(all_elev)
    grade_arr   = np.concatenate(all_grade)

    # Smooth velocity (preserve stops)
    v_smooth   = v_arr.copy()
    in_motion  = (v_arr > 0.0)
    m_starts   = np.where(np.diff(np.concatenate([[0], in_motion.astype(int)])) == 1)[0]
    m_ends     = np.where(np.diff(np.concatenate([in_motion.astype(int), [0]])) == -1)[0] + 1
    for ms, me in zip(m_starts, m_ends):
        seg = v_arr[ms:me]
        if len(seg) > 25:
            v_smooth[ms:me] = savgol_filter(seg, window_length=21, polyorder=2)
        v_smooth[ms:me] = np.clip(v_smooth[ms:me], 0.0, V_MAX)
    v_arr = v_smooth

    return t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr


# ── Verification ───────────────────────────────────────────────────────────────

def verify_profile(t_arr, v_arr, P_arr, lap_num_arr, verbose=True):
    """Returns (ok, total_km, duration_min, n_stops)."""
    total_dist_m  = float(np.trapezoid(v_arr, t_arr))
    total_dist_km = total_dist_m / 1000.0
    duration_s    = float(t_arr[-1])
    duration_min  = duration_s / 60.0

    v_zero     = (v_arr == 0.0)
    stop_starts = np.where(np.diff(v_zero.astype(int)) == 1)[0] + 1
    n_stops     = len(stop_starts)

    ok_dist = total_dist_km > 14.5
    ok_time = 34.0 <= duration_min <= 37.0
    ok_stop = n_stops == 11

    if verbose:
        status = "OK" if (ok_dist and ok_time and ok_stop) else "FAIL"
        print(f"    Verify: {total_dist_km:.3f} km | {duration_min:.1f} min | "
              f"{n_stops} stops | {status}")

    return (ok_dist and ok_time and ok_stop), total_dist_km, duration_min, n_stops


# ── Demand array from velocity profile ────────────────────────────────────────

def profile_to_demand(t_arr, v_arr, grade_arr, P_motor_arr):
    """
    Convert velocity profile to electrical demand array.

    Uses the actual P_elec from the P&G motor power (already computed in P_motor_arr).
    But we also need to account for motor efficiency to get true P_elec.
    Since P_motor_arr is already the electrical input power in the P&G builder,
    we use it directly as P_dem.
    """
    return P_motor_arr.copy()


# ── Strategy G factory ────────────────────────────────────────────────────────

def make_strategy_g(K_p, K_i=2.0, tau=15.0, SC_SOC_0_ref=SC_SOC_0):
    alpha = np.exp(-DT / tau)

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        strategy_fn.P_lpf = alpha * strategy_fn.P_lpf + (1 - alpha) * P_dem
        err = SC_SOC_0_ref - SOC
        strategy_fn.integral += err * DT
        P_cmd = strategy_fn.P_lpf + K_p * err + K_i * strategy_fn.integral

        if P_dem < 25.0 and SOC >= SC_SOC_0_ref:
            P_cmd = 0.0

        return float(np.clip(P_cmd, 0.0, FC_P_MAX))

    strategy_fn.P_lpf    = 0.0
    strategy_fn.integral = 0.0
    return strategy_fn


# ── Custom simulator ──────────────────────────────────────────────────────────

def simulate_custom(strategy_fn, P_dem_arr, lap_num_arr, SOC_0=SC_SOC_0):
    """
    Run full race simulation on a pre-built demand array.

    P_dem_arr  : electrical demand at each timestep [W]
    lap_num_arr: 1-indexed lap number per timestep
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

    # Build t_in_lap for each step: reset counter when lap changes
    prev_lap = int(lap_num_arr[0])
    t_in_lap = 0.0
    lap_t_counter = np.empty(n)
    for k in range(n):
        cur_lap = int(lap_num_arr[k])
        if cur_lap != prev_lap:
            t_in_lap = 0.0
            prev_lap = cur_lap
        lap_t_counter[k] = t_in_lap
        t_in_lap += DT

    for k in range(n):
        lap_idx = int(lap_num_arr[k]) - 1   # 0-indexed
        soc_k   = SOC_ar[k]
        Pd      = float(P_dem_arr[k])

        P_fc_cmd = float(strategy_fn(Pd, soc_k, P_fc_prev,
                                     float(lap_t_counter[k]), lap_idx))

        # FC ramp: only limit upward ramp (fc_can_off=True style)
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
        nxt_lap = int(lap_num_arr[k + 1]) if k + 1 < n else int(lap_num_arr[k]) + 1
        if nxt_lap != int(lap_num_arr[k]):
            lap_SOC[int(lap_num_arr[k])] = SOC_ar[k + 1]

    lap_SOC[N_LAPS] = SOC_ar[n]

    return {
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'SOC_final':  float(SOC_ar[n]),
        'lap_SOC':    lap_SOC,
    }


# ── Bisect K_p ────────────────────────────────────────────────────────────────

def bisect_kp(P_dem_arr, lap_num_arr, label='', max_iters=20):
    """
    Bisect K_p in [100, 3000] for |ΔSOC| < 0.01.
    Returns (K_p, result_dict) for best found.
    """
    lo, hi = 100.0, 3000.0
    K_p    = 800.0
    best_result = None
    best_Kp     = K_p

    for i in range(max_iters):
        strat  = make_strategy_g(K_p, K_i=K_I_FIXED, tau=TAU)
        result = simulate_custom(strat, P_dem_arr, lap_num_arr)
        d      = result['delta_SOC']

        if best_result is None or abs(d) < abs(best_result['delta_SOC']):
            best_result = result
            best_Kp     = K_p

        if abs(d) <= TOLERANCE:
            break

        # SOC drifts positive (SC gains charge) → need more FC suppression →
        # raise K_p so error term pushes P_cmd down more when SOC > ref
        # SOC drifts negative (SC depletes) → lower K_p
        if d > 0:
            lo = K_p    # ΔSOC too high (too much charging) → increase K_p
        else:
            hi = K_p    # ΔSOC too low (depleting) → decrease K_p
        K_p = (lo + hi) / 2.0

    return best_Kp, best_result


# ── Motor efficiency stats ────────────────────────────────────────────────────

def motor_efficiency_stats(P_elec_arr):
    """
    Compute mean motor η and active fraction from electrical demand array.

    Returns (mean_eta, active_frac).
    """
    active_mask = P_elec_arr > 10.0
    active_frac = float(np.mean(active_mask))

    if active_mask.sum() == 0:
        return 0.0, 0.0

    # For each active step, iteratively find P_out then η
    etas = np.array([_motor_eta_from_pelec(p) for p in P_elec_arr[active_mask]])
    mean_eta = float(np.mean(etas))
    return mean_eta, active_frac


# ── Sweep function ────────────────────────────────────────────────────────────

def run_sweep(p_pulse_values, label_prefix=''):
    """
    Run the full sweep over a list of P_PULSE values.
    Returns list of result dicts.
    """
    results = []

    print(f"\n{'P_PULSE':>8} {'H2_g':>8} {'km/m³':>8} {'ΔSOC':>8} "
          f"{'K_p':>8} {'η_mean':>8} {'act%':>7} {'dist_km':>8} {'dur_min':>8}")
    print("-" * 85)

    for P_PULSE in p_pulse_values:
        print(f"\n  Building profile P_PULSE={P_PULSE:.0f} W ...")
        t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr = \
            build_full_profile(P_PULSE, P_BOOST=P_BOOST_FIXED)

        ok, total_km, dur_min, n_stops = verify_profile(t_arr, v_arr, P_arr, lap_num_arr)

        if not ok:
            print(f"  {'SKIP':>8} — profile verification failed "
                  f"(dist={total_km:.2f}km, dur={dur_min:.1f}min, stops={n_stops})")
            results.append({
                'P_PULSE': P_PULSE, 'H2_g': np.nan, 'km_m3': np.nan,
                'delta_soc': np.nan, 'K_p': np.nan,
                'mean_eta': np.nan, 'active_frac': np.nan,
                'ok': False, 'total_km': total_km, 'dur_min': dur_min,
                't_arr': t_arr, 'v_arr': v_arr, 'P_arr': P_arr,
                'lap_num_arr': lap_num_arr,
            })
            continue

        # Motor efficiency stats (on per-step P_elec)
        mean_eta, active_frac = motor_efficiency_stats(P_arr)

        # Bisect K_p
        K_p, sim_result = bisect_kp(P_arr, lap_num_arr,
                                     label=f'P_PULSE={P_PULSE:.0f}')
        if sim_result is None:
            print(f"  {'SKIP':>8} — bisection failed")
            continue

        H2_g    = sim_result['m_H2_total']
        delta_s = sim_result['delta_SOC']
        km_m3   = TOTAL_KM / (H2_g / H2_DENSITY) if H2_g > 0 else 0.0

        charge_ok = abs(delta_s) <= TOLERANCE

        rec = {
            'P_PULSE':     P_PULSE,
            'H2_g':        H2_g,
            'km_m3':       km_m3,
            'delta_soc':   delta_s,
            'K_p':         K_p,
            'mean_eta':    mean_eta,
            'active_frac': active_frac,
            'ok':          charge_ok,
            'total_km':    total_km,
            'dur_min':     dur_min,
            't_arr':       t_arr,
            'v_arr':       v_arr,
            'P_arr':       P_arr,
            'lap_num_arr': lap_num_arr,
            'sim_result':  sim_result,
        }
        results.append(rec)

        eta_str   = f"{mean_eta*100:.1f}%"
        act_str   = f"{active_frac*100:.1f}%"
        flag      = "" if charge_ok else " [!charge]"
        print(f"{label_prefix}{P_PULSE:>8.0f} {H2_g:>8.3f} {km_m3:>8.1f} "
              f"{delta_s:>+8.4f} {K_p:>8.1f} {eta_str:>8} {act_str:>7} "
              f"{total_km:>8.3f} {dur_min:>8.1f}{flag}")

    return results


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(MATLAB_DIR,  exist_ok=True)

    # ── Coarse sweep ─────────────────────────────────────────────────────────
    print("=" * 85)
    print("AGENT 2: P_PULSE sweep  (P_BOOST=1000 W fixed, V_HIGH=8.2 m/s, V_LOW=5.83 m/s)")
    print("=" * 85)

    coarse_values = [400, 500, 600, 700, 800, 900, 1000]
    coarse_results = run_sweep(coarse_values)

    # Find best (highest km_m3, charge sustained)
    valid = [r for r in coarse_results if r['ok'] and not np.isnan(r['km_m3'])]
    if not valid:
        # Relax: take best regardless of charge sustenance
        valid = [r for r in coarse_results if not np.isnan(r['km_m3'])]

    if not valid:
        print("ERROR: No valid results from coarse sweep!")
        sys.exit(1)

    best_coarse = max(valid, key=lambda r: r['km_m3'])
    best_P      = best_coarse['P_PULSE']
    print(f"\n  Coarse sweep best: P_PULSE = {best_P:.0f} W  ({best_coarse['km_m3']:.1f} km/m³)")

    # ── Fine sweep ±50 W in 10 W steps ───────────────────────────────────────
    print("\n" + "=" * 85)
    print(f"Fine sweep ±50 W around P_PULSE = {best_P:.0f} W (10 W steps)")
    print("=" * 85)

    fine_lo = max(200, int(best_P) - 50)
    fine_hi = min(1000, int(best_P) + 50)
    fine_values = list(range(fine_lo, fine_hi + 1, 10))
    # Remove values already computed in coarse sweep
    fine_values_new = [p for p in fine_values if p not in coarse_values]

    fine_results = run_sweep(fine_values_new, label_prefix='  ')

    # Combine all
    all_results = coarse_results + fine_results
    valid_all   = [r for r in all_results if r['ok'] and not np.isnan(r['km_m3'])]
    if not valid_all:
        valid_all = [r for r in all_results if not np.isnan(r['km_m3'])]

    best = max(valid_all, key=lambda r: r['km_m3'])

    # ── Print final summary ────────────────────────────────────────────────
    baseline_km_m3 = 302.1
    beats = "YES" if best['km_m3'] > baseline_km_m3 else "NO"

    print()
    print("=== AGENT 2 BEST RESULT ===")
    print(f"P_PULSE = {best['P_PULSE']:.0f} W")
    print(f"P_BOOST = {P_BOOST_FIXED:.0f} W (fixed)")
    print(f"K_p     = {best['K_p']:.1f}")
    print(f"H2      = {best['H2_g']:.3f} g")
    print(f"km/m³   = {best['km_m3']:.1f}")
    print(f"ΔSOC    = {best['delta_soc']:+.4f}")
    print(f"Mean η  = {best['mean_eta']*100:.1f}%")
    print(f"Active  = {best['active_frac']*100:.1f}%")
    print(f"Beats baseline ({baseline_km_m3:.1f} km/m³): {beats}")
    print("===========================")

    # ── Save best CSV ──────────────────────────────────────────────────────
    t_b  = best['t_arr']
    v_b  = best['v_arr']
    P_b  = best['P_arr']

    # Rebuild full profile to get all columns
    t_b, v_b, P_b, s_b, lap_b, elev_b, grade_b = \
        build_full_profile(best['P_PULSE'], P_BOOST=P_BOOST_FIXED)

    df_best = pd.DataFrame({
        'time_s':        t_b,
        'velocity_ms':   v_b,
        'velocity_kmh':  v_b * 3.6,
        'dist_in_lap_m': s_b,
        'lap_num':       lap_b,
        'elevation_m':   elev_b,
        'grade':         grade_b,
        'P_elec_W':      P_b,
    })
    best_csv = os.path.join(MATLAB_DIR, 'sem_agent2_best.csv')
    df_best.to_csv(best_csv, index=False)
    print(f"\nBest profile saved → {best_csv}")

    # ── Plot ───────────────────────────────────────────────────────────────
    # Sort all results by P_PULSE for plotting
    plot_data = sorted([r for r in all_results if not np.isnan(r['km_m3'])],
                        key=lambda r: r['P_PULSE'])
    p_vals  = np.array([r['P_PULSE']    for r in plot_data])
    km_vals = np.array([r['km_m3']      for r in plot_data])
    eta_vals= np.array([r['mean_eta']*100 for r in plot_data])

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
    fig.suptitle(
        f"Hydraix I — P_PULSE Sweep | Silesia Ring SEM 2026\n"
        f"P_BOOST = {P_BOOST_FIXED:.0f} W (fixed), V_HIGH = {V_HIGH:.1f} m/s, "
        f"V_LOW = {V_LOW:.2f} m/s",
        fontsize=12, fontweight='bold'
    )

    # Panel 1: km/m³ vs P_PULSE
    ax1.plot(p_vals, km_vals, 'o-', color='royalblue', linewidth=2, markersize=6)
    ax1.axhline(baseline_km_m3, color='red', linestyle='--', linewidth=1.2,
                label=f'Baseline (Strategy G): {baseline_km_m3:.1f} km/m³')
    ax1.axvline(best['P_PULSE'], color='green', linestyle=':', linewidth=1.5,
                label=f'Best P_PULSE = {best["P_PULSE"]:.0f} W ({best["km_m3"]:.1f} km/m³)')
    # Mark invalid (charge not sustained) with different marker
    for r in all_results:
        if not np.isnan(r['km_m3']) and not r['ok']:
            ax1.plot(r['P_PULSE'], r['km_m3'], 'rx', markersize=10, markeredgewidth=2,
                     label='Charge not sustained' if r == [rr for rr in all_results
                                                           if not rr['ok'] and
                                                           not np.isnan(rr['km_m3'])][0]
                     else '')
    ax1.set_ylabel('km / m³', fontsize=11)
    ax1.set_title('Fuel Economy vs Motor Pulse Power', fontsize=10, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.4)
    ax1.set_xlabel('P_PULSE [W]')

    # Panel 2: Mean motor η vs P_PULSE
    ax2.plot(p_vals, eta_vals, 's-', color='darkorange', linewidth=2, markersize=6)
    ax2.axvline(best['P_PULSE'], color='green', linestyle=':', linewidth=1.5,
                label=f'Best P_PULSE = {best["P_PULSE"]:.0f} W')
    ax2.set_ylabel('Mean Motor η [%]', fontsize=11)
    ax2.set_xlabel('P_PULSE [W]')
    ax2.set_title('Mean Motor Efficiency (active timesteps) vs P_PULSE',
                  fontsize=10, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.4)

    # Panel 3: Single-lap velocity of best combo
    # Use first lap of best profile
    lap1_mask = lap_b == 1
    t1 = t_b[lap1_mask]
    v1 = v_b[lap1_mask] * 3.6   # km/h
    P1 = P_b[lap1_mask]

    ax3.fill_between(t1, v1, alpha=0.15, color='royalblue')
    ax3.plot(t1, v1, color='royalblue', linewidth=1.2, label='Velocity (km/h)')
    # Mark motor-on regions
    motor_on = P1 > 10.0
    ax3.fill_between(t1, 0, v1, where=motor_on, alpha=0.3, color='darkorange',
                     label='Motor ON')
    ax3.axhline(V_HIGH * 3.6, color='green', linestyle='--', linewidth=1.0,
                label=f'V_HIGH = {V_HIGH*3.6:.0f} km/h')
    ax3.axhline(V_LOW * 3.6,  color='red',   linestyle='--', linewidth=1.0,
                label=f'V_LOW = {V_LOW*3.6:.0f} km/h')
    ax3.set_ylabel('Velocity [km/h]', fontsize=11)
    ax3.set_xlabel('Time [s]', fontsize=11)
    ax3.set_title(
        f'Single-Lap Velocity Profile — Best: P_PULSE = {best["P_PULSE"]:.0f} W  '
        f'(km/m³ = {best["km_m3"]:.1f})',
        fontsize=10, fontweight='bold'
    )
    ax3.legend(fontsize=9, loc='upper right')
    ax3.grid(True, alpha=0.4)

    plt.tight_layout()
    result_png = os.path.join(RESULTS_DIR, 'agent2_power_sweep_result.png')
    fig.savefig(result_png, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Result plot saved → {result_png}")

    # ── Tabulated summary ──────────────────────────────────────────────────
    print("\n── Full Sweep Summary ────────────────────────────────────────────────")
    print(f"{'P_PULSE':>8} {'H2_g':>8} {'km/m³':>8} {'ΔSOC':>9} "
          f"{'K_p':>8} {'η_mean':>8} {'act%':>7} {'OK':>4}")
    print("-" * 72)
    for r in sorted(all_results, key=lambda x: x['P_PULSE']):
        if np.isnan(r['km_m3']):
            print(f"{r['P_PULSE']:>8.0f} {'—':>8} {'—':>8} {'—':>9} "
                  f"{'—':>8} {'—':>8} {'—':>7} SKIP")
        else:
            print(f"{r['P_PULSE']:>8.0f} {r['H2_g']:>8.3f} {r['km_m3']:>8.1f} "
                  f"{r['delta_soc']:>+9.4f} {r['K_p']:>8.1f} "
                  f"{r['mean_eta']*100:>7.1f}% {r['active_frac']*100:>6.1f}% "
                  f"{'YES' if r['ok'] else 'NO':>4}")
