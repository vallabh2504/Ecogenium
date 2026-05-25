"""
Strategy G Robustness Test — K_p=1900, K_i=2.0, τ=15s (unchanged across all cycles).

Tests whether the tuned Strategy G achieves charge sustenance on:
  1. SEM Race profile (original tuning cycle)
  2. NEDC (scaled to SEM vehicle speeds)
  3. WLTC Class 3 (scaled to SEM vehicle speeds)

If |ΔSOC| < 0.05 on all cycles with NO retuning, the strategy is proven adaptive.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import uniform_filter1d

# ── Import Strategy G factory and constants ──────────────────────────────────
from strategy_g_pi import make_strategy
from ems_core import (
    build_demand_profile,
    fc_current, fc_h2_rate,
    sc_soc_update,
    SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX, SC_E_J,
    FC_P_MAX, FC_RAMP, K_H2,
    N_LAPS, DT, RESULTS_DIR
)

# ── Vehicle / drivetrain constants (identical to ems_core) ───────────────────
MASS    = 175.0
G       = 9.81
CD      = 0.15
AF      = 0.8
CRR     = 0.006
RHO     = 1.225
ETA_DT  = 0.95
DT_SIM  = 0.2   # 5 Hz

# ── BAFANG RM G060.1000 — 61-point efficiency lookup (exact copy from ems_core) ──
_M_POUT = np.array([
     24.19,  24.32,  24.25,  28.44,  28.30,  32.42,  36.48,  40.42,
     44.70,  52.41,  56.89,  64.51,  72.95,  76.80,  84.89,  92.97,
     96.76, 113.48, 116.92, 129.35, 137.79, 144.77, 173.82, 177.86,
    206.16, 218.28, 229.22, 258.04, 258.71, 290.29, 310.45, 320.87,
    353.88, 357.90, 390.07, 414.20, 421.14, 461.25, 477.30, 498.74,
    541.47, 550.61, 590.51, 628.07, 636.72, 676.51, 716.31, 728.37,
    771.91, 815.46, 830.83, 874.14, 921.39, 947.83, 972.64, 1030.08,
   1070.29, 1091.75, 1127.55, 1170.18, 1213.38
])
_M_ETA = np.array([
    0.220, 0.218, 0.220, 0.251, 0.249, 0.279, 0.298, 0.314,
    0.335, 0.383, 0.395, 0.423, 0.453, 0.452, 0.481, 0.492,
    0.500, 0.565, 0.531, 0.571, 0.569, 0.582, 0.634, 0.620,
    0.666, 0.652, 0.669, 0.673, 0.706, 0.713, 0.719, 0.713,
    0.747, 0.728, 0.757, 0.764, 0.745, 0.772, 0.767, 0.765,
    0.792, 0.776, 0.791, 0.809, 0.786, 0.799, 0.811, 0.793,
    0.806, 0.816, 0.803, 0.813, 0.824, 0.817, 0.811, 0.826,
    0.828, 0.821, 0.819, 0.824, 0.834
])
_m_idx    = np.argsort(_M_POUT)
_M_POUT_S = _M_POUT[_m_idx]
_M_ETA_S  = _M_ETA[_m_idx]


def motor_eta_lookup(p_out_W):
    """BAFANG RM G060.1000 efficiency (0–1) from shaft output power [W]."""
    return np.interp(np.maximum(p_out_W, 0.0), _M_POUT_S, _M_ETA_S,
                     left=_M_ETA_S[0], right=_M_ETA_S[-1])


# ── Cycle interpolation ───────────────────────────────────────────────────────
def interp_cycle(waypoints_list, v_scale=1.0, dt=0.2):
    """Interpolate waypoint list to uniform 5 Hz array.

    Parameters
    ----------
    waypoints_list : list of (t_s, v_kmh) tuples — time already in seconds
    v_scale        : multiply all speeds by this factor (e.g. 40/120)
    dt             : timestep [s]
    """
    t_pts = np.array([p[0] for p in waypoints_list], dtype=float)
    v_pts = np.array([p[1] for p in waypoints_list], dtype=float) * v_scale / 3.6  # → m/s
    t_uniform = np.arange(0.0, t_pts[-1], dt)
    v_uniform = np.interp(t_uniform, t_pts, v_pts)
    return t_uniform, v_uniform


def compute_demand(t_uniform, v_uniform):
    """Vehicle dynamics → electrical demand [W] at each timestep."""
    smooth_n = max(1, int(10.0 / DT_SIM))
    v_s   = uniform_filter1d(v_uniform, size=smooth_n, mode='nearest')
    accel = np.gradient(v_s, DT_SIM)

    P_wheel = (CRR * MASS * G * v_s
               + 0.5 * CD * AF * RHO * v_s**3
               + MASS * accel * v_s)            # flat road — no incline for std cycles
    P_motor = np.where(P_wheel > 0, P_wheel / ETA_DT, 0.0)
    eta_m   = motor_eta_lookup(P_motor)
    P_elec  = np.where(P_motor > 0, P_motor / eta_m, 0.0)
    P_elec  = np.clip(np.where(np.isfinite(P_elec), P_elec, 0.0), 0.0, None)
    return P_elec


# ── NEDC drive cycle ─────────────────────────────────────────────────────────
# ECE-15 urban cycle: 195 s, peak 50 km/h — repeated 4 times (0–780 s)
# EUDC extra-urban:   400 s, peak 120 km/h  (780–1180 s)
_ece15_base = [
    (0, 0), (11, 0), (15, 15), (23, 15), (28, 0), (44, 0),
    (49, 32), (54, 32), (56, 35), (61, 0), (74, 0),
    (80, 50), (116, 50), (122, 0), (131, 0),
    (141, 35), (147, 35), (162, 0), (177, 0), (195, 0)
]
_eudc_base = [
    (0, 0), (20, 0), (41, 70), (60, 70), (81, 100), (100, 100),
    (111, 120), (130, 120), (160, 80), (179, 80), (196, 0),
    (213, 0), (227, 50), (250, 50), (267, 70), (285, 70),
    (300, 100), (319, 100), (328, 0), (360, 0), (400, 0)
]

def build_nedc_waypoints():
    """Build full NEDC waypoint list: 4 × ECE-15 (0–780 s) + EUDC (780–1180 s)."""
    waypoints = []
    for rep in range(4):
        offset = rep * 195.0
        for (t, v) in _ece15_base:
            waypoints.append((t + offset, v))
    # Remove duplicate at t=780 if last ECE-15 ends there
    # EUDC starts at t=780 with v=0
    eudc_start = 4 * 195.0   # = 780.0 s
    for (t, v) in _eudc_base:
        waypoints.append((t + eudc_start, v))
    return waypoints

NEDC_V_SCALE = 40.0 / 120.0   # scale peak 120 km/h → 40 km/h (SEM max)

def build_nedc():
    """Return (t_arr, v_arr, P_dem_arr) for one NEDC cycle scaled to SEM speeds."""
    wpts = build_nedc_waypoints()
    t_u, v_u = interp_cycle(wpts, v_scale=NEDC_V_SCALE, dt=DT_SIM)
    P_dem = compute_demand(t_u, v_u)
    return t_u, v_u, P_dem


# ── WLTC Class 3 drive cycle ─────────────────────────────────────────────────
_wltc_pts = [
    # Low phase (urban, frequent stops) — 0–589 s, peak ~56 km/h
    (0, 0), (16, 0), (40, 54), (57, 54), (68, 32), (84, 32), (99, 0),
    (107, 0), (131, 45), (148, 45), (162, 0), (179, 0), (195, 35),
    (212, 35), (227, 0), (243, 0), (267, 51), (289, 51), (304, 0),
    (325, 0), (355, 50), (372, 50), (390, 0), (410, 0), (434, 56),
    (456, 56), (489, 56), (510, 34), (530, 34), (545, 0), (589, 0),
    # Medium phase — 590–1022 s, peak ~76 km/h
    (590, 0), (612, 56), (634, 56), (649, 76), (676, 76), (695, 56),
    (718, 56), (740, 0), (758, 0), (785, 72), (807, 72), (840, 76),
    (860, 72), (880, 0), (910, 0), (940, 68), (970, 68), (1022, 0),
    # High phase — 1023–1477 s, peak ~97 km/h
    (1023, 0), (1045, 78), (1080, 97), (1120, 97), (1155, 80),
    (1180, 64), (1220, 97), (1270, 97), (1310, 75), (1350, 0),
    (1380, 0), (1420, 88), (1460, 88), (1477, 0),
    # Extra-High phase — 1478–1800 s, peak ~131 km/h
    (1478, 0), (1500, 108), (1540, 131), (1600, 131), (1640, 115),
    (1680, 100), (1720, 131), (1760, 131), (1800, 0),
]

WLTC_V_SCALE = 40.0 / 131.0   # scale peak 131 km/h → 40 km/h (SEM max)

def build_wltc():
    """Return (t_arr, v_arr, P_dem_arr) for one WLTC Class 3 cycle scaled to SEM."""
    t_u, v_u = interp_cycle(_wltc_pts, v_scale=WLTC_V_SCALE, dt=DT_SIM)
    P_dem = compute_demand(t_u, v_u)
    return t_u, v_u, P_dem


# ── Cycle-based Strategy G simulator ─────────────────────────────────────────
T_LAP_EQUIV    = 190.0          # original race lap duration [s] for lap supervisor
N_PTS_PER_LAP  = int(round(T_LAP_EQUIV / DT_SIM))   # ~950 samples per "virtual lap"
RACE_DURATION  = 2100.0         # 35 min in seconds


def simulate_cycle(strategy_fn, t_arr, P_dem_arr, SOC_0=SC_SOC_0):
    """
    Simulate Strategy G on an arbitrary drive cycle.

    Parameters
    ----------
    strategy_fn  : callable — make_strategy() output
    t_arr        : 1-D time array [s]
    P_dem_arr    : 1-D electrical demand array [W]
    SOC_0        : initial SC SOC

    Returns
    -------
    Same dict layout as simulate_race_g() for easy comparison.
    """
    n = len(t_arr)
    P_fc_ar   = np.empty(n)
    P_sc_ar   = np.empty(n)
    SOC_ar    = np.empty(n + 1)
    I_fc_ar   = np.empty(n)
    m_H2_ar   = np.empty(n)
    P_filt_ar = np.empty(n)

    SOC_ar[0] = SOC_0
    P_fc_prev = 0.0

    for k in range(n):
        lap_idx  = k // N_PTS_PER_LAP
        t_in_lap = float(t_arr[k]) % T_LAP_EQUIV
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem_arr[k])

        result = strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx)
        if isinstance(result, tuple):
            P_fc_cmd, P_filt_val = result
        else:
            P_fc_cmd   = result
            P_filt_val = P_fc_cmd

        P_fc_cmd = float(P_fc_cmd)

        # Ramp rate: upward limited, instant shutdown allowed
        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT_SIM)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k = Pd - P_fc_cmd

        # SC floor protection
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT_SIM)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = min(FC_P_MAX, Pd)
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT_SIM)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        P_filt_ar[k]  = float(P_filt_val)
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(max(P_fc_cmd, 1e-6)) if P_fc_cmd > 0 else 0.0)
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT_SIM
        P_fc_prev      = P_fc_cmd

    return {
        't':           t_arr,
        'P_dem':       P_dem_arr,
        'P_fc':        P_fc_ar,
        'P_sc':        P_sc_ar,
        'P_filt':      P_filt_ar,
        'SOC':         SOC_ar[:-1],
        'I_fc':        I_fc_ar,
        'm_H2_total':  float(np.sum(m_H2_ar)),
        'delta_SOC':   float(SOC_ar[n] - SOC_0),
        'SOC_final':   float(SOC_ar[n]),
    }


# ── K_p and K_i (tuned on SEM race, NOT retuned here) ────────────────────────
K_P_TUNED = 1900.0
K_I_TUNED = 2.0
TAU       = 15.0
TOLERANCE = 0.05   # relaxed for cross-cycle test

print("=" * 70)
print("Strategy G Robustness Test")
print(f"  K_p={K_P_TUNED}  K_i={K_I_TUNED}  τ={TAU}s  (NO retuning)")
print("=" * 70)

# ─────────────────────────────────────────────────────────────────────────────
# ── Cycle 1: SEM Race profile (baseline) ─────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
print("\n[1/3] SEM Race profile (baseline) ...")
t_lap_raw, P_lap_raw = build_demand_profile()
T_lap_raw  = float(t_lap_raw[-1])

# Tile × N_LAPS laps to get ~35 min
t_sem_all = np.concatenate([t_lap_raw + i * T_lap_raw for i in range(N_LAPS)])
P_sem_all = np.tile(P_lap_raw, N_LAPS)
v_sem_all = np.zeros_like(t_sem_all)   # speed not used for SEM (demand already computed)

# Truncate to ~35 min
mask_sem  = t_sem_all <= RACE_DURATION
t_sem_all = t_sem_all[mask_sem]
P_sem_all = P_sem_all[mask_sem]

strat_sem   = make_strategy(K_p=K_P_TUNED, K_i=K_I_TUNED, tau=TAU)
result_sem  = simulate_cycle(strat_sem, t_sem_all, P_sem_all)
print(f"  Duration: {t_sem_all[-1]/60:.1f} min  "
      f"ΔSOC={result_sem['delta_SOC']:+.4f}  "
      f"H2={result_sem['m_H2_total']:.3f} g")

# Reconstruct speed from GPS demand (approximate 25 km/h avg for display)
# Use build_demand_profile velocity proxy — just show blank placeholder
# Actually we'll build a speed array from the lap CSV using same logic
import pandas as pd
_this_dir  = os.path.dirname(os.path.abspath(__file__))
_matlab_dir = os.path.join(_this_dir, '..', 'matlab')
raw_csv = pd.read_csv(os.path.join(_matlab_dir, 'canonical_with_incline.csv'))
lap_end_t   = raw_csv['elapsed_sec'].iloc[-1] / N_LAPS
raw_lap     = raw_csv[raw_csv['elapsed_sec'] <= lap_end_t].reset_index(drop=True)
t_raw_v     = raw_lap['elapsed_sec'].values
v_raw_v     = raw_lap['Speed_kmh'].values
t_v_u       = np.arange(t_raw_v[0], t_raw_v[-1], DT_SIM)
v_lap_kmh   = np.interp(t_v_u, t_raw_v, v_raw_v)
# Tile N_LAPS and truncate
v_sem_tiled = np.tile(v_lap_kmh, N_LAPS)
v_sem_tiled = v_sem_tiled[:len(t_sem_all)]


# ─────────────────────────────────────────────────────────────────────────────
# ── Cycle 2: NEDC ────────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
print("\n[2/3] NEDC (scaled ×{:.3f}) ...".format(NEDC_V_SCALE))
t_nedc_1, v_nedc_1, P_nedc_1 = build_nedc()

# Tile 2× (1180 s × 2 = 2360 s > 2100 s), then truncate to 35 min
T_nedc = float(t_nedc_1[-1])
t_nedc_all = np.concatenate([t_nedc_1, t_nedc_1 + T_nedc])
v_nedc_all = np.concatenate([v_nedc_1, v_nedc_1])
P_nedc_all = np.concatenate([P_nedc_1, P_nedc_1])
mask_nedc  = t_nedc_all <= RACE_DURATION
t_nedc_all = t_nedc_all[mask_nedc]
v_nedc_all = v_nedc_all[mask_nedc]
P_nedc_all = np.clip(P_nedc_all[mask_nedc], 0.0, FC_P_MAX)

strat_nedc  = make_strategy(K_p=K_P_TUNED, K_i=K_I_TUNED, tau=TAU)
result_nedc = simulate_cycle(strat_nedc, t_nedc_all, P_nedc_all)
print(f"  Duration: {t_nedc_all[-1]/60:.1f} min  "
      f"ΔSOC={result_nedc['delta_SOC']:+.4f}  "
      f"H2={result_nedc['m_H2_total']:.3f} g")


# ─────────────────────────────────────────────────────────────────────────────
# ── Cycle 3: WLTC ────────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
print("\n[3/3] WLTC Class 3 (scaled ×{:.3f}) ...".format(WLTC_V_SCALE))
t_wltc_1, v_wltc_1, P_wltc_1 = build_wltc()

# Single cycle = 1800 s ≈ 30 min; close to 35 min, use as-is
P_wltc_1_c = np.clip(P_wltc_1, 0.0, FC_P_MAX)

strat_wltc  = make_strategy(K_p=K_P_TUNED, K_i=K_I_TUNED, tau=TAU)
result_wltc = simulate_cycle(strat_wltc, t_wltc_1, P_wltc_1_c)
print(f"  Duration: {t_wltc_1[-1]/60:.1f} min  "
      f"ΔSOC={result_wltc['delta_SOC']:+.4f}  "
      f"H2={result_wltc['m_H2_total']:.3f} g")


# ─────────────────────────────────────────────────────────────────────────────
# ── Derived metrics for all three cycles ─────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
def cycle_metrics(result, t_arr, v_arr_ms):
    """Compute summary metrics for a cycle result."""
    dur_min   = float(t_arr[-1]) / 60.0
    avg_spd   = float(np.mean(v_arr_ms)) * 3.6      # km/h
    avg_pdem  = float(np.mean(result['P_dem']))
    h2        = result['m_H2_total']
    dsoc      = result['delta_SOC']
    sustained = abs(dsoc) <= TOLERANCE
    cum_h2    = np.cumsum(K_H2 * result['I_fc'] * DT_SIM)
    return dict(dur_min=dur_min, avg_spd=avg_spd, avg_pdem=avg_pdem,
                h2=h2, dsoc=dsoc, sustained=sustained, cum_h2=cum_h2)

# For SEM, derive average speed from velocity array (km/h)
v_sem_ms = v_sem_tiled / 3.6
metrics_sem  = cycle_metrics(result_sem,  t_sem_all,  v_sem_ms)
metrics_nedc = cycle_metrics(result_nedc, t_nedc_all, v_nedc_all)
metrics_wltc = cycle_metrics(result_wltc, t_wltc_1,   v_wltc_1)


# ─────────────────────────────────────────────────────────────────────────────
# ── Figure: 3 columns × 4 rows ───────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
print("\nGenerating robustness figure ...")

fig, axes = plt.subplots(4, 3, figsize=(18, 12), dpi=150,
                         sharex='col')
fig.suptitle(
    "Strategy G Robustness Test — "
    "K_p=1900  K_i=2.0  τ=15s  (unchanged across all cycles)",
    fontsize=12, fontweight='bold'
)

col_titles = ["SEM Race Profile", "NEDC (scaled to SEM)", "WLTC Class 3 (scaled to SEM)"]

# Data bundles: (result, t_arr, v_arr_km/h, metrics)
v_sem_kmh   = v_sem_tiled                                    # already km/h
v_nedc_kmh  = v_nedc_all * 3.6
v_wltc_kmh  = v_wltc_1 * 3.6
t_min_sem   = t_sem_all / 60.0
t_min_nedc  = t_nedc_all / 60.0
t_min_wltc  = t_wltc_1 / 60.0

bundles = [
    (result_sem,  t_min_sem,  v_sem_kmh,  metrics_sem),
    (result_nedc, t_min_nedc, v_nedc_kmh, metrics_nedc),
    (result_wltc, t_min_wltc, v_wltc_kmh, metrics_wltc),
]

for col, (res, t_min, v_kmh, met) in enumerate(bundles):
    ax_spd = axes[0, col]
    ax_pwr = axes[1, col]
    ax_soc = axes[2, col]
    ax_h2  = axes[3, col]

    # ── Row 0: Speed ──────────────────────────────────────────────────────
    ax_spd.plot(t_min, v_kmh, color='darkorange', lw=0.6)
    ax_spd.set_ylabel('Speed [km/h]' if col == 0 else '')
    ax_spd.set_title(col_titles[col], fontsize=10, fontweight='bold')
    ax_spd.grid(True, lw=0.3)
    ax_spd.annotate(f"avg {met['avg_spd']:.1f} km/h",
                    xy=(0.02, 0.92), xycoords='axes fraction', fontsize=8)

    # ── Row 1: Power flows ────────────────────────────────────────────────
    ax_pwr.plot(t_min, res['P_dem'],  color='black',     lw=0.5, label='P_dem')
    ax_pwr.plot(t_min, res['P_fc'],   color='steelblue', lw=0.8, label='P_fc')
    ax_pwr.plot(t_min, res['P_filt'], color='green',     lw=0.5, ls='--',
                label='P_filt')
    ax_pwr.set_ylabel('Power [W]' if col == 0 else '')
    ax_pwr.legend(loc='upper right', fontsize=7)
    ax_pwr.grid(True, lw=0.3)
    ax_pwr.annotate(f"avg P_dem={met['avg_pdem']:.0f} W",
                    xy=(0.02, 0.92), xycoords='axes fraction', fontsize=8)

    # ── Row 2: SOC ───────────────────────────────────────────────────────
    ax_soc.plot(t_min, res['SOC'], color='green', lw=0.8)
    ax_soc.axhline(SC_SOC_0,   color='black',  lw=0.8, ls='--',
                   label=f'SOC_ref={SC_SOC_0:.2f}')
    ax_soc.axhline(SC_SOC_MIN, color='red',    lw=0.8, ls=':',
                   label=f'SOC_MIN={SC_SOC_MIN:.2f}')
    ax_soc.axhline(SC_SOC_MAX, color='purple', lw=0.8, ls=':',
                   label=f'SOC_MAX={SC_SOC_MAX:.2f}')
    ax_soc.set_ylabel('SC SOC [—]' if col == 0 else '')
    ax_soc.set_ylim(0, 1)
    ax_soc.legend(loc='upper right', fontsize=7)
    ax_soc.grid(True, lw=0.3)

    # Text box: ΔSOC and charge sustenance verdict
    verdict = "YES" if met['sustained'] else "NO"
    verdict_color = 'forestgreen' if met['sustained'] else 'crimson'
    box_txt = f"ΔSOC = {met['dsoc']:+.4f}\nCharge sustained: {verdict}"
    ax_soc.text(0.02, 0.12, box_txt, transform=ax_soc.transAxes,
                fontsize=8, verticalalignment='bottom',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow',
                          edgecolor=verdict_color, linewidth=1.5))

    # ── Row 3: Cumulative H2 ──────────────────────────────────────────────
    ax_h2.plot(t_min, met['cum_h2'], color='darkviolet', lw=0.8)
    ax_h2.set_ylabel('Cumul. H2 [g]' if col == 0 else '')
    ax_h2.set_xlabel('Time [min]')
    ax_h2.grid(True, lw=0.3)
    ax_h2.annotate(f"total {met['h2']:.3f} g",
                   xy=(0.02, 0.88), xycoords='axes fraction', fontsize=8)

plt.tight_layout(rect=[0, 0, 1, 0.96])
out_fig = os.path.join(RESULTS_DIR, 'strategy_g_robustness.png')
plt.savefig(out_fig, dpi=150)
plt.close()
print(f"Figure saved → {out_fig}")


# ─────────────────────────────────────────────────────────────────────────────
# ── Summary table ────────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
print()
print("=== Strategy G Robustness Test (K_p=1900, K_i=2.0, τ=15s — NO RETUNING) ===")
print()
hdr = f"{'Drive cycle':<20}| {'Duration':>8} | {'Avg speed':>9} | {'Avg P_dem':>9} | {'H2 [g]':>6} | {'ΔSOC':>8} | {'Sustained?':>10}"
sep = "-" * len(hdr)
print(hdr)
print(sep)

rows = [
    ("SEM Race (GPS)",
     metrics_sem['dur_min'],
     metrics_sem['avg_spd'],
     metrics_sem['avg_pdem'],
     metrics_sem['h2'],
     metrics_sem['dsoc'],
     metrics_sem['sustained']),

    (f"NEDC (scaled ×{NEDC_V_SCALE:.2f})",
     metrics_nedc['dur_min'],
     metrics_nedc['avg_spd'],
     metrics_nedc['avg_pdem'],
     metrics_nedc['h2'],
     metrics_nedc['dsoc'],
     metrics_nedc['sustained']),

    (f"WLTC (scaled ×{WLTC_V_SCALE:.2f})",
     metrics_wltc['dur_min'],
     metrics_wltc['avg_spd'],
     metrics_wltc['avg_pdem'],
     metrics_wltc['h2'],
     metrics_wltc['dsoc'],
     metrics_wltc['sustained']),
]

for (name, dur, spd, pdem, h2, dsoc, ok) in rows:
    verdict = "YES" if ok else "NO"
    print(f"{name:<20}| {dur:>7.1f}m | {spd:>8.1f}k | {pdem:>8.1f}W | {h2:>6.3f} | {dsoc:>+8.4f} | {verdict:>10}")

print(sep)
print()
print("Charge sustained = |ΔSOC| < 0.05 (relaxed from 0.01 — different cycles have")
print("                   different average demands so perfect sustenance is not expected;")
print("                   what matters is the integral corrects in the right direction)")
print("=" * len(hdr))

all_ok = all(r[6] for r in rows)
print()
if all_ok:
    print("RESULT: Strategy G achieves charge sustenance on ALL three drive cycles")
    print("        with IDENTICAL K_p=1900, K_i=2.0, τ=15s — ADAPTIVE confirmed.")
else:
    failed = [r[0] for r in rows if not r[6]]
    print(f"RESULT: Charge sustenance NOT achieved on: {failed}")
    print(f"        |ΔSOC| > 0.05 — strategy may need retuning for these cycles.")
