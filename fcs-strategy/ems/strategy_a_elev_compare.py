"""
Strategy A (Heuristic 2D LUT) on the elevation-aware P&G profile.

P_base is re-tuned to the P&G profile's mean demand (~131.8 W) and K_soc
is bisected for charge sustenance (|ΔSOC| < 0.01).

Outputs
-------
  results/ems/strategy_a_elev_comparison.png   — 4-panel time-series
  Console: K_soc, ΔSOC, H2, km/m³
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_E_J, SC_SOC_0, SC_SOC_MIN,
    K_H2, fc_current, fc_h2_rate, sc_soc_update,
    N_LAPS, DT, MATLAB_DIR, RESULTS_DIR,
)

TOTAL_KM   = 14.5
H2_DENSITY = 89.88   # g/m³ at STP

# ── Load P&G demand profile ────────────────────────────────────────────────────
df          = pd.read_csv(os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv'))
t_all       = df['time_s'].values
P_dem_all   = df['P_elec_W'].values
lap_num_all = df['lap_num'].values

MEAN_P_DEM = float(np.mean(P_dem_all))
RACE_DUR_S = float(t_all[-1])

# Pre-compute t_in_lap for every sample
_lap_t0 = np.array([t_all[lap_num_all == ln][0] for ln in range(1, N_LAPS + 1)])
t_in_lap_all = t_all - _lap_t0[(lap_num_all - 1)]

print(f"P&G profile loaded: {len(df)} samples")
print(f"  Mean P_dem  = {MEAN_P_DEM:.1f} W  (canonical = 259.0 W)")
print(f"  Race dur    = {RACE_DUR_S/60:.1f} min")


# ── Strategy A: 2D LUT with P_base matched to P&G mean demand ─────────────────
def make_strategy(K_soc):
    """
    2D LUT identical to the original Strategy A, but P_base = MEAN_P_DEM so
    the table's neutral-state FC output matches the P&G profile's average demand.
    """
    u_grid   = np.linspace(-0.050, 0.050, 21)
    soc_grid = np.linspace(0.10,   0.95,  18)
    K_rate   = 500.0

    TABLE = np.zeros((21, 18))
    for i, u_i in enumerate(u_grid):
        for j, soc_j in enumerate(soc_grid):
            delta_soc  = K_soc * (SC_SOC_0 - soc_j)
            delta_rate = (K_rate * (abs(u_i) - 0.020) * np.sign(u_i)
                          if abs(u_i) > 0.020 else 0.0)
            TABLE[i, j] = np.clip(MEAN_P_DEM + delta_soc + delta_rate,
                                  FC_P_MIN, FC_P_MAX)

    _interp = RegularGridInterpolator(
        (u_grid, soc_grid), TABLE,
        method='linear', bounds_error=False, fill_value=None,
    )

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        u = (P_dem - P_fc_prev) / SC_E_J
        return float(_interp([[u, SOC]])[0])

    return strategy_fn


# ── Custom simulator (uses P&G demand array directly) ─────────────────────────
def simulate(strategy_fn, SOC_0=SC_SOC_0):
    n = len(P_dem_all)
    P_fc_ar = np.empty(n)
    P_sc_ar = np.empty(n)
    SOC_ar  = np.empty(n + 1)
    I_fc_ar = np.empty(n)
    m_H2_ar = np.empty(n)

    lap_SOC    = np.empty(N_LAPS + 1)
    lap_SOC[0] = SOC_0
    SOC_ar[0]  = SOC_0
    P_fc_prev  = FC_P_MIN
    prev_lap   = int(lap_num_all[0])

    for k in range(n):
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem_all[k])
        lap_idx  = int(lap_num_all[k]) - 1   # 0-based
        t_in_lap = float(t_in_lap_all[k])

        P_fc_cmd = float(strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx))

        # Symmetric ramp-rate limit
        P_fc_cmd = float(np.clip(P_fc_cmd,
                                  P_fc_prev - FC_RAMP * DT,
                                  P_fc_prev + FC_RAMP * DT))
        P_fc_cmd = float(np.clip(P_fc_cmd, FC_P_MIN, FC_P_MAX))

        P_sc_k = Pd - P_fc_cmd

        # SC floor protection
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = float(np.clip(Pd, FC_P_MIN, FC_P_MAX))
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(P_fc_cmd))
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev     = P_fc_cmd

        curr_lap = int(lap_num_all[k])
        if curr_lap != prev_lap:
            lap_SOC[prev_lap] = SOC_ar[k]
            prev_lap = curr_lap

    lap_SOC[N_LAPS] = float(SOC_ar[n])

    return {
        't':          t_all,
        'P_dem':      P_dem_all,
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'lap_SOC':    lap_SOC,
    }


# ── Bisect K_soc for charge sustenance ────────────────────────────────────────
TOLERANCE = 0.01
lo, hi    = 50.0, 800.0
K_soc     = (lo + hi) / 2
best      = None

print(f"\n── Bisecting K_soc for |ΔSOC| < {TOLERANCE} ──")
for it in range(25):
    result    = simulate(make_strategy(K_soc))
    delta_soc = result['delta_SOC']
    h2        = result['m_H2_total']
    print(f"  iter {it+1:2d}  K_soc={K_soc:6.1f}  ΔSOC={delta_soc:+.4f}  H2={h2:.3f} g")

    if abs(delta_soc) <= TOLERANCE:
        best = result
        break

    if delta_soc < -TOLERANCE:
        lo = K_soc
    else:
        hi = K_soc
    K_soc = (lo + hi) / 2
    best  = result

if best is None:
    best = result

# ── Results ────────────────────────────────────────────────────────────────────
m_H2      = best['m_H2_total']
delta_soc = best['delta_SOC']
km_m3     = TOTAL_KM / (m_H2 / H2_DENSITY)
t_min     = best['t'] / 60.0
cum_H2    = np.cumsum(K_H2 * best['I_fc'] * DT)

print(f"\n── Strategy A results on P&G profile ──────────────────────────────")
print(f"  Tuned K_soc    : {K_soc:.1f}  W per unit SOC")
print(f"  P_base         : {MEAN_P_DEM:.1f}  W  (= mean P_dem of P&G profile)")
print(f"  H2 consumed    : {m_H2:.3f} g")
print(f"  km/m³          : {km_m3:.1f}")
print(f"  Final ΔSOC     : {delta_soc:+.4f}  (target |ΔSOC| < 0.01)")
print(f"  Charge sustained: {'YES' if abs(delta_soc) <= 0.01 else 'NO'}")
print(f"  Mean P_fc      : {float(np.mean(best['P_fc'])):.1f} W")
print(f"  Race duration  : {RACE_DUR_S/60:.1f} min")
print(f"──────────────────────────────────────────────────────────────────")

# ── Plot ───────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
fig.suptitle(
    f"Strategy A (2D LUT) — Elevation-Aware P&G Profile  |  "
    f"P_base={MEAN_P_DEM:.0f} W  |  K_soc={K_soc:.0f}  |  "
    f"ΔSOC={delta_soc:+.4f}  |  H2={m_H2:.3f} g  |  {km_m3:.0f} km/m³",
    fontsize=10, fontweight='bold',
)

ax0 = axes[0]
ax0.plot(t_min, best['P_dem'], color='black',      lw=0.5, label='P_dem')
ax0.plot(t_min, best['P_fc'],  color='steelblue',  lw=0.9, label='P_fc')
ax0.plot(t_min, best['P_sc'],  color='darkorange', lw=0.7, label='P_sc')
ax0.set_ylabel('Power [W]')
ax0.legend(loc='upper right', fontsize=8)
ax0.grid(True, lw=0.3)

ax1 = axes[1]
ax1.plot(t_min, best['SOC'], color='green', lw=0.8)
ax1.axhline(SC_SOC_0, color='black', lw=0.8, ls='--', label=f'SOC_0={SC_SOC_0}')
ax1.set_ylabel('SC SOC [—]')
ax1.set_ylim(0.0, 1.0)
ax1.legend(loc='upper right', fontsize=8)
ax1.grid(True, lw=0.3)

ax2 = axes[2]
ax2.plot(t_min, best['I_fc'], color='firebrick', lw=0.8)
ax2.set_ylabel('I_fc [A]')
ax2.grid(True, lw=0.3)

ax3 = axes[3]
ax3.plot(t_min, cum_H2, color='darkviolet', lw=0.8)
ax3.set_ylabel('Cumulative H2 [g]')
ax3.set_xlabel('Time [min]')
ax3.grid(True, lw=0.3)

fig.tight_layout(rect=[0, 0, 1, 0.96])
out = os.path.join(RESULTS_DIR, 'strategy_a_elev_comparison.png')
fig.savefig(out, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f"\nFigure saved → {out}")
