"""
Strategy A — Heuristic 2D Lookup Table EMS for Hydraix I (SEM 2026).

Maps (P_sc_unsat / SC_E_J, SOC) → FC power setpoint via a pre-built 21×18 table
with bilinear interpolation. K_soc is tuned by bisection to achieve charge
sustenance (|ΔSOC| < 0.01) over the full 11-lap race.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ems_core import (
    simulate_race, build_demand_profile,
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_C, SC_V_MAX, SC_V_MIN, SC_E_J, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, N_LAPS, DT, RESAMPLE_HZ, RESULTS_DIR
)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Scipy bilinear interpolation (preferred) ──────────────────────────────────
try:
    from scipy.interpolate import RegularGridInterpolator
    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False


def _bilinear(u_g, soc_g, T, u_val, soc_val):
    """
    Numpy-only bilinear interpolation on a regular grid.
    T shape: (len(u_g), len(soc_g))
    """
    i = np.searchsorted(u_g, u_val, side='right') - 1
    j = np.searchsorted(soc_g, soc_val, side='right') - 1
    i = np.clip(i, 0, len(u_g) - 2)
    j = np.clip(j, 0, len(soc_g) - 2)
    du = (u_val - u_g[i]) / (u_g[i + 1] - u_g[i])
    ds = (soc_val - soc_g[j]) / (soc_g[j + 1] - soc_g[j])
    return (T[i,     j    ] * (1 - du) * (1 - ds)
          + T[i + 1, j    ] *      du  * (1 - ds)
          + T[i,     j + 1] * (1 - du) *      ds
          + T[i + 1, j + 1] *      du  *      ds)


def make_strategy(K_soc):
    """
    Build a strategy function with the given SOC feedback gain [W per unit SOC].

    Table axes:
      u   = (P_dem − P_fc_prev) / SC_E_J  ∈ [−0.050, +0.050]  1/s  (21 pts)
      SOC ∈ [0.10, 0.95]                                              (18 pts)

    P_fc_base = 259 W is the race-average demand (151 Wh / 0.5833 h).
    K_rate feedforward activates beyond |u| = 0.020 to prevent deep SC swings.
    """
    u_grid   = np.linspace(-0.050, 0.050, 21)
    soc_grid = np.linspace(0.10, 0.95, 18)

    K_rate = 500.0
    P_base = 259.0

    TABLE = np.zeros((21, 18))
    for i, u_i in enumerate(u_grid):
        for j, soc_j in enumerate(soc_grid):
            delta_soc  = K_soc * (0.60 - soc_j)
            delta_rate = (K_rate * (abs(u_i) - 0.020) * np.sign(u_i)
                          if abs(u_i) > 0.020 else 0.0)
            TABLE[i, j] = np.clip(P_base + delta_soc + delta_rate,
                                  FC_P_MIN, FC_P_MAX)

    if _HAVE_SCIPY:
        _interp = RegularGridInterpolator(
            (u_grid, soc_grid), TABLE,
            method='linear', bounds_error=False, fill_value=None
        )

        def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
            u = (P_dem - P_fc_prev) / SC_E_J
            return float(_interp([[u, SOC]])[0])

    else:
        def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
            u     = (P_dem - P_fc_prev) / SC_E_J
            u_c   = np.clip(u,   u_grid[0],   u_grid[-1])
            soc_c = np.clip(SOC, soc_grid[0], soc_grid[-1])
            return float(_bilinear(u_grid, soc_grid, TABLE, u_c, soc_c))

    return strategy_fn


# ── Charge-sustenance bisection loop ─────────────────────────────────────────
TOLERANCE = 0.01
K_soc = 200.0
lo, hi = 50.0, 600.0
best_result = None
iteration = 0

print("Tuning K_soc for charge sustenance (|ΔSOC| < 0.01) ...")
for iteration in range(25):
    result = simulate_race(make_strategy(K_soc), SOC_0=SC_SOC_0, verbose=False)
    delta_soc = result['delta_SOC']
    print(f"  Iter {iteration + 1:2d}  K_soc={K_soc:.1f}  "
          f"ΔSOC={delta_soc:+.4f}  H2={result['m_H2_total']:.3f} g")

    if abs(delta_soc) <= TOLERANCE:
        best_result = result
        break

    # If SC net-discharges, FC must work harder → raise K_soc
    if delta_soc < -TOLERANCE:
        lo = K_soc
    else:
        hi = K_soc
    K_soc = (lo + hi) / 2
    best_result = result

if best_result is None:
    best_result = result


# ── Extract results ───────────────────────────────────────────────────────────
t        = best_result['t']
P_dem    = best_result['P_dem']
P_fc     = best_result['P_fc']
P_sc     = best_result['P_sc']
SOC      = best_result['SOC']
I_fc     = best_result['I_fc']
m_H2_total = best_result['m_H2_total']
delta_soc  = best_result['delta_SOC']
lap_SOC    = best_result['lap_SOC']

t_min       = t / 60.0
cum_H2      = np.cumsum(K_H2 * I_fc * DT)

mean_pfc  = float(np.mean(P_fc))
mean_psc  = float(np.mean(P_sc))
max_psc   = float(np.max(P_sc))
min_lap   = float(np.min(lap_SOC[1:]))
max_lap   = float(np.max(lap_SOC[1:]))


# ── Figure ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
fig.suptitle(
    f"Strategy A — Heuristic Lookup Table  |  "
    f"ΔSOC={delta_soc:+.4f}  |  "
    f"H2={m_H2_total:.3f} g  |  "
    f"K_soc={K_soc:.1f} W/SOC",
    fontsize=11
)

ax0 = axes[0]
ax0.plot(t_min, P_dem, color='black',  lw=0.6, label='P_dem')
ax0.plot(t_min, P_fc,  color='steelblue',  lw=0.8, label='P_fc')
ax0.plot(t_min, P_sc,  color='darkorange', lw=0.8, label='P_sc')
ax0.set_ylabel('Power [W]')
ax0.legend(loc='upper right', fontsize=8)
ax0.grid(True, lw=0.3)

ax1 = axes[1]
ax1.plot(t_min, SOC, color='green', lw=0.8)
ax1.axhline(SC_SOC_0,   color='black',  lw=0.8, ls='--', label=f'SOC_0={SC_SOC_0}')
ax1.axhline(SC_SOC_MIN, color='red',    lw=0.8, ls=':',  label=f'SOC_MIN={SC_SOC_MIN}')
ax1.axhline(SC_SOC_MAX, color='purple', lw=0.8, ls=':',  label=f'SOC_MAX={SC_SOC_MAX}')
ax1.set_ylabel('SC SOC [—]')
ax1.legend(loc='upper right', fontsize=8)
ax1.grid(True, lw=0.3)

ax2 = axes[2]
ax2.plot(t_min, I_fc, color='firebrick', lw=0.8)
ax2.set_ylabel('I_fc [A]')
ax2.grid(True, lw=0.3)

ax3 = axes[3]
ax3.plot(t_min, cum_H2, color='darkviolet', lw=0.8)
ax3.set_ylabel('Cumulative H2 [g]')
ax3.set_xlabel('Time [min]')
ax3.grid(True, lw=0.3)

fig.tight_layout(rect=[0, 0, 1, 0.96])
out_path = os.path.join(RESULTS_DIR, 'strategy_a_lut_results.png')
fig.savefig(out_path, dpi=150)
plt.close(fig)
print(f"\nFigure saved → {out_path}")


# ── Summary ───────────────────────────────────────────────────────────────────
print()
print("=== Strategy A: Heuristic Lookup Table ===")
print(f"  Tuned K_soc       : {K_soc:.1f} W per unit SOC")
print(f"  Final ΔSOC        : {delta_soc:+.4f}  (target |ΔSOC| < 0.01)")
print(f"  Total H2 consumed : {m_H2_total:.3f} g")
print(f"  Charge sustained  : {'YES' if abs(delta_soc) <= 0.01 else 'NO'}")
print(f"  Convergence iters : {iteration + 1}")
print(f"  Min lap SOC       : {min_lap:.3f}")
print(f"  Max lap SOC       : {max_lap:.3f}")
print(f"  Mean P_fc         : {mean_pfc:.1f} W")
print(f"  Mean P_sc         : {mean_psc:.1f} W")
print(f"  Peak SC discharge : {max_psc:.1f} W")
print("==========================================")
