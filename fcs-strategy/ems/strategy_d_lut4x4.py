"""
Strategy D — 4×4 Discrete Efficiency-Point Lookup Table for Hydraix I (SEM 2026).

Four FC operating levels, three of which are derived from FuelCellEstimate_v3:

  L0  GLIDE    :   0 W   — FC off (P_dem < 30 W AND SOC ≥ SOC_NOM floor)
  L1  PEAK     : 120 W   — peak system efficiency (η_sys = 59.0 %)
                           Active only when SC has surplus (HIGH SOC or very low demand)
  L2  CRUISE   :  ~W     — bisection knob; steady-state operating point (~race average)
  L3  BOOST    : 600 W   — SC emergency recharge (CRIT / LOW SOC + high demand)
                           Fixed at the 50 % descending-efficiency crossing (668 W ≈ 600 W)

The 4×4 grid maps (SOC_band × P_dem_band) → level.
P_dem (not u) is used as the second axis because it is stable; u = (P_dem-P_fc_prev)/E_sc
oscillates when P_fc_prev is far from the level setpoints.

Why L1 = 120 W can only be used selectively
────────────────────────────────────────────
  Race average demand ≈ 259 W.  Running FC at 120 W continuously → SC deficit of
  139 W continuously → 10.67 Wh SC drained in ≈ 4.6 min.
  L1 is viable only when the SC has surplus energy (HIGH SOC) or demand is so low
  that the SC absorbs the FC surplus without hitting its ceiling.
  This is physically correct: peak-efficiency operation is the luxury of a full SC.

Bisection knob: P_CRUISE (L2), targeting ΔSOC = 0 after 11 laps.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "FCS Strategy"))

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_ETA, SC_E_J,
    SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, DT, N_LAPS, RESULTS_DIR,
    build_demand_profile, fc_current, fc_h2_rate, sc_soc_update
)

# ── Derive L1 (peak efficiency) from FC model v3 ──────────────────────────────
try:
    from FuelCellEstimate_v3 import P_net as _fc_pnet, eta_system_net as _fc_eta, I_max as _IMAX
    _I  = np.linspace(0.01, _IMAX, 50_000)
    _pk = int(np.argmax(_fc_eta(_I)))
    L1_W   = float(_fc_pnet(_I)[_pk])
    L1_ETA = float(_fc_eta(_I)[_pk])
    _HAVE_FC = True
    print(f"FC model v3: L1 peak efficiency = {L1_W:.1f} W  η={L1_ETA:.2f}%")
except Exception as exc:
    print(f"[warn] FC model import failed ({exc}). Using L1=120.2 W.")
    L1_W, L1_ETA, _HAVE_FC = 120.2, 59.00, False

L0_W = 0.0          # FC off
L3_W = 600.0        # fixed boost (near descending 50 % crossing)

# ── Grid thresholds ───────────────────────────────────────────────────────────
# SOC bands  (4 rows, index 0 = lowest SOC)
SOC_BANDS = [0.30, 0.55, 0.70]   # [CRIT|LOW, LOW|NOM, NOM|HIGH]

# P_dem bands (4 cols, index 0 = lowest demand)
#   Designed so col-2 spans the range that needs steady-state cruise power.
P_DEM_BANDS = [30.0, L1_W, 350.0]   # [0–30W | 30–120W | 120–350W | 350+W]

# Glide: FC off only when demand is below this AND SOC is not low
P_GLIDE_MAX = 30.0   # W

# ── Decision table: (soc_band, pdem_band) → level index ──────────────────────
#
#               P<30W   30–120W  120–350W   >350W
_TABLE = np.array([
    [2, 3, 3, 3],   # SOC CRIT (<0.30)   → CRUISE or BOOST
    [1, 2, 2, 3],   # SOC LOW  (0.30–0.55) → PEAK or CRUISE or BOOST
    [0, 1, 2, 2],   # SOC NOM  (0.55–0.70) → OFF / PEAK / CRUISE
    [0, 0, 1, 2],   # SOC HIGH (>0.70)    → OFF / PEAK rarely
], dtype=int)

# Human-readable level names (for logging)
_LEVEL_NAMES = {0: 'GLIDE', 1: 'PEAK', 2: 'CRUISE', 3: 'BOOST'}


def _soc_idx(soc):
    return int(np.searchsorted(SOC_BANDS, soc, side='right'))

def _pdem_idx(p_dem):
    return int(np.searchsorted(P_DEM_BANDS, p_dem, side='right'))


# ── Local race simulator (P_fc ∈ [0, FC_P_MAX]) ──────────────────────────────
def simulate_race_d(strategy_fn, SOC_0=SC_SOC_0, verbose=False):
    t_lap, P_lap = build_demand_profile()
    N_pts = len(t_lap);  T_lap = float(t_lap[-1])
    t_all = np.concatenate([t_lap + i * T_lap for i in range(N_LAPS)])
    P_dem = np.tile(P_lap, N_LAPS);  n = N_LAPS * N_pts

    P_fc_ar  = np.empty(n);  P_sc_ar = np.empty(n)
    SOC_ar   = np.empty(n + 1);  I_fc_ar = np.empty(n)
    m_H2_ar  = np.empty(n);  lap_SOC = np.empty(N_LAPS + 1)
    level_ar = np.empty(n, dtype=int)

    SOC_ar[0] = SOC_0;  lap_SOC[0] = SOC_0;  P_fc_prev = 0.0

    for k in range(n):
        lap_idx  = k // N_pts
        t_in_lap = float(t_all[k] - lap_idx * T_lap)
        soc_k    = SOC_ar[k];  Pd = float(P_dem[k])

        P_cmd, lv = strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx)

        # Ramp-rate (instant shut-down; ramp-up limited)
        if P_cmd > P_fc_prev:
            P_cmd = min(P_cmd, P_fc_prev + FC_RAMP * DT)
        P_cmd = float(np.clip(P_cmd, 0.0, FC_P_MAX))

        P_sc_k = Pd - P_cmd

        # SC floor protection
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_cmd     = float(np.clip(Pd, 0.0, FC_P_MAX))
            P_sc_k    = max(0.0, Pd - P_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_cmd;  P_sc_ar[k] = P_sc_k
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(P_cmd)) if P_cmd > 0 else 0.0
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        level_ar[k]   = lv;  P_fc_prev = P_cmd

        if (k + 1) % N_pts == 0:
            lap = (k + 1) // N_pts
            lap_SOC[lap] = SOC_ar[k + 1]
            if verbose:
                lap_h2 = float(np.sum(m_H2_ar[k + 1 - N_pts: k + 1]))
                print(f"  Lap {lap:2d}  SOC={lap_SOC[lap]:.4f}  H2={lap_h2:.4f} g")

    return {
        't': t_all, 'P_dem': P_dem, 'P_fc': P_fc_ar, 'P_sc': P_sc_ar,
        'SOC': SOC_ar[:-1], 'I_fc': I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC': float(SOC_ar[n] - SOC_0),
        'SOC_final': float(SOC_ar[n]),
        'lap_SOC': lap_SOC,
        'level': level_ar,
        'N_pts_lap': N_pts, 'T_lap': T_lap,
    }


# ── Strategy factory ──────────────────────────────────────────────────────────
def make_strategy(P_cruise):
    """
    Build strategy with L2 = P_cruise.
    Returns a function that also yields the level index (for occupancy stats).
    """
    levels = np.array([L0_W, L1_W, P_cruise, L3_W])

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        si = _soc_idx(SOC)
        pi = _pdem_idx(P_dem)
        lv = int(_TABLE[si, pi])
        P  = float(levels[lv])

        # Override to L0 if this is a true glide (demand near zero + SC not depleted)
        if P_dem < P_GLIDE_MAX and lv <= 1 and SOC > SOC_BANDS[0]:
            P  = L0_W
            lv = 0

        return P, lv

    return strategy_fn


# ── Bisection on P_CRUISE (L2) ────────────────────────────────────────────────
TOLERANCE = 0.01
P_cruise  = 270.0          # starting guess (slightly above race avg 259 W)
lo, hi    = L1_W + 1, FC_P_MAX
best_result = None

print(f"\nTuning P_cruise (L2) for charge sustenance (|ΔSOC| < {TOLERANCE}) ...")
for iteration in range(30):
    result    = simulate_race_d(make_strategy(P_cruise), SOC_0=SC_SOC_0)
    delta_soc = result['delta_SOC']
    print(f"  Iter {iteration + 1:2d}  P_cruise={P_cruise:6.1f} W  "
          f"ΔSOC={delta_soc:+.4f}  H2={result['m_H2_total']:.3f} g")

    if abs(delta_soc) <= TOLERANCE:
        best_result = result
        break

    if delta_soc < -TOLERANCE:
        lo = P_cruise       # SC depleting → raise cruise
    else:
        hi = P_cruise       # SC overcharging → lower cruise
    P_cruise    = (lo + hi) / 2
    best_result = result

if best_result is None:
    best_result = result

# ── Extract results ───────────────────────────────────────────────────────────
t         = best_result['t'];       P_dem_r = best_result['P_dem']
P_fc_r    = best_result['P_fc'];    P_sc_r  = best_result['P_sc']
SOC_r     = best_result['SOC'];     I_fc_r  = best_result['I_fc']
lv_r      = best_result['level']
m_H2_total = best_result['m_H2_total']
delta_soc  = best_result['delta_SOC']
lap_SOC    = best_result['lap_SOC']

t_min   = t / 60.0
cum_H2  = np.cumsum(K_H2 * I_fc_r * DT)
mean_pfc = float(np.mean(P_fc_r))
occ     = {i: float(np.mean(lv_r == i)) * 100 for i in range(4)}

# ── Results plot ──────────────────────────────────────────────────────────────
LEVEL_COLORS = ['#888888', '#00C853', '#1565C0', '#E65100']
LEVEL_LABELS = [f'L0  0 W (glide)', f'L1 {L1_W:.0f} W (peak η {L1_ETA:.0f}%)',
                f'L2 {P_cruise:.0f} W (cruise)', f'L3 {L3_W:.0f} W (boost)']

fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
fig.suptitle(
    f"Strategy D — 4×4 Discrete Efficiency-Point LUT  |  "
    f"ΔSOC={delta_soc:+.4f}  |  H₂={m_H2_total:.3f} g  |  "
    f"P_cruise={P_cruise:.1f} W",
    fontsize=11
)

ax0 = axes[0]
ax0.plot(t_min, P_dem_r, color='black', lw=0.5, alpha=0.55, label='P_dem')
ax0.plot(t_min, P_fc_r,  color='#D32F2F', lw=0.8, label='P_fc (actual)')
for lv_i, lv_c, lv_l in zip(range(1, 4), LEVEL_COLORS[1:], LEVEL_LABELS[1:]):
    lv_val = [L0_W, L1_W, P_cruise, L3_W][lv_i]
    ax0.axhline(lv_val, color=lv_c, lw=1.0, ls='--', alpha=0.65, label=lv_l)
ax0.set_ylabel('Power [W]')
ax0.legend(loc='upper right', fontsize=7.5, ncol=2)
ax0.grid(True, lw=0.3)

# Level band background
for k_s in range(0, len(t_min) - 1, 1):
    lv = lv_r[k_s]
    ax0.axvspan(t_min[k_s], t_min[min(k_s + 1, len(t_min) - 1)],
                color=LEVEL_COLORS[lv], alpha=0.04, linewidth=0)

ax1 = axes[1]
ax1.plot(t_min, SOC_r, color='green', lw=0.8)
ax1.axhline(SC_SOC_0,   color='black',  lw=0.8, ls='--', label=f'SOC₀={SC_SOC_0}')
ax1.axhline(SC_SOC_MIN, color='red',    lw=0.8, ls=':',  label=f'SOC_MIN={SC_SOC_MIN}')
ax1.axhline(SC_SOC_MAX, color='purple', lw=0.8, ls=':',  label=f'SOC_MAX={SC_SOC_MAX}')
for thresh in SOC_BANDS:
    ax1.axhline(thresh, color='grey', lw=0.6, ls=':', alpha=0.45, label=f'band {thresh}')
ax1.set_ylabel('SC SOC [—]')
ax1.legend(loc='upper right', fontsize=7, ncol=2)
ax1.grid(True, lw=0.3)

ax2 = axes[2]
ax2.plot(t_min, I_fc_r, color='firebrick', lw=0.6)
for lv_i, lv_c in zip(range(1, 4), LEVEL_COLORS[1:]):
    lv_val = [L0_W, L1_W, P_cruise, L3_W][lv_i]
    if lv_val > 0:
        ax2.axhline(float(fc_current(lv_val)), color=lv_c, lw=0.8, ls='--', alpha=0.5)
ax2.set_ylabel('I_fc [A]')
ax2.grid(True, lw=0.3)

ax3 = axes[3]
ax3.plot(t_min, cum_H2, color='darkviolet', lw=0.8)
ax3.set_ylabel('Cumulative H₂ [g]')
ax3.set_xlabel('Time [min]')
ax3.grid(True, lw=0.3)

fig.tight_layout(rect=[0, 0, 1, 0.96])
out_path = os.path.join(RESULTS_DIR, 'strategy_d_lut4x4_results.png')
fig.savefig(out_path, dpi=150)
plt.close(fig)
print(f"\nFigure → {out_path}")

# ── Summary ───────────────────────────────────────────────────────────────────
eta_cruise = float(np.interp(P_cruise, _fc_pnet(_I), _fc_eta(_I))) if _HAVE_FC else 0.0
print()
print("=== Strategy D: 4×4 Discrete Efficiency-Point LUT ===")
print(f"  L0 GLIDE    :   0.0 W  (FC off)")
print(f"  L1 PEAK     : {L1_W:5.1f} W  (η={L1_ETA:.1f}% — peak from FC model v3)")
print(f"  L2 CRUISE   : {P_cruise:5.1f} W  (η≈{eta_cruise:.1f}% — tuned for charge sustenance)")
print(f"  L3 BOOST    : {L3_W:5.1f} W  (fixed SC emergency recharge)")
print(f"  Final ΔSOC  : {delta_soc:+.4f}  (target |ΔSOC| < {TOLERANCE})")
print(f"  H₂ consumed : {m_H2_total:.3f} g")
print(f"  Sustained   : {'YES ✓' if abs(delta_soc) <= TOLERANCE else 'NO ✗'}")
print(f"  Iterations  : {iteration + 1}")
print(f"  Mean P_fc   : {mean_pfc:.1f} W")
print(f"  Level occupancy:")
for lv_i, lv_l in enumerate(LEVEL_LABELS):
    print(f"    {lv_l:30s}: {occ[lv_i]:.1f}%")
print("======================================================")
print()
print("── Comparison ──")
print(f"  Strategy A (LUT continuous)   : 8.204 g  ΔSOC=+0.006")
print(f"  Strategy B (FSM)              : 8.425 g  ΔSOC=-0.003")
print(f"  Strategy C (ECMS/PMP)         : 8.197 g  ΔSOC=-0.007")
print(f"  Strategy D (4×4 discrete)     : {m_H2_total:.3f} g  ΔSOC={delta_soc:+.3f}")
