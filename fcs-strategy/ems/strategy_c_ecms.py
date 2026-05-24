"""
Strategy C — ECMS (Equivalent Consumption Minimization Strategy) for Hydraix I.

Theoretically optimal EMS derived from Pontryagin's Minimum Principle.
At each timestep, minimises the scalar Hamiltonian:

    H(P_fc) = ṁ_H2_fc(P_fc)  +  s · (P_dem − P_fc) / ECMS_DENOM

where s is the co-state (equivalence factor) tuned offline by bisection to
enforce charge sustenance (ΔSOC ≈ 0) over the full 11-lap race.

Fixes vs original implementation
─────────────────────────────────
1. Local simulator clamps P_fc to [0, FC_P_MAX]: FC can be fully off during
   glide. The ems_core version forces FC_P_MIN=100 W always, which causes
   ~1 kW overcharge of the 10.67 Wh SC in seconds during zero-demand glide
   phases and prevents bisection convergence.

2. ECMS Hamiltonian evaluated over 201 candidates including P_fc=0 W.
   When P_fc=0: I_fc=0, ṁ_H2=0. During glide (P_dem≈0), H(0)=0 which
   beats H(any P>0) = K_H2·I(P) + s·(-P)/ECMS_DENOM once s is reasonable,
   so ECMS naturally selects FC-off during glide.

3. SOC boundary masking: hard gates block SC actions that would violate
   SC_SOC_MAX (block further charging) or SC_SOC_MIN (block discharging).
   This prevents the "charging past the ceiling" energy waste that made
   power balance meaningless in the original.

Reference: Paganelli et al. (2002), "General supervisory control policy for
           the energy optimisation in charge-sustaining hybrid electric vehicles."
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ems_core import (
    build_demand_profile, fc_current, fc_h2_rate,
    sc_soc_update, sc_voltage,
    SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX, SC_E_J,
    FC_P_MAX, FC_RAMP,
    K_H2, LHV_H2, ETA_FC_REF, ECMS_DENOM,
    N_LAPS, DT, RESULTS_DIR
)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ── Local simulator: P_fc ∈ [0, FC_P_MAX] (FC fully off allowed) ─────────────
def simulate_race_ecms(strategy_fn, SOC_0=SC_SOC_0, verbose=False):
    """
    Race simulator identical to ems_core.simulate_race() EXCEPT:
      - P_fc is clamped to [0, FC_P_MAX] (FC can be fully shut down)
      - When P_fc_cmd == 0: I_fc = 0, H2 flow = 0 (truly off)
      - Ramp-up is limited by FC_RAMP; instantaneous shut-down is allowed
      - SC floor protection still forces FC on if SC would drop below SC_SOC_MIN
    """
    t_lap, P_lap = build_demand_profile()
    N_pts = len(t_lap)
    T_lap = float(t_lap[-1])
    t_all = np.concatenate([t_lap + i * T_lap for i in range(N_LAPS)])
    P_dem = np.tile(P_lap, N_LAPS)
    n = N_LAPS * N_pts

    P_fc_ar  = np.empty(n)
    P_sc_ar  = np.empty(n)
    SOC_ar   = np.empty(n + 1)
    I_fc_ar  = np.empty(n)
    m_H2_ar  = np.empty(n)
    lap_SOC  = np.empty(N_LAPS + 1)

    SOC_ar[0]  = SOC_0
    lap_SOC[0] = SOC_0
    P_fc_prev  = 0.0   # FC starts off

    for k in range(n):
        lap_idx  = k // N_pts
        t_in_lap = float(t_all[k] - lap_idx * T_lap)
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem[k])

        P_fc_cmd = float(strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx))

        # Ramp-up limited; instantaneous shut-down allowed
        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k = Pd - P_fc_cmd

        # SC floor protection: if SC would deplete, force FC to cover demand
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = float(np.clip(Pd, 0.0, FC_P_MAX))
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(P_fc_cmd) if P_fc_cmd > 0 else 0.0)
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev      = P_fc_cmd

        if (k + 1) % N_pts == 0:
            lap = (k + 1) // N_pts
            lap_SOC[lap] = SOC_ar[k + 1]
            if verbose:
                lap_h2 = float(np.sum(m_H2_ar[k + 1 - N_pts: k + 1]))
                print(f"  Lap {lap:2d}  SOC={lap_SOC[lap]:.3f}  H2={lap_h2:.3f} g")

    return {
        't':          t_all,
        'P_dem':      P_dem,
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'SOC_final':  float(SOC_ar[n]),
        'lap_SOC':    lap_SOC,
        'N_pts_lap':  N_pts,
        'T_lap':      T_lap,
    }


# ── ECMS Hamiltonian with 201-point grid (0 W included) ──────────────────────
# H(P_fc) = K_H2·I_fc(P_fc) + s·(P_dem − P_fc)/ECMS_DENOM
#
# With P_fc = 0 in the grid:
#   H(0) = 0 + s·P_dem/ECMS_DENOM   (only SC-equivalent cost, no H2 burned)
# During glide (P_dem ≈ 0): H(0) ≈ 0, which beats any positive P_fc.
# During driving: ECMS finds the optimal split between H2 cost and SC cost.
_P_CANDS = np.linspace(0.0, FC_P_MAX, 201)   # 0 W included


def make_strategy(s):
    """
    Return a strategy function that minimises H(P_fc) for equivalence factor s,
    with SOC boundary masking to prevent hard-limit violations.

    Higher s → SC discharge penalised more → FC runs higher → SOC recovers.
    Lower s  → SC discharge tolerated     → FC runs leaner → SOC depletes.
    """
    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        # H2 flow for each candidate (0 W → 0 A → 0 g/s)
        I_arr   = np.where(_P_CANDS > 0, fc_current(_P_CANDS), 0.0)
        mdot_fc = K_H2 * I_arr                         # [g/s]
        mdot_sc = (P_dem - _P_CANDS) / ECMS_DENOM      # [g/s] (negative = charging)
        H       = mdot_fc + s * mdot_sc                # Hamiltonian

        # SOC boundary masking: block SC actions that would violate hard limits
        P_sc_c = P_dem - _P_CANDS
        mask = np.ones(201, dtype=bool)
        if SOC >= SC_SOC_MAX - 0.03:      # SC nearly full → block further charging
            mask[P_sc_c < -10.0] = False
        if SOC <= SC_SOC_MIN + 0.03:      # SC nearly empty → block further discharging
            mask[P_sc_c >  10.0] = False

        H_masked = np.where(mask, H, np.inf)
        best_idx = int(np.argmin(H_masked))
        if not np.isfinite(H_masked[best_idx]):         # fallback: match demand
            return float(np.clip(P_dem, 0.0, FC_P_MAX))
        return float(_P_CANDS[best_idx])

    return strategy_fn


# ── Charge-sustenance bisection on s ─────────────────────────────────────────
# Physics:
#   s too low  → H penalises SC usage weakly → SC drains  → ΔSOC < 0 → raise s
#   s too high → H over-penalises SC usage   → SC charges → ΔSOC > 0 → lower s
TOLERANCE = 0.01
s      = 1.5        # initial guess
lo, hi = 0.1, 5.0
best_result = None

print("Tuning ECMS equivalence factor s for charge sustenance (|ΔSOC| < 0.01) ...")
for iteration in range(30):
    result    = simulate_race_ecms(make_strategy(s), SOC_0=SC_SOC_0, verbose=False)
    delta_soc = result['delta_SOC']
    print(f"  Iter {iteration+1:2d}  s={s:.4f}  ΔSOC={delta_soc:+.4f}  H2={result['m_H2_total']:.3f} g")

    if abs(delta_soc) <= TOLERANCE:
        best_result = result
        break

    # Bisection: ΔSOC < 0 → SC net-discharged → penalise SC discharge more
    if delta_soc < -TOLERANCE:
        lo = s
    else:                         # ΔSOC > 0 → SC net-charged → relax penalty
        hi = s
    s = (lo + hi) / 2
    best_result = result

if best_result is None:
    best_result = result


# ── Extract arrays ────────────────────────────────────────────────────────────
t         = best_result['t']
P_dem     = best_result['P_dem']
P_fc      = best_result['P_fc']
P_sc      = best_result['P_sc']
SOC       = best_result['SOC']
I_fc      = best_result['I_fc']
m_H2      = best_result['m_H2_total']
delta_soc = best_result['delta_SOC']
lap_SOC   = best_result['lap_SOC']

t_min = t / 60.0

# ── Derived statistics ────────────────────────────────────────────────────────
mean_pfc   = float(np.mean(P_fc))
pct_off    = 100.0 * float(np.sum(P_fc == 0.0)) / len(P_fc)
in_band    = (P_fc >= 700.0) & (P_fc <= 900.0)
pct_cruise = 100.0 * float(np.sum(in_band)) / len(P_fc)
min_soc    = float(np.min(lap_SOC[1:]))
max_soc    = float(np.max(lap_SOC[1:]))

# Cumulative H2 [g]
cum_H2 = np.cumsum(K_H2 * I_fc * DT)


# ── Figure ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
fig.suptitle(
    f"Strategy C — ECMS (corrected — FC-off glide allowed)  |  s={s:.4f}  |  "
    f"ΔSOC={delta_soc:+.4f}  |  H2={m_H2:.3f} g",
    fontsize=11
)

# Panel 1: Power split
ax0 = axes[0]
ax0.plot(t_min, P_dem, color='black',      lw=0.6, label='P_dem')
ax0.plot(t_min, P_fc,  color='steelblue',  lw=0.8, label='P_fc')
ax0.plot(t_min, P_sc,  color='darkorange', lw=0.8, label='P_sc')
ax0.set_ylabel('Power [W]')
ax0.legend(loc='upper right', fontsize=8)
ax0.grid(True, lw=0.3)

# Panel 2: SC state of charge
ax1 = axes[1]
ax1.plot(t_min, SOC, color='green', lw=0.8)
ax1.axhline(0.60,       color='black',  lw=0.8, ls='--', label='SOC_ref=0.60')
ax1.axhline(SC_SOC_MIN, color='red',    lw=0.8, ls=':',  label=f'SOC_MIN={SC_SOC_MIN:.2f}')
ax1.axhline(SC_SOC_MAX, color='purple', lw=0.8, ls=':',  label=f'SOC_MAX={SC_SOC_MAX:.2f}')
ax1.set_ylabel('SC SOC [—]')
ax1.legend(loc='upper right', fontsize=8)
ax1.grid(True, lw=0.3)

# Panel 3: FC stack current
ax2 = axes[2]
ax2.plot(t_min, I_fc, color='firebrick', lw=0.8)
ax2.axhline(37.5, color='navy', lw=0.8, ls='--', label='I_nom = 37.5 A')
ax2.set_ylabel('I_fc [A]')
ax2.legend(loc='upper right', fontsize=8)
ax2.grid(True, lw=0.3)

# Panel 4: Cumulative H2 [g]
ax3 = axes[3]
ax3.plot(t_min, cum_H2, color='darkviolet', lw=0.8)
ax3.set_ylabel('Cumulative H2 [g]')
ax3.set_xlabel('Time [min]')
ax3.grid(True, lw=0.3)

fig.tight_layout(rect=[0, 0, 1, 0.96])
out_path = os.path.join(RESULTS_DIR, 'strategy_c_ecms_results.png')
fig.savefig(out_path, dpi=150)
plt.close(fig)
print(f"\nFigure saved → {out_path}")


# ── Summary ───────────────────────────────────────────────────────────────────
print()
print("=== Strategy C: ECMS (corrected — FC-off glide allowed) ===")
print(f"  Tuned equiv factor: s = {s:.4f}")
print(f"  ECMS_DENOM        : {ECMS_DENOM:.1f} W·s/g")
print(f"  Final ΔSOC        : {delta_soc:+.4f}")
print(f"  Total H2 consumed : {m_H2:.3f} g")
print(f"  Charge sustained  : {'YES' if abs(delta_soc) <= 0.01 else 'NO'}")
print(f"  Convergence iters : {iteration+1}")
print(f"  Min/Max lap SOC   : {min_soc:.3f} / {max_soc:.3f}")
print(f"  Mean P_fc         : {mean_pfc:.1f} W")
print(f"  FC off (P=0) time : {pct_off:.1f}%")
print(f"  FC in 700–900 W   : {pct_cruise:.1f}%")
print("===========================================================")
