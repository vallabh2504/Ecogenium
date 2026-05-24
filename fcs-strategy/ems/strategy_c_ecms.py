"""
Strategy C — ECMS (Equivalent Consumption Minimization Strategy) for Hydraix I.

Theoretically optimal EMS derived from Pontryagin's Minimum Principle.
At each timestep, minimises the scalar Hamiltonian:

    H(P_fc) = ṁ_H2_fc(P_fc)  +  s_eff(t) · (P_dem − P_fc) / ECMS_DENOM

where s_eff(t) = s₀ + K_cs · (SOC(t) − SOC_ref) is the adaptive equivalence
factor (A-ECMS). The base co-state s₀ is tuned offline by bisection; the
SOC-error feedback gain K_cs is a small linear correction (standard A-ECMS,
Onori & Serrao 2011) that breaks the open-loop quantization dead-band without
meaningfully departing from the PMP optimum.

Reference: Paganelli et al. (2002), "General supervisory control policy for
           the energy optimisation in charge-sustaining hybrid electric vehicles."
           Onori & Serrao (2011), "On Adaptive-ECMS Strategies for Hybrid
           Electric Vehicles."
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ems_core import (
    simulate_race, fc_current, fc_h2_rate,
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    SC_E_J, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, LHV_H2, ETA_FC_REF, ECMS_DENOM,
    N_LAPS, DT, RESULTS_DIR
)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# Pre-compute FC power candidates once — avoids repeated linspace allocation
_P_CANDS = np.linspace(FC_P_MIN, FC_P_MAX, 201)

# Pre-build the constant part of the Hamiltonian (mdot_fc) once globally
_I_CANDS      = fc_current(_P_CANDS)           # [A]  shape (201,)
_MDOT_FC_BASE = K_H2 * _I_CANDS               # [g/s] actual H2 flow, constant


def make_strategy(s0, K_cs=0.5):
    """
    Return an A-ECMS strategy function.

    H(P_fc, t) = ṁ_fc(P_fc)  +  s_eff(t) · (P_dem − P_fc) / ECMS_DENOM

    s_eff(t) = s0 + K_cs · (SOC(t) − SOC_ref)

    The SOC-error term implements the integral feedback that PMP theory requires
    but that pure offline bisection cannot deliver due to numerical quantization:
    when SOC drifts below SC_SOC_0, s_eff increases, making SC discharge more
    expensive, which forces higher P_fc and recovers SOC — and vice versa.

    Parameters
    ----------
    s0    : float  base equivalence factor (dimensionless), tuned by bisection
    K_cs  : float  charge-sustaining gain [1/SOC unit], typically 0.3–1.0
    """
    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        s_eff   = s0 + K_cs * (SOC - SC_SOC_0)       # adaptive co-state
        mdot_sc = (P_dem - _P_CANDS) / ECMS_DENOM    # equiv H2 from SC [g/s]
        H       = _MDOT_FC_BASE + s_eff * mdot_sc    # Hamiltonian [g/s]
        return float(_P_CANDS[np.argmin(H)])

    return strategy_fn


# ── Charge-sustenance bisection on s0 ────────────────────────────────────────
# Physics:
#   s0 too low  → H penalises SC usage weakly → SC drains  → ΔSOC < 0 → raise s0
#   s0 too high → H over-penalises SC usage   → SC charges → ΔSOC > 0 → lower s0
#
# K_cs provides continuous SOC-error correction that bridges the discrete jump
# in ΔSOC(s0) that the open-loop Hamiltonian cannot avoid due to argmin
# quantization over a finite candidate set.
TOLERANCE = 0.01
K_CS = 0.5           # charge-sustaining gain; mild enough not to distort PMP
s    = 1.8           # initial guess (expected range 0.7–1.5 for this vehicle)
lo, hi = 0.5, 8.0
best_result = None

print("Tuning ECMS equivalence factor s for charge sustenance (|ΔSOC| < 0.01) ...")
for iteration in range(30):
    result    = simulate_race(make_strategy(s, K_CS), SOC_0=SC_SOC_0, verbose=False)
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

# ── Hamiltonian profile H(t) for the final tuned s ───────────────────────────
# Re-evaluate at the actual P_fc and SOC at each timestep, using adaptive s_eff.
# The resulting trace verifies that the strategy is genuinely minimising H.
s_eff_trace   = s + K_CS * (SOC - SC_SOC_0)             # adaptive s at each step
mdot_fc_trace = K_H2 * I_fc                              # [g/s]
mdot_sc_trace = (P_dem - P_fc) / ECMS_DENOM             # [g/s]
H_trace       = mdot_fc_trace + s_eff_trace * mdot_sc_trace  # [g/s]

# ── Derived statistics ────────────────────────────────────────────────────────
mean_pfc   = float(np.mean(P_fc))
max_psc    = float(np.max(P_sc))
# Fraction of time FC operates in the high-efficiency band (700–900 W net)
in_band    = (P_fc >= 700.0) & (P_fc <= 900.0)
pct_cruise = 100.0 * float(np.sum(in_band)) / len(P_fc)


# ── Figure ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
fig.suptitle(
    f"Strategy C — ECMS  |  s={s:.4f}  |  ΔSOC={delta_soc:+.4f}  |  H2={m_H2:.3f} g",
    fontsize=11
)

# Panel 1: Power split
ax0 = axes[0]
ax0.plot(t_min, P_dem, color='black',      lw=0.6, label='P_dem')
ax0.plot(t_min, P_fc,  color='steelblue',  lw=0.8, label='P_fc')
ax0.plot(t_min, P_sc,  color='darkorange', lw=0.8, label='P_sc')
ax0.axhline(FC_P_MAX, color='darkred', lw=0.8, ls='--', label=f'FC_P_MAX={FC_P_MAX:.0f} W')
ax0.axhline(700.0,    color='green',   lw=0.8, ls='--', label='700 W (peak η band)')
ax0.set_ylabel('Power [W]')
ax0.legend(loc='upper right', fontsize=8)
ax0.grid(True, lw=0.3)

# Panel 2: SC state of charge
ax1 = axes[1]
ax1.plot(t_min, SOC, color='green', lw=0.8)
ax1.axhline(SC_SOC_0,   color='black',  lw=0.8, ls='--', label=f'SOC_0={SC_SOC_0:.2f}')
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

# Panel 4: Hamiltonian H(t) — verifies instantaneous minimisation
ax3 = axes[3]
ax3.plot(t_min, H_trace, color='darkviolet', lw=0.6)
ax3.set_ylabel('H(t) [g/s]')
ax3.set_xlabel('Time [min]')
ax3.grid(True, lw=0.3)

fig.tight_layout(rect=[0, 0, 1, 0.96])
out_path = os.path.join(RESULTS_DIR, 'strategy_c_ecms_results.png')
fig.savefig(out_path, dpi=150)
plt.close(fig)
print(f"\nFigure saved → {out_path}")


# ── Summary ───────────────────────────────────────────────────────────────────
print()
print("=== Strategy C: ECMS ===")
print(f"  Tuned equiv factor: s = {s:.4f}")
print(f"  ECMS_DENOM        : {ECMS_DENOM:.1f} W·s/g  (η_fc_ref={ETA_FC_REF:.3f}, LHV=120 kJ/g)")
print(f"  Final ΔSOC        : {delta_soc:+.4f}  (target |ΔSOC| < 0.01)")
print(f"  Total H2 consumed : {m_H2:.3f} g")
print(f"  Charge sustained  : {'YES' if abs(delta_soc) <= 0.01 else 'NO'}")
print(f"  Convergence iters : {iteration+1}")
print(f"  Min lap SOC       : {min(lap_SOC):.3f}")
print(f"  Max lap SOC       : {max(lap_SOC):.3f}")
print(f"  Mean P_fc         : {mean_pfc:.1f} W")
print(f"  FC in 700–900 W   : {pct_cruise:.1f}%  (peak-η operating window)")
print(f"  Peak SC discharge : {max_psc:.1f} W")
print("========================")
