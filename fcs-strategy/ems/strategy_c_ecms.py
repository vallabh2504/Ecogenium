"""
Strategy C — ECMS (Equivalent Consumption Minimization Strategy) for Hydraix I.

Theoretically optimal EMS derived from Pontryagin's Minimum Principle.
At each timestep, minimises the scalar Hamiltonian:

    H(P_fc) = ṁ_H2_fc(P_fc)  +  s · (P_dem − P_fc) / ECMS_DENOM

where s is the co-state (equivalence factor) tuned offline by bisection to
enforce charge sustenance (ΔSOC ≈ 0) over the full 11-lap race.

Key design choices
──────────────────
1. FC can be fully off (P_fc = 0): ems_core.simulate_race(fc_can_off=True) is
   used, which allows P_fc = 0 during glide phases and correctly sets I_fc = 0
   when P_fc < FC_P_MIN (preventing fc_current() clip artefacts).

2. H(0) vs H(P_fc*) comparison in the continuous minimizer:
   When P_dem=0: H(0) = 0, H(P_fc*) = K_H2·I(P_fc*) − s·P_fc*/ECMS_DENOM.
   H(P_fc*) < H(0) means running the FC to charge SC is "free" in equivalent
   H2 cost — valid ECMS behavior when s > s_breakeven ≈ 0.70.  The optimizer
   evaluates both candidates and picks the lower Hamiltonian.

3. SOC boundary masking: hard gates block SC actions that would violate
   SC_SOC_MAX (block further charging) or SC_SOC_MIN (block discharging)
   when within 3 % of the respective hard limit.

4. Bisection uses lo=0.1, hi=5.0 with continuous interpolated P_fc* so
   ΔSOC(s) varies smoothly — avoids discrete steps from coarse grids.

Reference: Paganelli et al. (2002), "General supervisory control policy for
           the energy optimisation in charge-sustaining hybrid electric vehicles."
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ems_core import (
    simulate_race, build_demand_profile, fc_current, fc_h2_rate,
    sc_soc_update, sc_voltage,
    SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX, SC_E_J,
    FC_P_MIN, FC_P_MAX, FC_RAMP,
    K_H2, LHV_H2, ETA_FC_REF, ECMS_DENOM,
    N_LAPS, DT, RESULTS_DIR
)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ── Continuous Hamiltonian minimizer with P_fc = 0 option ────────────────────
# Pre-build a high-resolution I_fc table and its derivative dI/dP_fc.
# The optimality condition dH/dP_fc = 0 reduces to:
#     K_H2 · dI_fc/dP_fc  =  s / ECMS_DENOM
# Precomputing dI/dP_fc allows us to solve this by scalar interpolation in O(1).
#
# P_fc = 0 is also a valid candidate: H(0) = s·P_dem/ECMS_DENOM
# (no H2 burned, SC provides P_dem entirely).
# The minimizer compares H(0) with H(P_fc*) and takes the lower one.
_P_HI  = np.linspace(0.1, FC_P_MAX, 100_000)   # high-res grid (excludes 0)
_I_HI  = fc_current(_P_HI)                      # I_fc [A]
_dI_HI = np.gradient(_I_HI, _P_HI[1] - _P_HI[0])  # dI_fc/dP_fc [A/W]

# Also keep a 201-point candidate array for reference / plotting
_P_CANDS = np.linspace(0.0, FC_P_MAX, 201)


def _opt_pfc(s, P_dem, SOC):
    """
    Solve dH/dP_fc = 0 for the interior optimum P_fc*, then compare with P=0.

    The interior optimum satisfies:
        K_H2·dI/dP_fc = s/ECMS_DENOM

    Falls back to the boundary (FC_P_MAX) if dI/dP_fc is monotone and the
    target slope lies outside the achievable range.

    Returns the P_fc (in [0, FC_P_MAX]) that minimises H, after also applying
    SOC boundary masking to prevent hard-limit violations.
    """
    target = s / (K_H2 * ECMS_DENOM)
    dI_min, dI_max = _dI_HI.min(), _dI_HI.max()

    if target <= dI_min:
        P_interior = _P_HI[0]   # H monotonically increasing → minimum at lower bound
    elif target >= dI_max:
        P_interior = FC_P_MAX   # H monotonically decreasing → minimum at upper bound
    else:
        g = _dI_HI - target
        sign_changes = np.where(np.diff(np.sign(g)))[0]
        if len(sign_changes) == 0:
            P_interior = float(_P_HI[np.argmin(np.abs(g))])
        else:
            i = sign_changes[0]
            P_interior = float(np.interp(0.0, [g[i], g[i + 1]], [_P_HI[i], _P_HI[i + 1]]))

    # Evaluate H at the interior optimum and at P_fc = 0
    I_int  = float(fc_current(P_interior))
    H_int  = K_H2 * I_int + s * (P_dem - P_interior) / ECMS_DENOM
    H_zero = s * P_dem / ECMS_DENOM   # I=0 when P_fc=0, no H2 burned

    # Choose the lower-H candidate
    P_best = 0.0 if H_zero <= H_int else P_interior

    # SOC boundary masking: override if the chosen action would violate limits
    P_sc_best = P_dem - P_best
    if SOC >= SC_SOC_MAX - 0.03 and P_sc_best < -10.0:
        # SC nearly full: block further charging → move FC toward demand
        P_best = float(np.clip(P_dem, 0.0, FC_P_MAX))
    if SOC <= SC_SOC_MIN + 0.03 and P_sc_best > 10.0:
        # SC nearly empty: block further discharging → raise FC toward demand
        P_best = float(np.clip(P_dem, 0.0, FC_P_MAX))

    return float(P_best)


def make_strategy(s):
    """
    Return a strategy function that minimises H(P_fc) for equivalence factor s.

    H(P_fc) = K_H2·I_fc(P_fc)  +  s·(P_dem − P_fc) / ECMS_DENOM

    Units of both terms: [g/s] (hydrogen mass flow equivalent).
    - Higher s → SC discharge penalised more → FC commanded higher → SOC recovers.
    - Lower s  → SC discharge tolerated     → FC runs leaner   → SOC depletes.

    Ramp-rate and saturation constraints are applied by simulate_race()
    after this function returns.
    """
    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        return _opt_pfc(s, P_dem, SOC)

    return strategy_fn


# ── Standalone analysis (bisection + plots) ───────────────────────────────────
if __name__ == '__main__':

    # ── Charge-sustenance bisection on s ──────────────────────────────────────
    # Physics:
    #   s too low  → H penalises SC usage weakly → SC drains  → ΔSOC < 0 → raise s
    #   s too high → H over-penalises SC usage   → SC charges → ΔSOC > 0 → lower s
    TOLERANCE = 0.01
    s      = 1.5        # initial guess
    lo, hi = 0.1, 5.0
    best_result = None

    print("Tuning ECMS equivalence factor s for charge sustenance (|ΔSOC| < 0.01) ...")
    for iteration in range(30):
        result    = simulate_race(make_strategy(s), SOC_0=SC_SOC_0, verbose=False, fc_can_off=True)
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


    # ── Extract arrays ────────────────────────────────────────────────────────
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

    # ── Derived statistics ────────────────────────────────────────────────────
    mean_pfc   = float(np.mean(P_fc))
    pct_off    = 100.0 * float(np.sum(P_fc == 0.0)) / len(P_fc)
    in_band    = (P_fc >= 700.0) & (P_fc <= 900.0)
    pct_cruise = 100.0 * float(np.sum(in_band)) / len(P_fc)
    min_soc    = float(np.min(lap_SOC[1:]))
    max_soc    = float(np.max(lap_SOC[1:]))

    # Cumulative H2 [g]
    cum_H2 = np.cumsum(K_H2 * I_fc * DT)


    # ── Figure ────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(
        f"Strategy C — ECMS (FC-off glide allowed)  |  s={s:.4f}  |  "
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


    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print("=== Strategy C: ECMS (FC-off glide allowed) ===")
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
