"""
Strategy E — Adaptive ECMS (A-ECMS) for Hydraix I.

Extends Strategy C (ECMS) by replacing the fixed equivalence factor s with
a real-time adaptive update based on SOC deviation from the reference:

    s(t) = s_nominal + K_adapt × (SOC_ref − SOC(t))

where:
  - s_nominal = 0.7896 is the charge-sustaining equilibrium value from Strategy C
  - K_adapt > 0 is the adaptation gain (tuned by bisection)
  - SOC_ref = SC_SOC_0 = 0.60

When SOC < SOC_ref → s increases → Hamiltonian penalises SC discharge more
                    → FC commanded higher → SC recharges toward reference.
When SOC > SOC_ref → s decreases → FC backs off → SC drains toward reference.

The Hamiltonian and minimisation are identical to Strategy C:

    H(P_fc) = K_H2 · I_fc(P_fc) + s(t) · (P_dem − P_fc) / ECMS_DENOM

Solved analytically using the precomputed dI/dP_fc table (no discrete grid argmin).

Reference: Musardo et al. (2005), "A-ECMS: An Adaptive Algorithm for Hybrid
           Electric Vehicle Energy Management."
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


# ── Nominal equivalence factor from Strategy C bisection ─────────────────────
s_nominal = 0.7896


# ── Local simulator: P_fc ∈ [0, FC_P_MAX] (FC fully off allowed) ─────────────
def simulate_race_e(strategy_fn, SOC_0=SC_SOC_0, verbose=False):
    """
    Race simulator identical to strategy_c_ecms.simulate_race_ecms() EXCEPT
    uses the A-ECMS strategy function. P_fc ∈ [0, FC_P_MAX] (FC can be off).

      - Ramp-up limited by FC_RAMP; instantaneous shut-down is allowed
      - SC floor protection forces FC on if SC would drop below SC_SOC_MIN
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


# ── Analytical Hamiltonian minimizer (identical to Strategy C) ────────────────
# Precompute high-resolution dI_fc/dP_fc table.
# The optimality condition dH/dP_fc = 0 reduces to:
#     K_H2 · dI_fc/dP_fc  =  s / ECMS_DENOM
# Solved by scalar interpolation in O(1).
#
# P_fc = 0 is also a valid candidate: H(0) = s·P_dem/ECMS_DENOM
# Minimizer compares H(0) with H(P_fc*) and takes the lower one.
_P_HI  = np.linspace(0.1, FC_P_MAX, 100_000)   # high-res grid (excludes 0)
_I_HI  = fc_current(_P_HI)                      # I_fc [A]
_dI_HI = np.gradient(_I_HI, _P_HI[1] - _P_HI[0])  # dI_fc/dP_fc [A/W]


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


# ── Strategy factory ──────────────────────────────────────────────────────────
def make_strategy(K_adapt):
    """
    Return an A-ECMS strategy function with the given adaptation gain K_adapt.

    At each timestep:
        s(t) = s_nominal + K_adapt × (SOC_ref − SOC(t))
        s(t) is clamped to [0.1, 8.0] to prevent runaway.

    The Hamiltonian minimization is then identical to Strategy C.
    """
    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        s_t = s_nominal + K_adapt * (SC_SOC_0 - SOC)
        s_t = max(0.1, min(s_t, 8.0))   # clamp to prevent runaway
        return _opt_pfc(s_t, P_dem, SOC)
    return strategy_fn


# ── Bisection on K_adapt for charge sustenance ────────────────────────────────
# Physics:
#   K_adapt too low  → s adapts weakly → SC drifts → ΔSOC non-zero
#   K_adapt too high → aggressive s swings → may cause oscillation or poor efficiency
#   Bisection finds K_adapt such that |ΔSOC| < 0.01
TOLERANCE = 0.01
K_adapt   = 5.0        # initial guess
lo, hi    = 0.0, 30.0
best_result = None
best_K_adapt = K_adapt

print("Tuning A-ECMS adaptation gain K_adapt for charge sustenance (|ΔSOC| < 0.01) ...")
print(f"  s_nominal = {s_nominal}  |  SOC_ref = {SC_SOC_0}")
converged = False
conv_iters = 0

for iteration in range(25):
    result    = simulate_race_e(make_strategy(K_adapt), SOC_0=SC_SOC_0, verbose=False)
    delta_soc = result['delta_SOC']
    print(f"  Iter {iteration+1:2d}  K_adapt={K_adapt:.4f}  ΔSOC={delta_soc:+.4f}  H2={result['m_H2_total']:.3f} g")

    if abs(delta_soc) <= TOLERANCE:
        best_result  = result
        best_K_adapt = K_adapt
        converged    = True
        conv_iters   = iteration + 1
        break

    # Bisection:
    # If ΔSOC < 0 (SC net-discharged) → need stronger s adaptation → increase K_adapt
    # If ΔSOC > 0 (SC net-charged)    → K_adapt too strong → decrease K_adapt
    if delta_soc < -TOLERANCE:
        lo = K_adapt
    else:
        hi = K_adapt
    K_adapt = (lo + hi) / 2
    best_result  = result
    best_K_adapt = K_adapt

if not converged:
    conv_iters = 25
    best_result = result
    best_K_adapt = K_adapt

K_adapt = best_K_adapt


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

# Cumulative H2 [g]
cum_H2 = np.cumsum(K_H2 * I_fc * DT)

# km/m³ H2 efficiency
H2_DENSITY = 89.88   # g/m³ at STP
km_per_m3  = 159.5 / (m_H2 / H2_DENSITY)

# Derived statistics
min_soc    = float(np.min(lap_SOC[1:]))
max_soc    = float(np.max(lap_SOC[1:]))
mean_pfc   = float(np.mean(P_fc))
pct_off    = 100.0 * float(np.sum(P_fc == 0.0)) / len(P_fc)


# ── 4-panel figure ────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
fig.suptitle(
    f"Strategy E — Adaptive ECMS  |  K_adapt={K_adapt:.4f}  |  "
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
ax1.axhline(SC_SOC_0,   color='black',  lw=0.8, ls='--', label=f'SOC_ref={SC_SOC_0:.2f}')
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
out_path = os.path.join(RESULTS_DIR, 'strategy_e_aecms_results.png')
fig.savefig(out_path, dpi=150)
plt.close(fig)
print(f"\nFigure saved → {out_path}")


# ── Summary ───────────────────────────────────────────────────────────────────
print()
print("=== Strategy E: Adaptive ECMS ===")
print(f"  s_nominal           : {s_nominal}")
print(f"  Tuned K_adapt       : {K_adapt:.4f}")
print(f"  Final ΔSOC          : {delta_soc:+.4f}")
print(f"  Total H2 consumed   : {m_H2:.3f} g")
print(f"  km per m³ H2        : {km_per_m3:.1f}  (159.5 km / ({m_H2:.3f} g / {H2_DENSITY} g/m³))")
print(f"  Charge sustained    : {'YES' if abs(delta_soc) <= 0.01 else 'NO'}")
print(f"  Convergence iters   : {conv_iters}")
print("=================================")
print("Comparison:")
print("  Strategy A (LUT):  8.204 g  1747.9 km/m³")
print("  Strategy B (FSM):  8.425 g  1701.8 km/m³")
print("  Strategy C (ECMS): 8.197 g  1749.4 km/m³")
print("  Strategy D (4×4):  8.487 g  1691.5 km/m³")
print(f"  Strategy E (A-ECMS): {m_H2:.3f} g  {km_per_m3:.1f} km/m³")
