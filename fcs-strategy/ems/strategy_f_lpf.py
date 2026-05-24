"""
Strategy F — Low-Pass Filter EMS for Hydraix I (SEM 2026).

Decomposes the demand signal into frequency components:
  - FC handles the slow/DC component (low-pass filtered demand)
  - SC absorbs fast transients (high-frequency residual)

A 1st-order IIR filter with τ=30s smooths over ~5× the mean demand burst
length, removing transients while preserving the lap-average power level.
SOC feedback corrects for long-term drift. K_soc is tuned by bisection to
achieve charge sustenance (|ΔSOC| < 0.01) over the full 11-lap race.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ems_core import (
    build_demand_profile, fc_current, fc_h2_rate,
    sc_soc_update, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    FC_P_MAX, FC_RAMP, K_H2,
    N_LAPS, DT, RESULTS_DIR
)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ── Filter constant ────────────────────────────────────────────────────────────
TAU   = 30.0                  # time constant [s] — smooths ~5× burst length
ALPHA = np.exp(-DT / TAU)    # IIR coefficient ≈ 0.9934


# ── Local simulator: allows P_fc ∈ [0, FC_P_MAX] (FC can be fully off) ────────
def simulate_race_f(strategy_fn, SOC_0=SC_SOC_0, verbose=False):
    """
    Race simulator identical to ems_core.simulate_race() EXCEPT:
      - P_fc is clamped to [0, FC_P_MAX] (FC can be fully shut down)
      - When P_fc_cmd == 0: I_fc = 0, H2 flow = 0 (truly off)
      - Ramp-up limited to FC_RAMP × DT per step; instant shut-down allowed
      - SC floor protection still forces FC on if SC would drop below SC_SOC_MIN
    """
    t_lap, P_lap = build_demand_profile()
    N_pts = len(t_lap)
    T_lap = float(t_lap[-1])
    t_all = np.concatenate([t_lap + i * T_lap for i in range(N_LAPS)])
    P_dem = np.tile(P_lap, N_LAPS)
    n     = N_LAPS * N_pts

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

        # Ramp rate: up is limited, instantaneous shut-down is allowed
        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k = Pd - P_fc_cmd

        # SC floor protection: if SC would deplete, force FC to cover demand
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = min(FC_P_MAX, Pd)
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(max(P_fc_cmd, 1e-6)) if P_fc_cmd > 0 else 0.0)
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev     = P_fc_cmd

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


# ── Strategy factory ───────────────────────────────────────────────────────────
def make_strategy(K_soc):
    """
    Create a LPF-EMS strategy function with SOC feedback gain K_soc [W/SOC].

    The filter state P_filt is maintained in a mutable container and resets
    at the start of each race (t_in_lap < DT and lap_idx == 0).
    """
    state = {'P_filt': 0.0}   # filter state — resets each make_strategy call

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        # Reset filter at start of race to avoid slow startup transient
        if t_in_lap < DT and lap_idx == 0:
            state['P_filt'] = P_dem

        # Update 1st-order IIR low-pass filter
        state['P_filt'] = ALPHA * state['P_filt'] + (1.0 - ALPHA) * P_dem

        # SOC feedback correction
        P_cmd = state['P_filt'] + K_soc * (SC_SOC_0 - SOC)

        # True glide override: FC off when demand is near zero and SC not depleted
        if P_dem < 25.0 and SOC >= SC_SOC_0 - 0.05:
            P_cmd = 0.0

        return float(np.clip(P_cmd, 0.0, FC_P_MAX))

    return strategy_fn


# ── Charge-sustenance tuning via bisection on K_soc ───────────────────────────
# K_soc range discovered empirically: needs ~5000-10000 W/SOC because the
# glide override (FC off at low demand) causes SC net discharge that K_soc
# must correct. Low K_soc (<3000) cannot compensate enough.
print("Tuning K_soc for charge sustenance ...")
TOLERANCE = 0.01
K_soc     = 6000.0
lo, hi    = 1000.0, 15000.0
best_result   = None
iters_done    = 0

for iteration in range(25):
    strat  = make_strategy(K_soc)
    result = simulate_race_f(strat, SOC_0=SC_SOC_0, verbose=False)
    delta_soc = result['delta_SOC']
    print(f"  Iter {iteration+1:2d}  K_soc={K_soc:.1f} W/SOC  ΔSOC={delta_soc:+.4f}  H2={result['m_H2_total']:.3f} g")
    iters_done = iteration + 1

    if abs(delta_soc) <= TOLERANCE:
        best_result = result
        break

    if delta_soc < -TOLERANCE:
        lo = K_soc   # net discharge → raise K_soc to increase FC output
    else:
        hi = K_soc   # net charge → lower K_soc to reduce FC output
    K_soc = (lo + hi) / 2
    best_result = result

if best_result is None:
    best_result = result


# ── Derived metrics ───────────────────────────────────────────────────────────
delta_soc  = best_result['delta_SOC']
m_H2       = best_result['m_H2_total']
t_min      = best_result['t'] / 60.0
P_dem_arr  = best_result['P_dem']
P_fc_arr   = best_result['P_fc']
P_sc_arr   = best_result['P_sc']
SOC_arr    = best_result['SOC']
I_fc_arr   = best_result['I_fc']
mean_pfc   = float(np.mean(P_fc_arr))

# Low-pass filtered demand (for reporting the mean P_filt)
# Recompute P_filt trace for mean reporting
P_filt_trace = np.empty(len(P_dem_arr))
pf = P_dem_arr[0]
for i, pd in enumerate(P_dem_arr):
    if i == 0:
        pf = pd
    pf = ALPHA * pf + (1.0 - ALPHA) * pd
    P_filt_trace[i] = pf
mean_pfilt = float(np.mean(P_filt_trace))

# Cumulative H2 [g]
cum_H2 = np.cumsum(K_H2 * I_fc_arr * DT)

# km per m³ H2 (H2 density at STP = 89.88 g/m³)
H2_DENSITY = 89.88   # g/m³ at STP
km_per_m3  = 159.5 / (m_H2 / H2_DENSITY)

charge_sustained = abs(delta_soc) <= TOLERANCE


# ── 4-panel figure ────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
fig.suptitle(
    f"Strategy F — Low-Pass Filter EMS | ΔSOC={delta_soc:+.4f} | H2={m_H2:.3f} g | τ=30s K_soc={K_soc:.1f}",
    fontsize=12
)

# Panel 1: Power flows
ax1 = axes[0]
ax1.plot(t_min, P_dem_arr, color='black',     linewidth=0.5, label='P_dem')
ax1.plot(t_min, P_fc_arr,  color='steelblue', linewidth=0.8, label='P_fc (LPF)')
ax1.plot(t_min, P_sc_arr,  color='orange',    linewidth=0.6, label='P_sc')
ax1.set_ylabel('Power [W]')
ax1.legend(loc='upper right', fontsize=8)
ax1.set_title('Power Flows')

# Panel 2: SOC
ax2 = axes[1]
ax2.plot(t_min, SOC_arr, color='green', linewidth=0.8)
ax2.axhline(SC_SOC_0,   color='blue',   linestyle='--', linewidth=0.8, label=f'SOC_ref {SC_SOC_0:.2f}')
ax2.axhline(SC_SOC_MIN, color='red',    linestyle='--', linewidth=0.8, label=f'floor {SC_SOC_MIN:.2f}')
ax2.axhline(SC_SOC_MAX, color='purple', linestyle='--', linewidth=0.8, label=f'ceiling {SC_SOC_MAX:.2f}')
ax2.set_ylabel('SC SOC')
ax2.set_ylim(0, 1)
ax2.legend(loc='upper right', fontsize=8)
ax2.set_title('Supercapacitor SOC')

# Panel 3: FC current
ax3 = axes[2]
ax3.plot(t_min, I_fc_arr, color='red', linewidth=0.7)
ax3.set_ylabel('I_fc [A]')
ax3.set_title('FC Stack Current')

# Panel 4: Cumulative H2
ax4 = axes[3]
ax4.plot(t_min, cum_H2, color='darkorange', linewidth=0.8)
ax4.set_ylabel('H2 consumed [g]')
ax4.set_xlabel('Time [min]')
ax4.set_title('Cumulative H2 Consumed')

plt.tight_layout()
out_path = os.path.join(RESULTS_DIR, 'strategy_f_lpf_results.png')
plt.savefig(out_path, dpi=150)
plt.close()
print(f"\nFigure saved → {out_path}")


# ── Final print block ─────────────────────────────────────────────────────────
print()
print("=== Strategy F: Low-Pass Filter EMS ===")
print(f"  Filter time const τ : {TAU:.1f} s  (α={ALPHA:.4f})")
print(f"  Tuned K_soc         : {K_soc:.1f} W/SOC")
print(f"  Final ΔSOC          : {delta_soc:+.4f}")
print(f"  Total H2 consumed   : {m_H2:.3f} g")
print(f"  km per m³ H2        : {km_per_m3:.1f}  (159.5 km / (H2_g / {H2_DENSITY} g/m³))")
print(f"  Charge sustained    : {'YES' if charge_sustained else 'NO'}")
print(f"  Convergence iters   : {iters_done}")
print(f"  Mean P_fc           : {mean_pfc:.1f} W")
print(f"  Mean P_filt (LP)    : {mean_pfilt:.1f} W  (should ≈ race avg 259 W)")
print("========================================")
print("Comparison:")
print("  Strategy A (LUT):    8.204 g  1747.9 km/m³")
print("  Strategy B (FSM):    8.425 g  1701.8 km/m³")
print("  Strategy C (ECMS):   8.197 g  1749.4 km/m³")
print("  Strategy D (4×4):    8.487 g  1691.5 km/m³")
print(f"  Strategy F (LPF):    {m_H2:.3f} g  {km_per_m3:.1f} km/m³")
