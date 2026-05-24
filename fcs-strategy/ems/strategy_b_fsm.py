import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ems_core import (
    simulate_race, FC_P_MIN, FC_P_MAX,
    SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    N_LAPS, DT, RESULTS_DIR,
    fc_current, K_H2,
)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── SOC thresholds (with hysteresis) ──────────────────────────────────────────
SOC_H_HI = 0.85
SOC_HI   = 0.75
SOC_REF  = 0.60
SOC_LO   = 0.45
SOC_L_LO = 0.20

# ── Power thresholds ──────────────────────────────────────────────────────────
P_LO   = 150.0
P_MED  = 500.0
P_HI   = 900.0
P_PEAK = 1013.0

# Minimum timesteps in a state before a transition is permitted (10 × 0.2 s = 2 s)
MIN_DWELL = 10


def next_state(prev_state, P_dem, SOC, dwell_count):
    if prev_state == 0:       # IDLE
        if SOC < SOC_L_LO:
            return 3
        if SOC < SOC_LO or P_dem > P_MED:
            return 1
        if P_dem < P_LO and SOC < SOC_HI:
            return 0
        if SOC > SOC_H_HI:
            return 0
        return 1

    elif prev_state == 1:     # CRUISE
        if SOC < SOC_L_LO or P_dem > P_PEAK:
            return 3
        if SOC < SOC_LO or P_dem > P_HI:
            return 2
        if P_dem < P_LO and SOC > SOC_REF - 0.05:
            return 0
        if SOC > SOC_H_HI and P_dem < P_LO:
            return 0
        return 1

    elif prev_state == 2:     # HIGH
        if SOC < SOC_L_LO or P_dem > P_PEAK:
            return 3
        if SOC > SOC_HI and P_dem < P_MED:
            return 1
        if P_dem < P_LO and SOC > SOC_HI:
            return 0
        return 2

    else:                     # MAX
        if SOC > SOC_LO + 0.05 and P_dem < P_HI:
            return 2
        if SOC > SOC_HI and P_dem < P_MED:
            return 1
        return 3


def make_strategy(P_fc_S0):
    STATE_POWER = [P_fc_S0, 700.0, 900.0, 1013.0]
    state = [1]   # start in CRUISE
    dwell = [0]
    state_log = []

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        dwell[0] += 1
        if dwell[0] >= MIN_DWELL:
            new_state = next_state(state[0], P_dem, SOC, dwell[0])
            if new_state != state[0]:
                state[0] = new_state
                dwell[0] = 0
        state_log.append(state[0])
        return STATE_POWER[state[0]]

    # Expose log for post-run analysis
    strategy_fn.state_log = state_log
    return strategy_fn


# ── Charge-sustenance tuning (bisection on P_fc_S0) ──────────────────────────
print("Tuning P_fc_S0 for charge sustenance …")
TOLERANCE = 0.01
P_fc_S0 = 100.0
lo, hi   = 50.0, 300.0
best_result   = None
best_strategy = None

for iteration in range(25):
    strat = make_strategy(P_fc_S0)
    result = simulate_race(strat, SOC_0=SC_SOC_0, verbose=False)
    delta_soc = result['delta_SOC']
    print(f"  Iter {iteration+1:2d}  P_fc_S0={P_fc_S0:.1f} W  ΔSOC={delta_soc:+.4f}  H2={result['m_H2_total']:.3f} g")

    if abs(delta_soc) <= TOLERANCE:
        best_result   = result
        best_strategy = strat
        break

    # Bisection: net discharge → raise idle power; net charge → lower it
    if delta_soc < -TOLERANCE:
        lo = P_fc_S0
    else:
        hi = P_fc_S0
    P_fc_S0 = (lo + hi) / 2
    best_result   = result
    best_strategy = strat

if best_result is None:
    best_result   = result
    best_strategy = strat

# ── Derived metrics ───────────────────────────────────────────────────────────
delta_soc  = best_result['delta_SOC']
m_H2       = best_result['m_H2_total']
lap_SOC    = best_result['lap_SOC']
t_min      = best_result['t'] / 60.0
P_dem_arr  = best_result['P_dem']
P_fc_arr   = best_result['P_fc']
P_sc_arr   = best_result['P_sc']
SOC_arr    = best_result['SOC']
I_fc_arr   = best_result['I_fc']
mean_pfc   = float(np.mean(P_fc_arr))

# Cumulative H2 [g]
cum_H2 = np.cumsum(K_H2 * I_fc_arr * DT)

# State distribution from log
state_log = np.array(best_strategy.state_log)
n_total   = len(state_log)
pct_s0 = 100.0 * np.sum(state_log == 0) / n_total
pct_s1 = 100.0 * np.sum(state_log == 1) / n_total
pct_s2 = 100.0 * np.sum(state_log == 2) / n_total
pct_s3 = 100.0 * np.sum(state_log == 3) / n_total

# ── Figure ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
fig.suptitle(
    f"Strategy B — Rule-Based FSM | ΔSOC={delta_soc:+.4f} | H2={m_H2:.3f} g | P_idle={P_fc_S0:.1f} W",
    fontsize=12
)

# Panel 1: Power flows
ax1 = axes[0]
for lvl in [P_fc_S0, 700, 900, 1013]:
    ax1.axhline(lvl, color='lightgrey', linewidth=0.8, linestyle='--')
ax1.plot(t_min, P_dem_arr, color='black',  linewidth=0.7, label='P_dem')
ax1.plot(t_min, P_fc_arr,  color='blue',   linewidth=0.8, label='P_fc')
ax1.plot(t_min, P_sc_arr,  color='orange', linewidth=0.7, label='P_sc')
ax1.set_ylabel('Power [W]')
ax1.legend(loc='upper right', fontsize=8)
ax1.set_title('Power Flows')

# Panel 2: SOC
ax2 = axes[1]
ax2.plot(t_min, SOC_arr, color='green', linewidth=0.8)
ax2.axhline(0.60, color='blue',  linestyle='--', linewidth=0.8, label='SOC_ref 0.60')
ax2.axhline(0.15, color='red',   linestyle='--', linewidth=0.8, label='floor 0.15')
ax2.axhline(0.95, color='purple',linestyle='--', linewidth=0.8, label='ceiling 0.95')
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
out_path = os.path.join(RESULTS_DIR, 'strategy_b_fsm_results.png')
plt.savefig(out_path, dpi=150)
plt.close()
print(f"\nFigure saved → {out_path}")

# ── Final print block ─────────────────────────────────────────────────────────
print()
print("=== Strategy B: Rule-Based FSM ===")
print(f"  Tuned P_fc_IDLE   : {P_fc_S0:.1f} W")
print(f"  Final ΔSOC        : {delta_soc:+.4f}  (target |ΔSOC| < 0.01)")
print(f"  Total H2 consumed : {m_H2:.3f} g")
print(f"  Charge sustained  : {'YES' if abs(delta_soc) <= 0.01 else 'NO'}")
print(f"  Convergence iters : {iteration+1}")
print(f"  Min lap SOC       : {min(lap_SOC):.3f}")
print(f"  Max lap SOC       : {max(lap_SOC):.3f}")
print(f"  Mean P_fc         : {mean_pfc:.1f} W")
print(f"  State distribution: S0={pct_s0:.1f}% S1={pct_s1:.1f}% S2={pct_s2:.1f}% S3={pct_s3:.1f}%")
print("==================================")
