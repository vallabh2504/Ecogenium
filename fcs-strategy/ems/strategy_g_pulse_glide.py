"""
Pulse-and-Glide Drive Cycle Generator + Strategy G Comparison for Hydraix I (SEM 2026).

Rationale
---------
The BAFANG motor efficiency curve peaks at high output power (η≈83% at 1213W output)
but drops sharply at low/mid power (η≈57% at 126W). Constant-speed cruising forces
the motor to operate in this inefficient mid-power region.

Pulse-and-Glide (P&G) exploits the efficiency peak:
  - PULSE phase: motor at full throttle (high P_out → high η)
  - GLIDE phase: motor off, vehicle coasts (zero motor electrical loss)
  - Same average speed, lower electrical energy per km, less H₂ consumed.

Profile design constraints:
  - 11 laps × 1318.18 m/lap = 14.5 km total  (exact race distance)
  - Each lap starts from v=0 and ends with controlled braking to v=0
  - Total race duration ≈ 35 min

This script:
  1. Builds a P&G drive cycle via per-lap forward-dynamics simulation.
  2. Grid-sweeps (v_high, v_low, P_pulse) to find optimal P&G parameters.
  3. Runs Strategy G (K_p=1900, K_i=2.0) on both GPS and best P&G profiles.
  4. Computes motor efficiency distributions for both profiles.
  5. Produces a 4-row comparison figure.
  6. Verifies: total distance, duration, complete stops at lap boundaries.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import scipy.signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ems_core import (
    build_demand_profile, fc_current, fc_h2_rate,
    sc_soc_update, SC_SOC_0, SC_SOC_MIN, SC_ETA, SC_E_J,
    FC_P_MAX, FC_RAMP, K_H2,
    DT, RESULTS_DIR, N_LAPS as _N_LAPS,
)
from ems_core import motor_eta
from strategy_g_pi import make_strategy

# ── Vehicle / drivetrain constants (mirror ems_core private values) ────────────
MASS   = 175.0
G      = 9.81
CD     = 0.15
AF     = 0.8
CRR    = 0.006
RHO    = 1.225
ETA_DT = 0.95

# ── Race geometry ──────────────────────────────────────────────────────────────
D_LAP_M     = 14500.0 / _N_LAPS   # 1318.18 m per lap (= 14.5 km / 11 laps)
T_LAP_EQUIV = 190.0                # lap boundary cadence for supervisor [s]
A_BRAKE     = 1.5                  # braking deceleration [m/s²]


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1 — Per-lap forward dynamics simulator
# ═══════════════════════════════════════════════════════════════════════════════

def _build_one_lap(v_high_ms, v_low_ms, P_motor_out_W, P_elec_W,
                   d_lap=D_LAP_M, a_brake=A_BRAKE):
    """
    Simulate one pulse-and-glide lap via forward dynamics.

    Starts from v=0. Runs PULSE/GLIDE until the remaining distance equals the
    kinematic stopping distance, then brakes to a complete stop at d_lap.

    Parameters
    ----------
    v_high_ms     : float  upper velocity threshold [m/s]
    v_low_ms      : float  lower velocity threshold [m/s]
    P_motor_out_W : float  wheel power during PULSE [W]
    P_elec_W      : float  electrical power drawn from bus during PULSE [W]
    d_lap         : float  lap distance [m]
    a_brake       : float  braking deceleration magnitude [m/s²]

    Returns
    -------
    t_arr : ndarray [s]
    v_arr : ndarray [m/s]
    P_arr : ndarray [W]  electrical demand
    """
    v, x, t = 0.0, 0.0, 0.0
    mode = 'PULSE'
    t_lst, v_lst, P_lst = [], [], []

    for _ in range(int(600.0 / DT)):   # safety cap: 600 s per lap
        # ── Record state ──────────────────────────────────────────────────────
        t_lst.append(t)
        v_lst.append(v)

        # ── Brake trigger: start braking when stopping distance ≥ remaining ──
        d_stop    = v**2 / (2.0 * a_brake)
        remaining = d_lap - x

        if mode != 'BRAKE' and remaining <= d_stop + v * DT:
            mode = 'BRAKE'

        # ── Compute new velocity ───────────────────────────────────────────────
        F_drag = 0.5 * CD * AF * RHO * v**2
        F_roll = CRR * MASS * G

        if mode == 'BRAKE':
            P_lst.append(0.0)
            v_new = max(v - a_brake * DT, 0.0)

        elif mode == 'PULSE':
            F_wheel = P_motor_out_W / max(v, 0.3)   # clamp to avoid ÷0 at v=0
            a       = (F_wheel - F_drag - F_roll) / MASS
            P_lst.append(P_elec_W)
            v_new   = min(v + a * DT, v_high_ms)
            if v_new >= v_high_ms:
                mode = 'GLIDE'

        else:   # GLIDE
            a     = -(F_drag + F_roll) / MASS
            P_lst.append(0.0)
            v_new = max(v + a * DT, 0.0)
            if v_new <= v_low_ms:
                v_new = v_low_ms
                mode  = 'PULSE'

        # ── Integrate position (midpoint rule) ────────────────────────────────
        x += (v + v_new) / 2.0 * DT
        v  = v_new
        t += DT

        # ── Lap ends when car reaches complete stop during BRAKE ──────────────
        if mode == 'BRAKE' and v == 0.0:
            t_lst.append(t)
            v_lst.append(0.0)
            P_lst.append(0.0)
            break

    return np.array(t_lst), np.array(v_lst), np.array(P_lst)


def build_pulse_glide_profile(v_high_kmh, v_low_kmh, P_pulse_elec_W,
                               n_laps=_N_LAPS, d_lap=D_LAP_M):
    """
    Build n_laps pulse-and-glide laps, each starting and ending at v=0.

    Total distance ≈ n_laps × d_lap = 14.5 km.
    Each lap is built independently via _build_one_lap().

    Parameters
    ----------
    v_high_kmh     : float  upper velocity threshold [km/h]
    v_low_kmh      : float  lower velocity threshold [km/h]
    P_pulse_elec_W : float  electrical power during PULSE [W]
    n_laps         : int    number of laps  (default = 11)
    d_lap          : float  distance per lap [m]  (default = 14500/11 m)

    Returns
    -------
    t_arr : ndarray [s]    (concatenated across all laps, continuous timeline)
    v_arr : ndarray [m/s]
    P_arr : ndarray [W]    electrical demand
    """
    v_high_ms = v_high_kmh / 3.6
    v_low_ms  = v_low_kmh  / 3.6

    eta_guess   = motor_eta(P_pulse_elec_W * 0.80)
    P_motor_out = P_pulse_elec_W * eta_guess * ETA_DT

    all_t, all_v, all_P = [], [], []
    t_offset = 0.0

    for _ in range(n_laps):
        t_lap, v_lap, P_lap = _build_one_lap(
            v_high_ms, v_low_ms, P_motor_out, P_pulse_elec_W, d_lap
        )
        all_t.append(t_lap + t_offset)
        all_v.append(v_lap)
        all_P.append(P_lap)
        t_offset += float(t_lap[-1]) + DT   # next lap starts one DT after this lap ends

    return np.concatenate(all_t), np.concatenate(all_v), np.concatenate(all_P)


# ═══════════════════════════════════════════════════════════════════════════════
# Cycle-level simulator (allows P_fc ∈ [0, FC_P_MAX])
# ═══════════════════════════════════════════════════════════════════════════════

def simulate_cycle(strategy_fn, t_arr, P_dem_arr):
    """
    Run the Strategy G controller over an arbitrary time / demand array.

    P_fc is clamped to [0, FC_P_MAX] (FC can be fully shut down).
    Ramp-up limited to FC_RAMP × DT per step; instantaneous shut-down allowed.
    SC floor protection forces FC on if SC would drop below SC_SOC_MIN.

    Returns a results dict compatible with strategy_g_pi outputs.
    """
    n            = len(t_arr)
    N_PTS_LAP    = max(1, int(T_LAP_EQUIV / DT))

    P_fc_ar  = np.empty(n)
    P_sc_ar  = np.empty(n)
    SOC_ar   = np.empty(n + 1)
    I_fc_ar  = np.empty(n)
    m_H2_ar  = np.empty(n)

    SOC_ar[0]  = SC_SOC_0
    P_fc_prev  = 0.0

    for k in range(n):
        lap_idx  = k // N_PTS_LAP
        t_in_lap = float(t_arr[k]) % T_LAP_EQUIV
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem_arr[k])

        result   = strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx)
        P_fc_cmd = float(result[0] if isinstance(result, tuple) else result)

        # Ramp: only limit upward; instantaneous shut-down allowed
        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k    = Pd - P_fc_cmd
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        # SC floor protection
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = float(np.clip(Pd, 0.0, FC_P_MAX))
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(max(P_fc_cmd, 1e-6)) if P_fc_cmd > 0 else 0.0)
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev     = P_fc_cmd

    return {
        't':           t_arr,
        'P_dem':       P_dem_arr,
        'P_fc':        P_fc_ar,
        'P_sc':        P_sc_ar,
        'SOC':         SOC_ar[:-1],
        'I_fc':        I_fc_ar,
        'm_H2_total':  float(np.sum(m_H2_ar)),
        'delta_SOC':   float(SOC_ar[n] - SC_SOC_0),
    }


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2 — Grid sweep over (v_high, v_low, P_pulse)
# ═══════════════════════════════════════════════════════════════════════════════

print("=" * 62)
print("  Pulse-and-Glide vs GPS Profile — Strategy G K_p=1900 K_i=2.0")
print("=" * 62)
print(f"\nRace geometry: {_N_LAPS} laps × {D_LAP_M:.2f} m/lap = "
      f"{_N_LAPS * D_LAP_M / 1000:.3f} km total")

v_high_options  = [30, 33, 36, 38]     # km/h
v_low_options   = [18, 21, 24]         # km/h
P_pulse_options = [800, 1000, 1200]    # W electrical (motor input)

print("\nGrid sweep — (v_high, v_low, P_pulse) × Strategy G K_p=1900 K_i=2.0:")
results_grid = []

for v_h in v_high_options:
    for v_l in v_low_options:
        if v_h - v_l < 6:
            continue
        for P_p in P_pulse_options:
            t_arr, v_arr, P_dem = build_pulse_glide_profile(v_h, v_l, P_p)
            res       = simulate_cycle(make_strategy(1900, 2.0), t_arr, P_dem)
            T_total   = float(t_arr[-1])
            D_total   = float(np.trapezoid(v_arr, t_arr))   # m, from velocity integral
            avg_speed = D_total / T_total * 3.6              # km/h
            avg_Pdem  = float(np.mean(P_dem))
            h2        = res['m_H2_total']
            dSOC      = res['delta_SOC']
            km_m3     = 14.5 / (h2 / 89.88) if h2 > 0 else 0.0

            results_grid.append(dict(
                v_high=v_h, v_low=v_l, P_pulse=P_p,
                avg_speed=avg_speed, avg_Pdem=avg_Pdem,
                T_total=T_total, D_total=D_total,
                H2=h2, dSOC=dSOC, km_m3=km_m3,
                t_arr=t_arr, v_arr=v_arr, P_dem=P_dem, res=res,
            ))
            print(f"  v_H={v_h:2d} v_L={v_l:2d} P={P_p:4d}W  "
                  f"{T_total/60:.1f}min  {D_total/1000:.3f}km  "
                  f"avg {avg_speed:5.1f}km/h  P_dem={avg_Pdem:5.0f}W  "
                  f"H2={h2:.3f}g  ΔSOC={dSOC:+.4f}  {km_m3:.1f}km/m³")

# Best = minimum H2 with |ΔSOC| < 0.05
valid = [r for r in results_grid if abs(r['dSOC']) < 0.05]
best  = min(valid, key=lambda r: r['H2']) if valid else \
        min(results_grid, key=lambda r: r['H2'])
print(f"\nBest: v_high={best['v_high']} v_low={best['v_low']} "
      f"P_pulse={best['P_pulse']}W")


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3 — Verification of best P-G profile
# ═══════════════════════════════════════════════════════════════════════════════

print("\n── Verification (best P-G profile) ──────────────────────────────")
t_best = best['t_arr']
v_best = best['v_arr']
P_best = best['P_dem']

T_total_best = float(t_best[-1])
D_total_best = float(np.trapezoid(v_best, t_best)) / 1000.0  # km

# Detect complete stops (v drops to 0 during braking at lap end)
zero_mask  = v_best == 0.0
stop_idxs  = np.where(zero_mask)[0]
# Count distinct stop events (contiguous zero-speed blocks count as one stop)
n_stops = int(np.sum(np.diff(np.concatenate([[0], zero_mask.astype(int)])) == 1))

print(f"  Total distance   : {D_total_best:.3f} km  (need 14.5 km)")
print(f"  Duration         : {T_total_best/60:.1f} min")
print(f"  Avg speed        : {D_total_best / (T_total_best/3600):.2f} km/h"
      f"  (target 24.86 km/h)")
print(f"  Min velocity     : {float(np.min(v_best))*3.6:.2f} km/h  (must be 0.0)")
print(f"  Zero-speed steps : {int(np.sum(zero_mask))}  (need ≥ {_N_LAPS})")
print(f"  Lap-end stops    : {n_stops}  (need {_N_LAPS})")
print()

dist_ok  = abs(D_total_best - 14.5) < 0.05   # allow ≤50 m error
stops_ok = n_stops >= _N_LAPS

if dist_ok and stops_ok:
    print("VERDICT:")
    print(f"  ✓ Distance {D_total_best:.3f} km ≈ 14.5 km  (error "
          f"{abs(D_total_best-14.5)*1000:.0f} m)")
    print(f"  ✓ {n_stops} complete stops  (one at end of every lap)")
else:
    if not dist_ok:
        print(f"  ✗ Distance {D_total_best:.3f} km vs 14.5 km target")
    if not stops_ok:
        print(f"  ✗ Only {n_stops} stops detected (need {_N_LAPS})")


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4 — GPS SEM profile through the same simulate_cycle
# ═══════════════════════════════════════════════════════════════════════════════

print("\nBuilding GPS SEM profile (one lap × 11) …")
t_lap_gps, P_lap_gps = build_demand_profile()
N_LAPS     = _N_LAPS
T_lap_gps  = float(t_lap_gps[-1])
t_gps      = np.concatenate([t_lap_gps + i * T_lap_gps for i in range(N_LAPS)])
P_dem_gps  = np.tile(P_lap_gps, N_LAPS)

print("Running Strategy G on GPS profile …")
res_gps = simulate_cycle(make_strategy(1900, 2.0), t_gps, P_dem_gps)

# Best P-G results
t_pg   = best['t_arr']
v_pg   = best['v_arr']
P_pg   = best['P_dem']
res_pg = best['res']


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 5 — Motor efficiency analysis
# ═══════════════════════════════════════════════════════════════════════════════

mask_gps = P_dem_gps > 10.0
mask_pg  = P_pg       > 10.0

eta_arr_gps = motor_eta(P_dem_gps[mask_gps] * 0.85)
eta_arr_pg  = motor_eta(P_pg[mask_pg]        * 0.85)

mean_eta_gps    = float(np.mean(eta_arr_gps))
mean_eta_pg     = float(np.mean(eta_arr_pg))
active_frac_gps = float(np.sum(mask_gps)) / len(P_dem_gps)
active_frac_pg  = float(np.sum(mask_pg))  / len(P_pg)

# LPF for plotting Row 2
TAU   = 15.0
ALPHA = float(np.exp(-DT / TAU))
b_lpf = [1.0 - ALPHA]
a_lpf = [1.0, -ALPHA]
P_filt_gps = scipy.signal.lfilter(b_lpf, a_lpf, P_dem_gps)
P_filt_pg  = scipy.signal.lfilter(b_lpf, a_lpf, P_pg)

t_min_gps = t_gps / 60.0
t_min_pg  = t_pg  / 60.0


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 6 — 4-row comparison figure
# ═══════════════════════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(16, 12), dpi=150)

gs = fig.add_gridspec(4, 2, hspace=0.52, wspace=0.28)
ax_vel  = fig.add_subplot(gs[0, :])      # Row 1 — full width

ax_pw_gps  = fig.add_subplot(gs[1, 0])
ax_pw_pg   = fig.add_subplot(gs[1, 1])

ax_soc_gps = fig.add_subplot(gs[2, 0])
ax_soc_pg  = fig.add_subplot(gs[2, 1])

ax_eh_gps  = fig.add_subplot(gs[3, 0])
ax_eh_pg   = fig.add_subplot(gs[3, 1])

fig.suptitle(
    f"Pulse-and-Glide vs GPS Profile | Strategy G  K_p=1900  K_i=2.0 | "
    f"Best P-G: v={best['v_low']}–{best['v_high']} km/h  P_pulse={best['P_pulse']}W\n"
    f"P-G: {D_total_best:.3f} km in {T_total_best/60:.1f} min  "
    f"({n_stops} lap stops verified)",
    fontsize=9, y=0.998,
)

# ── Row 1: Velocity overlay (first 3 minutes) ──────────────────────────────
T3_MIN = 3.0
avg_v_gps_kmh = 14.5 / (float(t_gps[-1]) / 3600.0)

t3_mask_pg = t_min_pg <= T3_MIN

ax_vel.plot(t_min_pg[t3_mask_pg],
            v_pg[t3_mask_pg] * 3.6,
            color='firebrick', lw=0.8,
            label=f"Pulse-Glide v={best['v_low']}–{best['v_high']} km/h "
                  f"(stops at lap ends)")
ax_vel.axhline(avg_v_gps_kmh, color='steelblue', lw=1.2, alpha=0.8, ls='-',
               label=f'GPS constant-speed ≈ {avg_v_gps_kmh:.1f} km/h')
ax_vel.axhline(0, color='gray', lw=0.5, ls=':')
ax_vel.set_ylabel('Speed [km/h]')
ax_vel.set_xlabel('Time [min]')
ax_vel.set_xlim(0, T3_MIN)
ax_vel.set_title(
    f'Velocity Profile — first 3 minutes  '
    f'(P-G: {D_total_best:.3f} km / {T_total_best/60:.1f} min, '
    f'{n_stops} complete stops  |  GPS: {avg_v_gps_kmh:.1f} km/h constant)'
)
ax_vel.legend(fontsize=8, loc='upper right')
ax_vel.grid(True, lw=0.3)

# ── Row 2: Power vs time (first 5 minutes) ─────────────────────────────────
T5_MIN = 5.0
t5_gps = t_min_gps <= T5_MIN
t5_pg  = t_min_pg  <= T5_MIN

for ax, mask, t_min, P_dem, P_fc, P_sc, P_filt, label in [
    (ax_pw_gps, t5_gps, t_min_gps, P_dem_gps, res_gps['P_fc'], res_gps['P_sc'],
     P_filt_gps, 'GPS — constant speed'),
    (ax_pw_pg,  t5_pg,  t_min_pg,  P_pg, res_pg['P_fc'], res_pg['P_sc'],
     P_filt_pg,  f"Pulse-Glide {best['v_low']}–{best['v_high']} km/h"),
]:
    ax.plot(t_min[mask], P_dem[mask],  color='black',     lw=0.5, label='P_dem')
    ax.plot(t_min[mask], P_fc[mask],   color='steelblue', lw=0.8, label='P_fc')
    ax.plot(t_min[mask], P_sc[mask],   color='orange',    lw=0.6, label='P_sc')
    ax.plot(t_min[mask], P_filt[mask], color='green',     lw=0.5, ls='--',
            label='P_filt')
    ax.set_ylabel('Power [W]')
    ax.set_xlabel('Time [min]')
    ax.set_title(f'Power Flows — {label}  (first 5 min)')
    ax.legend(fontsize=6, loc='upper right', ncol=2)
    ax.grid(True, lw=0.3)

# ── Row 3: SOC vs time (full race) ─────────────────────────────────────────
from ems_core import SC_SOC_MIN as _SOC_MIN

for ax, t_min, SOC, dSOC, label in [
    (ax_soc_gps, t_min_gps, res_gps['SOC'], res_gps['delta_SOC'],
     'GPS — constant speed'),
    (ax_soc_pg,  t_min_pg,  res_pg['SOC'],  res_pg['delta_SOC'],
     f"Pulse-Glide {best['v_low']}–{best['v_high']} km/h"),
]:
    ax.plot(t_min, SOC, color='green', lw=0.8, label='SOC')
    ax.axhline(SC_SOC_0, color='black', lw=0.8, ls='--',
               label=f'SOC_ref={SC_SOC_0:.2f}')
    ax.axhline(_SOC_MIN,  color='red',   lw=0.8, ls=':',
               label=f'SOC_MIN={_SOC_MIN:.2f}')
    ax.annotate(f'ΔSOC={dSOC:+.4f}',
                xy=(0.02, 0.05), xycoords='axes fraction',
                fontsize=8, color='darkgreen',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.6))
    ax.set_ylabel('SC SOC [—]')
    ax.set_xlabel('Time [min]')
    ax.set_ylim(0, 1)
    ax.set_title(f'SC SOC — {label}')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, lw=0.3)

# ── Row 4: Motor efficiency histograms ─────────────────────────────────────
eta_bins = np.arange(0.40, 0.87, 0.02)

for ax, eta_arr, mean_eta, active_frac, color, label in [
    (ax_eh_gps, eta_arr_gps, mean_eta_gps, active_frac_gps, 'steelblue',
     'GPS — constant speed'),
    (ax_eh_pg,  eta_arr_pg,  mean_eta_pg,  active_frac_pg,  'firebrick',
     f"Pulse-Glide {best['v_low']}–{best['v_high']} km/h"),
]:
    ax.hist(eta_arr, bins=eta_bins, color=color, edgecolor='white',
            linewidth=0.3, alpha=0.85)
    ax.axvline(mean_eta, color='black', lw=1.2, ls='--',
               label=f'Mean η={mean_eta:.1%}')
    ax.set_xlabel('Motor efficiency η')
    ax.set_ylabel('Timestep count')
    ax.set_title(
        f'Motor η Distribution — {label}\n'
        f'Mean η = {mean_eta:.1%}  |  Active fraction = {active_frac:.1%}',
        fontsize=8,
    )
    ax.legend(fontsize=7)
    ax.grid(True, lw=0.3, axis='y')

plt.tight_layout(rect=[0, 0, 1, 0.993])
out_path = os.path.join(RESULTS_DIR, 'strategy_g_pulse_glide.png')
plt.savefig(out_path, dpi=150)
plt.close()
print(f"\nFigure saved → {out_path}")


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 7 — Results table
# ═══════════════════════════════════════════════════════════════════════════════

H2_GPS   = res_gps['m_H2_total']
H2_PG    = res_pg['m_H2_total']
dSOC_gps = res_gps['delta_SOC']
dSOC_pg  = res_pg['delta_SOC']

km_m3_gps = 14.5 / (H2_GPS / 89.88) if H2_GPS > 0 else 0.0
km_m3_pg  = 14.5 / (H2_PG  / 89.88) if H2_PG  > 0 else 0.0

avg_speed_gps = 14.5 / (float(t_gps[-1]) / 3600.0)
avg_speed_pg  = best['avg_speed']
avg_Pdem_gps  = float(np.mean(P_dem_gps))
avg_Pdem_pg   = best['avg_Pdem']

dH2   = H2_PG  - H2_GPS
dPdem = avg_Pdem_pg - avg_Pdem_gps
d_eta = mean_eta_pg - mean_eta_gps
dkm   = km_m3_pg - km_m3_gps

print()
print("=== Pulse-and-Glide vs GPS Constant-Speed Profile ===")
print()
print(f"{'Profile':<22}| {'Avg speed':>9} | {'Avg P_dem':>9} | {'Active%':>7} | "
      f"{'Mean η_motor':>12} | {'H2 [g]':>6} | {'ΔSOC':>8} | {'km/m³':>7}")
print("-" * 22 + "+" + "-" * 11 + "+" + "-" * 11 + "+" + "-" * 9 + "+"
      + "-" * 14 + "+" + "-" * 8 + "+" + "-" * 9 + "+" + "-" * 9)
print(f"{'GPS (constant speed)':<22}| {avg_speed_gps:>8.1f}  | {avg_Pdem_gps:>8.1f}  | "
      f"{active_frac_gps*100:>6.1f}%  | {mean_eta_gps:>11.1%}  | {H2_GPS:>6.3f} | "
      f"{dSOC_gps:>+8.4f} | {km_m3_gps:>7.1f}")
print(f"{'Pulse-Glide optimal':<22}| {avg_speed_pg:>8.1f}  | {avg_Pdem_pg:>8.1f}  | "
      f"{active_frac_pg*100:>6.1f}%  | {mean_eta_pg:>11.1%}  | {H2_PG:>6.3f} | "
      f"{dSOC_pg:>+8.4f} | {km_m3_pg:>7.1f}")
print(f"{'Improvement':<22}| {'':>9} | {dPdem:>+8.1f}  | {'':>7}  | "
      f"{d_eta:>+10.1%}  | {dH2:>+6.3f} | {'—':>8} | {dkm:>+7.1f}")
print()
print(f"Best P-G params: v_low={best['v_low']} km/h, v_high={best['v_high']} km/h, "
      f"P_pulse={best['P_pulse']}W")
print()
print("Grid search summary (all valid results sorted by H2):")
valid_sorted = sorted(valid, key=lambda r: r['H2'])[:5]
for i, r in enumerate(valid_sorted, 1):
    print(f"  {i}. v_H={r['v_high']:2d} v_L={r['v_low']:2d} P={r['P_pulse']:4d}W  "
          f"avg {r['avg_speed']:5.1f}km/h  H2={r['H2']:.3f}g  "
          f"ΔSOC={r['dSOC']:+.4f}  {r['km_m3']:.1f}km/m³")
print()
print("km/m³ formula: 14.5 km / (H2_g / 89.88 g/m³)")
print("=" * 54)
