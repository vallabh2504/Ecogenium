"""
Power & Velocity analysis — uniform-time-grid resampling fix.

Root cause of previous artefacts:
  - GPS recorded in duty-cycle bursts (~2 s on, ~12 s off)
  - Dense in-burst clusters (dt < 1 ms) + 12 s blackouts → irregular grid
  - movmean(N points) on irregular grid ≠ movmean over N seconds
  → Fix: interpolate to a uniform 5 Hz grid FIRST, then smooth, then differentiate
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ── Vehicle parameters ────────────────────────────────────────────────────────
vehicle_mass            = 175.0   # kg
gravity                 = 9.81
drag_coefficient        = 0.15
frontal_area            = 0.8     # m²  (standalone _1.m parameters)
rolling_resistance_coef = 0.006
air_density             = 1.225
drivetrain_efficiency   = 0.95
motor_efficiency        = 0.96
motor_power_rated       = 760     # W   (2 × 380 W continuous)
FC_MAX_POWER            = 1040    # W   (FC Stack LC 52.30 nominal max)

RESAMPLE_HZ  = 5          # uniform grid frequency
SMOOTH_SEC   = 10         # velocity smoothing window (seconds)
SMOOTH_PTS   = RESAMPLE_HZ * SMOOTH_SEC  # = 50 points on uniform grid

# ── Load & single-lap filter ──────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
raw = pd.read_csv(os.path.join(script_dir, 'canonical_with_incline.csv'))

lap_end = raw['elapsed_sec'].iloc[-1] / 11
raw = raw[raw['elapsed_sec'] <= lap_end].reset_index(drop=True)

t_raw   = raw['elapsed_sec'].values
v_raw   = raw['Speed_kmh'].values / 3.6        # m/s
inc_raw = raw['inc_angle_deg'].values

print("=== RAW GPS DATA ===")
print(f"  Points       : {len(t_raw)}")
print(f"  Duration     : {t_raw[-1]/60:.2f} min")
print(f"  Distance     : {raw['Distance_km'].iloc[-1]:.3f} km")
dt_raw = np.diff(t_raw)
print(f"  dt: mean={dt_raw.mean():.3f}s  min={dt_raw.min():.4f}s  max={dt_raw.max():.2f}s")
print(f"  Gaps > 1 s   : {(dt_raw > 1.0).sum()}")
print(f"  Gaps < 0.01s : {(dt_raw < 0.01).sum()}")

# ── Uniform-grid resampling ───────────────────────────────────────────────────
t_u = np.arange(t_raw[0], t_raw[-1], 1.0 / RESAMPLE_HZ)   # 5 Hz grid

# Linear interpolation fills the 12-second GPS blackouts
v_u   = np.interp(t_u, t_raw, v_raw)
inc_u = np.interp(t_u, t_raw, inc_raw)

print(f"\n=== UNIFORM GRID ({RESAMPLE_HZ} Hz) ===")
print(f"  Points       : {len(t_u)}")
print(f"  dt constant  : {1/RESAMPLE_HZ:.3f} s")

# ── Smoothing & differentiation on the uniform grid ──────────────────────────
def movmean(x, w):
    return pd.Series(x).rolling(w, center=True, min_periods=1).mean().values

v_smooth = movmean(v_u, SMOOTH_PTS)                     # 10-second smoothing
accel    = np.gradient(v_smooth, 1.0 / RESAMPLE_HZ)     # clean central diff

# ── Power calculation ─────────────────────────────────────────────────────────
P_rolling  = rolling_resistance_coef * vehicle_mass * gravity * v_smooth
P_aero     = 0.5 * drag_coefficient * frontal_area * air_density * v_smooth**3
P_accel    = vehicle_mass * accel * v_smooth
P_gradient = vehicle_mass * gravity * np.sin(np.deg2rad(inc_u)) * v_smooth

P_wheel = P_rolling + P_aero + P_accel + P_gradient
P_motor = np.where(P_wheel > 0, P_wheel / drivetrain_efficiency, 0.0)
P_elec  = np.where(P_motor > 0, P_motor / motor_efficiency, 0.0)
P_elec  = np.clip(np.where(np.isfinite(P_elec), P_elec, 0.0), 0, None)

# Component breakdown (only positive contributions)
P_aero_s  = movmean(np.maximum(P_aero,  0) / drivetrain_efficiency, SMOOTH_PTS)
P_roll_s  = movmean(np.maximum(P_rolling,0) / drivetrain_efficiency, SMOOTH_PTS)
P_accel_s = movmean(np.maximum(P_accel, 0) / drivetrain_efficiency, SMOOTH_PTS)
P_grad_s  = movmean(np.maximum(P_gradient,0) / drivetrain_efficiency, SMOOTH_PTS)

# Energy
dt_u          = 1.0 / RESAMPLE_HZ
energy_Wh     = np.sum(P_elec) * dt_u / 3600
energy_per_km = energy_Wh / raw['Distance_km'].iloc[-1]
moving        = v_smooth > 0.5   # m/s threshold

# Energy breakdown
e_roll  = np.sum(np.maximum(P_roll_s,  0)) * dt_u / 3600
e_aero  = np.sum(np.maximum(P_aero_s,  0)) * dt_u / 3600
e_accel = np.sum(np.maximum(P_accel_s, 0)) * dt_u / 3600
e_grad  = np.sum(np.maximum(P_grad_s,  0)) * dt_u / 3600
e_total = e_roll + e_aero + e_accel + e_grad

print(f"\n=== POWER & ENERGY RESULTS ===")
print(f"  Peak demand  : {P_elec.max():.1f} W")
print(f"  Mean (moving): {P_elec[moving].mean():.1f} W")
print(f"  Total energy : {energy_Wh:.2f} Wh  ({energy_per_km:.2f} Wh/km)")
print(f"\n  Energy breakdown:")
print(f"    Rolling    : {e_roll:.2f} Wh  ({e_roll/e_total*100:.1f}%)")
print(f"    Aero       : {e_aero:.2f} Wh  ({e_aero/e_total*100:.1f}%)")
print(f"    Accel      : {e_accel:.2f} Wh  ({e_accel/e_total*100:.1f}%)")
print(f"    Gradient   : {e_grad:.2f} Wh  ({e_grad/e_total*100:.1f}%)")

results_dir = os.path.join(script_dir, '..', '..', 'results')
os.makedirs(results_dir, exist_ok=True)

t_min_u   = t_u   / 60
t_min_raw = t_raw / 60

# =============================================================================
# FIGURE 1: VELOCITY PROFILE
# =============================================================================
fig1, axes = plt.subplots(2, 2, figsize=(14, 9))
fig1.suptitle('Velocity Profile Analysis — GPS Raw vs Uniform-Grid Resampled',
              fontsize=13, fontweight='bold')

# Panel 1: Full lap — raw dots vs smoothed line
ax = axes[0, 0]
ax.scatter(t_min_raw, v_raw * 3.6, s=6, color='steelblue', alpha=0.5,
           zorder=2, label=f'Raw GPS ({len(t_raw)} pts, irregular)')
ax.plot(t_min_u, v_smooth * 3.6, color='crimson', linewidth=2,
        label=f'Uniform 5 Hz + {SMOOTH_SEC}s smooth', zorder=3)
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Speed (km/h)', fontsize=11)
ax.set_title('Full Lap: Raw GPS vs Smoothed', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min_u[-1]])
ax.set_ylim([-2, 42])

# Panel 2: Launch zone zoom (0–35 s) — show the 12 s blackout
ax = axes[0, 1]
zoom_mask_raw = t_raw <= 35
zoom_mask_u   = t_u   <= 35
ax.scatter(t_min_raw[zoom_mask_raw], v_raw[zoom_mask_raw] * 3.6, s=20,
           color='steelblue', alpha=0.8, zorder=3, label='Raw GPS points')
ax.plot(t_min_u[zoom_mask_u], v_u[zoom_mask_u] * 3.6,
        color='orange', linewidth=1.5, linestyle='--',
        label='Linear interp (fills blackout)', zorder=2)
ax.plot(t_min_u[zoom_mask_u], v_smooth[zoom_mask_u] * 3.6,
        color='crimson', linewidth=2.5, label='After 10-s smooth', zorder=4)
# Annotate the gap
ax.axvspan(10.447/60, 22.646/60, color='yellow', alpha=0.4,
           label='12.2 s GPS blackout')
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Speed (km/h)', fontsize=11)
ax.set_title('Launch Zone (0–35 s): GPS Blackout Visible', fontsize=12, fontweight='bold')
ax.legend(fontsize=8, loc='upper left')
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, 35/60])
ax.set_ylim([-2, 42])

# Panel 3: Acceleration profile (clean on uniform grid)
ax = axes[1, 0]
ax.plot(t_min_u, accel, color='purple', linewidth=1.5, label='Acceleration (m/s²)')
ax.axhline(0, color='black', linewidth=0.8, linestyle='-')
ax.axhline( 2.0, color='red',   linestyle=':', linewidth=1.5, label='±2 m/s²')
ax.axhline(-2.0, color='red',   linestyle=':', linewidth=1.5)
ax.fill_between(t_min_u, accel, 0, where=accel > 0,
                color='green', alpha=0.25, label='Positive (motor)')
ax.fill_between(t_min_u, accel, 0, where=accel < 0,
                color='red',   alpha=0.20, label='Negative (coast/brake)')
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Acceleration (m/s²)', fontsize=11)
ax.set_title('Smoothed Acceleration Profile', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min_u[-1]])

# Panel 4: Speed histogram — GPS raw vs resampled
ax = axes[1, 1]
ax.hist(v_raw[v_raw > 0.5] * 3.6, bins=40, color='steelblue', alpha=0.6,
        density=True, label='Raw GPS')
ax.hist(v_smooth[v_smooth > 0.5] * 3.6, bins=40, color='crimson', alpha=0.6,
        density=True, label='Resampled + smoothed')
ax.set_xlabel('Speed (km/h)', fontsize=11)
ax.set_ylabel('Probability density', fontsize=11)
ax.set_title('Speed Distribution (moving only)', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
vel_path = os.path.join(results_dir, 'velocity_profile_analysis.png')
fig1.savefig(vel_path, dpi=150, bbox_inches='tight')
print(f"\nSaved: {vel_path}")

# =============================================================================
# FIGURE 2: POWER vs TIME
# =============================================================================
fig2, axes = plt.subplots(2, 2, figsize=(14, 9))
fig2.suptitle(
    f"Power Analysis (Uniform 5 Hz resampling) — m={vehicle_mass:.0f} kg  |  "
    f"Peak: {P_elec.max():.0f} W   Mean: {P_elec[moving].mean():.0f} W   "
    f"{energy_per_km:.2f} Wh/km",
    fontsize=12, fontweight='bold'
)

# Panel 1: Total power vs time
ax = axes[0, 0]
ax.plot(t_min_u, P_elec, color='black', linewidth=1.5, label='Electrical demand')
ax.axhline(motor_power_rated, color='red',  linestyle='--', linewidth=2,
           label=f'{motor_power_rated} W motor rated (continuous)')
ax.axhline(FC_MAX_POWER,      color='blue', linestyle='--', linewidth=2,
           label=f'{FC_MAX_POWER} W FC nominal max')
ax.fill_between(t_min_u, P_elec, FC_MAX_POWER,
                where=P_elec > FC_MAX_POWER,
                color='red', alpha=0.25, label='Exceeds FC max (buffer needed)')
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Electrical Power (W)', fontsize=11)
ax.set_title('Total Electrical Power Demand — Single Lap', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min_u[-1]])
ax.set_ylim([0, max(P_elec.max() * 1.15, motor_power_rated * 1.15)])

# Panel 2: Component breakdown
ax = axes[0, 1]
ax.plot(t_min_u, P_aero_s,  color=[1, 0.5, 0],    linewidth=1.5, label='Aero drag')
ax.plot(t_min_u, P_roll_s,  color=[0.8, 0.2, 0.2], linewidth=1.5, label='Rolling resist.')
ax.plot(t_min_u, P_accel_s, color=[0.2, 0.6, 0.9], linewidth=1.5, label='Inertial (accel)')
ax.plot(t_min_u, P_grad_s,  color=[0.2, 0.8, 0.2], linewidth=1.5, label='Gradient')
ax.plot(t_min_u, P_elec,    color='black',          linewidth=2,   label='Total')
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Power (W)', fontsize=11)
ax.set_title('Power Component Breakdown', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min_u[-1]])

# Panel 3: Energy breakdown bar chart
ax = axes[1, 0]
labels = ['Rolling', 'Aero', 'Accel', 'Gradient']
vals   = [e_roll, e_aero, e_accel, e_grad]
colors = [[0.8, 0.2, 0.2], [1, 0.5, 0], [0.2, 0.6, 0.9], [0.2, 0.8, 0.2]]
bars = ax.bar(labels, vals, color=colors, edgecolor='black', linewidth=0.7)
for bar, val in zip(bars, vals):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
            f'{val:.2f} Wh\n({val/e_total*100:.1f}%)',
            ha='center', va='bottom', fontsize=10, fontweight='bold')
ax.set_ylabel('Energy (Wh)', fontsize=11)
ax.set_title(f'Energy Breakdown — 1 Lap ({energy_Wh:.2f} Wh total)', fontsize=12, fontweight='bold')
ax.grid(True, linestyle='--', alpha=0.6, axis='y')
ax.set_ylim([0, max(vals) * 1.3])

# Panel 4: Power histogram
ax = axes[1, 1]
ax.hist(P_elec[P_elec > 10], bins=50, color=[0.2, 0.6, 0.9],
        edgecolor='black', linewidth=0.5)
mean_p = P_elec[moving].mean()
ax.axvline(mean_p,       color='red',  linestyle='--', linewidth=2,
           label=f'Mean: {mean_p:.0f} W')
ax.axvline(FC_MAX_POWER, color='blue', linestyle='--', linewidth=2,
           label=f'FC Max: {FC_MAX_POWER} W')
ax.axvline(motor_power_rated, color='orange', linestyle='--', linewidth=2,
           label=f'Motor rated: {motor_power_rated} W')
ax.set_xlabel('Power (W)', fontsize=11)
ax.set_ylabel('Frequency', fontsize=11)
ax.set_title('Power Demand Distribution', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
pwr_path = os.path.join(results_dir, 'power_vs_time_resampled.png')
fig2.savefig(pwr_path, dpi=150, bbox_inches='tight')
print(f"Saved: {pwr_path}")
