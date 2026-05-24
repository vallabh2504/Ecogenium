"""
Power vs Time analysis — Python port of ecogenium_energy_analysis_1.m (v1.2 fixed).
Exact same physics and signal processing as the corrected Matlab script.
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ── Parameters (matching fixed ecogenium_energy_analysis_1.m) ────────────────
vehicle_mass            = 175.0   # kg  (corrected from 160)
wheel_radius            = 0.295   # m
gravity                 = 9.81
drag_coefficient        = 0.15
frontal_area            = 0.8     # m²
rolling_resistance_coef = 0.006
air_density             = 1.225
drivetrain_efficiency   = 0.95    # corrected from 0.85
motor_power_rated       = 760     # W  (2 × 380 W)
motor_efficiency        = 0.96
FC_MAX_POWER            = 1040    # W  (FC Stack LC 52.30 nominal max)

# ── Load data ─────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
data = pd.read_csv(os.path.join(script_dir, 'canonical_with_incline.csv'))

# Single-lap filter
single_lap_duration = data['elapsed_sec'].iloc[-1] / 11
data = data[data['elapsed_sec'] <= single_lap_duration].reset_index(drop=True)

time        = data['elapsed_sec'].values
speed_kmh   = data['Speed_kmh'].values
incline_deg = data['inc_angle_deg'].values
speed_ms    = speed_kmh / 3.6

print(f"  Lap duration : {time[-1]/60:.2f} min")
print(f"  Lap distance : {data['Distance_km'].iloc[-1]:.3f} km")
print(f"  Data points  : {len(time)}")

# ── Signal processing: smooth velocity BEFORE differentiating ─────────────────
avg_dt          = np.mean(np.diff(time))
avg_sample_rate = 1.0 / avg_dt
vel_window      = max(3, int(round(10 * avg_sample_rate)))   # 10-second window

def movmean(x, w):
    return pd.Series(x).rolling(w, center=True, min_periods=1).mean().values

speed_ms_smooth = movmean(speed_ms, vel_window)

dt = np.diff(time)
dt[dt < 0.01] = avg_dt    # protect against GPS timestamp gaps
dt = np.concatenate([[dt[0]], dt])

dv           = np.diff(speed_ms_smooth)
accel_raw    = np.concatenate([[0], dv / dt[1:]])
acceleration = movmean(accel_raw, 30)  # 30-point post-smooth

# ── Power calculation ─────────────────────────────────────────────────────────
P_rolling  = rolling_resistance_coef * vehicle_mass * gravity * speed_ms_smooth
P_aero     = 0.5 * drag_coefficient * frontal_area * air_density * speed_ms_smooth**3
P_accel    = vehicle_mass * acceleration * speed_ms_smooth
P_gradient = vehicle_mass * gravity * np.sin(np.deg2rad(incline_deg)) * speed_ms_smooth

P_wheel = P_rolling + P_aero + P_accel + P_gradient
P_motor = np.where(P_wheel > 0, P_wheel / drivetrain_efficiency, 0.0)
P_elec  = np.where(P_motor > 0, P_motor / motor_efficiency, 0.0)
P_elec  = np.clip(P_elec, 0, None)
P_elec  = np.where(np.isfinite(P_elec), P_elec, 0.0)

# Smoothed component breakdown (for breakdown panel)
win = max(3, int(round(10 * avg_sample_rate)))
P_aero_s  = movmean(np.maximum(P_aero,  0) / drivetrain_efficiency, win)
P_roll_s  = movmean(np.maximum(P_rolling,0) / drivetrain_efficiency, win)
P_accel_s = movmean(np.maximum(P_accel, 0) / drivetrain_efficiency, win)
P_grad_s  = movmean(np.maximum(P_gradient,0) / drivetrain_efficiency, win)

energy_Wh     = np.trapezoid(P_elec, time) / 3600
energy_per_km = energy_Wh / data['Distance_km'].iloc[-1]
moving        = speed_ms_smooth > 0

print(f"  Peak power    : {P_elec.max():.1f} W")
print(f"  Mean power    : {P_elec[moving].mean():.1f} W")
print(f"  Total energy  : {energy_Wh:.2f} Wh  ({energy_per_km:.2f} Wh/km)")

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 9))
fig.suptitle(
    f"Vehicle Power Analysis — m={vehicle_mass:.0f} kg, "
    f"η_dt={drivetrain_efficiency*100:.0f}%  |  "
    f"Peak: {P_elec.max():.0f} W,  Mean: {P_elec[moving].mean():.0f} W,  "
    f"{energy_per_km:.2f} Wh/km",
    fontsize=12, fontweight='bold'
)
t_min = time / 60

# Panel 1: Total Power vs Time
ax = axes[0, 0]
ax.plot(t_min, P_elec, color='black', linewidth=1.5, label='Power Demand')
ax.axhline(motor_power_rated, color='red',   linestyle='--', linewidth=2,
           label=f'{motor_power_rated} W Motor Rated')
ax.axhline(FC_MAX_POWER,      color='blue',  linestyle='--', linewidth=2,
           label=f'{FC_MAX_POWER} W FC Nominal Max')
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Electrical Power (W)', fontsize=11)
ax.set_title('Total Electrical Power Demand — Single Lap', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min[-1]])
ax.set_ylim([0, max(P_elec.max() * 1.15, motor_power_rated * 1.1)])

# Panel 2: Component breakdown
ax = axes[0, 1]
ax.plot(t_min, P_aero_s,  color=[1, 0.5, 0],   linewidth=1.5, label='Aero Drag')
ax.plot(t_min, P_roll_s,  color=[0.8, 0.2, 0.2],linewidth=1.5, label='Rolling')
ax.plot(t_min, P_accel_s, color=[0.2, 0.6, 0.9],linewidth=1.5, label='Inertial')
ax.plot(t_min, P_grad_s,  color=[0.2, 0.8, 0.2],linewidth=1.5, label='Gradient')
ax.plot(t_min, P_elec,    color='black',         linewidth=2,   label='Total')
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Power (W)', fontsize=11)
ax.set_title('Power Component Breakdown', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min[-1]])

# Panel 3: Power vs Speed
ax = axes[1, 0]
sc = ax.scatter(speed_kmh, P_elec, c=time, s=8, cmap='jet', alpha=0.6)
cb = fig.colorbar(sc, ax=ax)
cb.set_label('Time (s)', fontsize=10)
ax.axhline(FC_MAX_POWER, color='blue', linestyle='--', linewidth=1.5,
           label='FC Nominal Max')
ax.set_xlabel('Speed (km/h)', fontsize=11)
ax.set_ylabel('Electrical Power (W)', fontsize=11)
ax.set_title('Power Demand vs Speed', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)

# Panel 4: Power histogram
ax = axes[1, 1]
ax.hist(P_elec[P_elec > 10], bins=40, color=[0.2, 0.6, 0.9], edgecolor='black',
        linewidth=0.5)
mean_p = P_elec[moving].mean()
ax.axvline(mean_p,        color='red',  linestyle='--', linewidth=2,
           label=f'Mean: {mean_p:.0f} W')
ax.axvline(FC_MAX_POWER,  color='blue', linestyle='--', linewidth=2,
           label=f'FC Max: {FC_MAX_POWER} W')
ax.set_xlabel('Power (W)', fontsize=11)
ax.set_ylabel('Frequency', fontsize=11)
ax.set_title('Power Demand Distribution', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()

# ── Save ─────────────────────────────────────────────────────────────────────
results_dir = os.path.join(script_dir, '..', '..', 'results')
os.makedirs(results_dir, exist_ok=True)
out_path = os.path.join(results_dir, 'power_vs_time_single_lap.png')
fig.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"\nSaved: {out_path}")
