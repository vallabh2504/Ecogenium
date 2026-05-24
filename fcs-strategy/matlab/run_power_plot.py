"""
Power & Velocity analysis — uniform-time-grid resampling + GPS speed calibration.

Processing chain:
  1. Uniform 5 Hz resampling (fills GPS duty-cycle blackouts via linear interp)
  2. 10-second velocity smoothing on the regular grid
  3. GPS speed calibration: scale × 0.906354 so 11 laps = exactly 14.5 km
     (Silesia Ring Shell Eco-marathon official course: 1,283 m/lap × 11 = 14.11 km;
      user target 14.5 km → scale = 14.5 / (unscaled smoothed 11-lap distance 15.998 km) = 0.906)

Motor: BAFANG RM G060.1000 6T 90A 48V (serial 12403050001, test date 2024-03-05)
  Peak output: 1213.38 W at T=31.4 N·m, 369 r/min, V=48.31 V, I=30.10 A
  Peak efficiency: 83.4 % (coincides with peak-power point)
  Efficiency varies 22 % (no-load) → 83.4 % (rated) — embedded as 61-point lookup
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
frontal_area            = 0.8     # m²
rolling_resistance_coef = 0.006
air_density             = 1.225
drivetrain_efficiency   = 0.95
FC_MAX_POWER            = 1040    # W   (FC Stack LC 52.30 nominal max)

# ── BAFANG RM G060.1000 6T 90A 48V — efficiency lookup ───────────────────────
# 61 test points: normal operating region (~380 r/min, torque 0.6→31.4 N·m)
# Source: factory test record 12403050001, 2024-03-05
motor_power_rated = 1213.0  # W  (peak output: 1213.38 W at T=31.4 N·m, 369 r/min)

_M_POUT = np.array([
     24.19,  24.32,  24.25,  28.44,  28.30,  32.42,  36.48,  40.42,
     44.70,  52.41,  56.89,  64.51,  72.95,  76.80,  84.89,  92.97,
     96.76, 113.48, 116.92, 129.35, 137.79, 144.77, 173.82, 177.86,
    206.16, 218.28, 229.22, 258.04, 258.71, 290.29, 310.45, 320.87,
    353.88, 357.90, 390.07, 414.20, 421.14, 461.25, 477.30, 498.74,
    541.47, 550.61, 590.51, 628.07, 636.72, 676.51, 716.31, 728.37,
    771.91, 815.46, 830.83, 874.14, 921.39, 947.83, 972.64,1030.08,
   1070.29,1091.75,1127.55,1170.18,1213.38
])
_M_ETA = np.array([
    0.220, 0.218, 0.220, 0.251, 0.249, 0.279, 0.298, 0.314,
    0.335, 0.383, 0.395, 0.423, 0.453, 0.452, 0.481, 0.492,
    0.500, 0.565, 0.531, 0.571, 0.569, 0.582, 0.634, 0.620,
    0.666, 0.652, 0.669, 0.673, 0.706, 0.713, 0.719, 0.713,
    0.747, 0.728, 0.757, 0.764, 0.745, 0.772, 0.767, 0.765,
    0.792, 0.776, 0.791, 0.809, 0.786, 0.799, 0.811, 0.793,
    0.806, 0.816, 0.803, 0.813, 0.824, 0.817, 0.811, 0.826,
    0.828, 0.821, 0.819, 0.824, 0.834
])
# Pre-sort by P_out (required by np.interp)
_M_IDX   = np.argsort(_M_POUT)
_M_POUT_S = _M_POUT[_M_IDX]
_M_ETA_S  = _M_ETA[_M_IDX]

def motor_eta_vec(p_out_W):
    """η (0–1) from BAFANG RM G060.1000 test data. Vectorized, handles arrays."""
    return np.interp(np.maximum(p_out_W, 0.0), _M_POUT_S, _M_ETA_S,
                     left=_M_ETA_S[0], right=_M_ETA_S[-1])

RESAMPLE_HZ  = 5
SMOOTH_SEC   = 10
SMOOTH_PTS   = RESAMPLE_HZ * SMOOTH_SEC   # 50 points

# GPS speed calibration — scales raw speed so 11-lap smoothed profile = 14.5 km
# (0.87 would give only 13.9 km; exact value computed numerically below)
GPS_SPEED_SCALE = 14.5 / 11   # target km/lap — resolved inside the script

# ── Load & single-lap filter ──────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
raw = pd.read_csv(os.path.join(script_dir, 'canonical_with_incline.csv'))

lap_end = raw['elapsed_sec'].iloc[-1] / 11
raw = raw[raw['elapsed_sec'] <= lap_end].reset_index(drop=True)

t_raw   = raw['elapsed_sec'].values
v_raw   = raw['Speed_kmh'].values / 3.6        # m/s  (unscaled)
inc_raw = raw['inc_angle_deg'].values

dt_raw = np.diff(t_raw)
print("=== RAW GPS DATA (unscaled) ===")
print(f"  Points       : {len(t_raw)}")
print(f"  Duration     : {t_raw[-1]/60:.2f} min")
print(f"  GPS distance : {raw['Distance_km'].iloc[-1]:.3f} km/lap")
print(f"  dt: mean={dt_raw.mean():.3f}s  min={dt_raw.min():.4f}s  max={dt_raw.max():.2f}s")

# ── Uniform-grid resampling & smoothing helper ────────────────────────────────
def movmean(x, w):
    return pd.Series(x).rolling(w, center=True, min_periods=1).mean().values

def smoothed_dist_km(speed_scale):
    t_u_ = np.arange(t_raw[0], t_raw[-1], 1.0 / RESAMPLE_HZ)
    v_u_ = np.interp(t_u_, t_raw, v_raw * speed_scale)
    v_s_ = movmean(v_u_, SMOOTH_PTS)
    return float(np.trapezoid(v_s_, t_u_) / 1000)   # km per lap

# ── Exact GPS scale factor — binary search so 11 laps = 14.5 km ──────────────
TARGET_TOTAL_KM = 14.5
target_lap_km   = TARGET_TOTAL_KM / 11

lo, hi = 0.80, 1.05
for _ in range(60):
    mid = (lo + hi) / 2
    if smoothed_dist_km(mid) < target_lap_km: lo = mid
    else: hi = mid
GPS_SPEED_SCALE = (lo + hi) / 2

print(f"\n=== GPS SPEED CALIBRATION ===")
print(f"  Target total : {TARGET_TOTAL_KM:.2f} km  ({target_lap_km*1000:.1f} m/lap × 11)")
print(f"  Exact scale  : {GPS_SPEED_SCALE:.6f}")
print(f"  Check (×11)  : {smoothed_dist_km(GPS_SPEED_SCALE)*11:.4f} km")

# ── Build the uniform grid with calibrated speed ──────────────────────────────
t_u   = np.arange(t_raw[0], t_raw[-1], 1.0 / RESAMPLE_HZ)
v_u   = np.interp(t_u, t_raw, v_raw * GPS_SPEED_SCALE)   # scaled + interpolated
inc_u = np.interp(t_u, t_raw, inc_raw)

print(f"\n=== UNIFORM GRID ({RESAMPLE_HZ} Hz, calibrated) ===")
print(f"  Points       : {len(t_u)}")
print(f"  Max speed    : {v_u.max()*3.6:.1f} km/h  (was {v_raw.max()*3.6:.1f} km/h)")

# ── Smoothing & differentiation ───────────────────────────────────────────────
v_smooth = movmean(v_u, SMOOTH_PTS)
accel    = np.gradient(v_smooth, 1.0 / RESAMPLE_HZ)

# ── Power calculation ─────────────────────────────────────────────────────────
P_rolling  = rolling_resistance_coef * vehicle_mass * gravity * v_smooth
P_aero     = 0.5 * drag_coefficient * frontal_area * air_density * v_smooth**3
P_accel    = vehicle_mass * accel * v_smooth
P_gradient = vehicle_mass * gravity * np.sin(np.deg2rad(inc_u)) * v_smooth

P_wheel = P_rolling + P_aero + P_accel + P_gradient
P_motor = np.where(P_wheel > 0, P_wheel / drivetrain_efficiency, 0.0)   # shaft output
eta_m   = motor_eta_vec(P_motor)                                          # BAFANG lookup
P_elec  = np.where(P_motor > 0, P_motor / eta_m, 0.0)
P_elec  = np.clip(np.where(np.isfinite(P_elec), P_elec, 0.0), 0, None)

# Component breakdown (only positive contributions)
P_aero_s  = movmean(np.maximum(P_aero,  0) / drivetrain_efficiency, SMOOTH_PTS)
P_roll_s  = movmean(np.maximum(P_rolling,0) / drivetrain_efficiency, SMOOTH_PTS)
P_accel_s = movmean(np.maximum(P_accel, 0) / drivetrain_efficiency, SMOOTH_PTS)
P_grad_s  = movmean(np.maximum(P_gradient,0) / drivetrain_efficiency, SMOOTH_PTS)

# Energy
dt_u          = 1.0 / RESAMPLE_HZ
lap_dist_km   = float(np.trapezoid(v_smooth, t_u) / 1000)   # from calibrated smooth speed
energy_Wh     = np.sum(P_elec) * dt_u / 3600
energy_per_km = energy_Wh / lap_dist_km
moving        = v_smooth > 0.5   # m/s threshold

# Energy breakdown
e_roll  = np.sum(np.maximum(P_roll_s,  0)) * dt_u / 3600
e_aero  = np.sum(np.maximum(P_aero_s,  0)) * dt_u / 3600
e_accel = np.sum(np.maximum(P_accel_s, 0)) * dt_u / 3600
e_grad  = np.sum(np.maximum(P_grad_s,  0)) * dt_u / 3600
e_total = e_roll + e_aero + e_accel + e_grad

mean_motor_eta = float(np.mean(eta_m[P_motor > 10]))
eta_at_peak    = float(motor_eta_vec(P_motor.max()))

print(f"\n=== POWER & ENERGY RESULTS (calibrated — 14.5 km / 11 laps) ===")
print(f"  GPS scale    : {GPS_SPEED_SCALE:.4f}×")
print(f"  Lap distance : {lap_dist_km*1000:.1f} m  (×11 = {lap_dist_km*11:.3f} km)")
print(f"  Peak speed   : {v_smooth.max()*3.6:.1f} km/h")
print(f"  Peak demand  : {P_elec.max():.1f} W  (FC max: {FC_MAX_POWER} W)")
print(f"  Mean (moving): {P_elec[moving].mean():.1f} W")
print(f"  Total energy : {energy_Wh:.2f} Wh/lap  ({energy_per_km:.2f} Wh/km)")
print(f"  Over 11 laps : {energy_Wh*11:.2f} Wh  ({energy_Wh*11/1000:.3f} kWh)")
print(f"\n  Motor (BAFANG RM G060.1000):")
print(f"    Rated power  : {motor_power_rated:.0f} W")
print(f"    η at peak    : {eta_at_peak*100:.1f}%  (P_shaft={P_motor.max():.0f} W)")
print(f"    η mean (run) : {mean_motor_eta*100:.1f}%  (vs old fixed 96%)")
print(f"\n  Energy breakdown:")
print(f"    Rolling    : {e_roll:.2f} Wh  ({e_roll/e_total*100:.1f}%)")
print(f"    Aero       : {e_aero:.2f} Wh  ({e_aero/e_total*100:.1f}%)")
print(f"    Accel      : {e_accel:.2f} Wh  ({e_accel/e_total*100:.1f}%)")
print(f"    Gradient   : {e_grad:.2f} Wh  ({e_grad/e_total*100:.1f}%)")

results_dir = os.path.join(script_dir, '..', 'results')
os.makedirs(results_dir, exist_ok=True)

t_min_u   = t_u   / 60
t_min_raw = t_raw / 60

# =============================================================================
# FIGURE 1: VELOCITY PROFILE
# =============================================================================
fig1, axes = plt.subplots(2, 2, figsize=(14, 9))
fig1.suptitle(
    f'Velocity Profile — GPS Calibrated ({GPS_SPEED_SCALE:.4f}×) · '
    f'11 laps = {lap_dist_km*11:.2f} km  ·  max {v_smooth.max()*3.6:.1f} km/h',
    fontsize=13, fontweight='bold'
)

# Panel 1: Full lap — raw dots vs smoothed line
ax = axes[0, 0]
ax.scatter(t_min_raw, v_raw * GPS_SPEED_SCALE * 3.6, s=6, color='steelblue', alpha=0.5,
           zorder=2, label=f'Raw GPS calibrated ({GPS_SPEED_SCALE:.3f}×, {len(t_raw)} pts)')
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
ax.scatter(t_min_raw[zoom_mask_raw], v_raw[zoom_mask_raw] * GPS_SPEED_SCALE * 3.6, s=20,
           color='steelblue', alpha=0.8, zorder=3, label='Raw GPS points (calibrated)')
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
ax.hist(v_raw[v_raw > 0.5] * GPS_SPEED_SCALE * 3.6, bins=40, color='steelblue', alpha=0.6,
        density=True, label=f'Raw GPS (calibrated {GPS_SPEED_SCALE:.3f}×)')
ax.hist(v_smooth[v_smooth > 0.5] * 3.6, bins=40, color='crimson', alpha=0.6,
        density=True, label='Resampled + smoothed')
ax.set_xlabel('Speed (km/h)', fontsize=11)
ax.set_ylabel('Probability density', fontsize=11)
ax.set_title('Speed Distribution (moving only)', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
vel_path = os.path.join(results_dir, 'velocity_profile_calibrated.png')
fig1.savefig(vel_path, dpi=150, bbox_inches='tight')
print(f"\nSaved: {vel_path}")

# =============================================================================
# FIGURE 2: POWER vs TIME
# =============================================================================
fig2, axes = plt.subplots(2, 2, figsize=(14, 9))
fig2.suptitle(
    f"Power Analysis — m={vehicle_mass:.0f} kg · GPS scale {GPS_SPEED_SCALE:.3f}× · "
    f"11 laps = {lap_dist_km*11:.2f} km  |  "
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
pwr_path = os.path.join(results_dir, 'power_vs_time_calibrated.png')
fig2.savefig(pwr_path, dpi=150, bbox_inches='tight')
print(f"Saved: {pwr_path}")

# =============================================================================
# FIGURE 3: MOTOR EFFICIENCY MAP — BAFANG RM G060.1000
# =============================================================================
# Full torque-speed-efficiency dataset from factory test record (all 106 points)
_M_T_ALL = np.array([
     0.6,  0.6,  0.6,  0.7,  0.7,  0.8,  0.9,  1.0,  1.1,  1.3,  1.4,
     1.6,  1.8,  1.9,  2.1,  2.3,  2.4,  2.8,  2.9,  3.2,  3.4,  3.6,
     4.3,  4.4,  5.1,  5.4,  5.7,  6.4,  6.4,  7.2,  7.7,  8.0,  8.8,
     8.9,  9.7, 10.3, 10.5, 11.5, 11.9, 12.5, 13.5, 13.8, 14.8, 15.7,
    16.0, 17.0, 18.0, 18.4, 19.5, 20.6, 21.1, 22.2, 23.4, 24.2, 24.9,
    26.3, 27.4, 28.1, 29.1, 30.2, 31.4, 32.2, 33.3, 34.3, 35.5, 36.7,
    37.7, 38.7, 39.8, 41.0, 41.9, 42.8, 43.8, 45.1, 45.8, 47.5, 48.3,
    49.4, 50.6, 51.9, 52.0, 53.8, 54.2, 55.9, 56.9, 57.7, 58.8, 59.9,
    61.1, 61.7, 63.7, 64.4, 65.4, 66.0, 67.5, 68.4, 70.1, 70.6, 71.8,
    72.9, 73.6, 75.1, 75.8, 76.2, 76.7, 76.6
])
_M_N_ALL = np.array([
    385, 387, 386, 388, 386, 387, 387, 386, 388, 385, 388,
    385, 387, 386, 386, 386, 385, 387, 385, 386, 387, 384,
    386, 386, 386, 386, 384, 386, 385, 385, 385, 383, 384,
    384, 384, 384, 383, 383, 383, 381, 383, 381, 381, 382,
    380, 380, 380, 378, 378, 378, 376, 376, 376, 374, 373,
    374, 373, 371, 370, 370, 369, 354, 342, 331, 318, 308,
    299, 290, 282, 273, 266, 258, 251, 243, 237, 231, 225,
    218, 213, 207, 202, 197, 193, 187, 182, 178, 173, 169,
    165, 160, 157, 153, 148, 145, 141, 137, 134, 130, 124,
    115, 106,  97,  87,  68,  12,   0
])
_M_ETA_ALL = np.array([
    22.0, 21.8, 22.0, 25.1, 24.9, 27.9, 29.8, 31.4, 33.5, 38.3, 39.5,
    42.3, 45.3, 45.2, 48.1, 49.2, 50.0, 56.5, 53.1, 57.1, 56.9, 58.2,
    63.4, 62.0, 66.6, 65.2, 66.9, 70.6, 67.3, 71.3, 71.9, 71.3, 74.7,
    72.8, 75.7, 76.4, 74.5, 77.2, 76.7, 76.5, 79.2, 77.6, 79.1, 80.9,
    78.6, 79.9, 81.1, 79.3, 80.6, 81.6, 80.3, 81.3, 82.4, 81.7, 81.1,
    82.6, 82.8, 82.1, 81.9, 82.4, 83.4, 83.1, 83.0, 82.8, 82.2, 82.3,
    82.1, 81.7, 81.7, 81.5, 81.2, 80.5, 80.2, 79.9, 79.1, 80.0, 79.1,
    78.4, 78.5, 78.2, 76.5, 77.1, 76.2, 76.1, 75.4, 74.8, 74.1, 73.8,
    73.5, 72.0, 72.9, 71.8, 70.6, 69.8, 69.4, 68.3, 68.5, 67.0, 65.8,
    64.4, 62.4, 61.3, 57.9, 50.9, 16.6,  0.0
])

fig3, axes3 = plt.subplots(1, 3, figsize=(18, 6))
fig3.suptitle(
    'BAFANG RM G060.1000 6T 90A 48V — Motor Efficiency Analysis',
    fontsize=14, fontweight='bold'
)

# Panel 1: Efficiency vs output power (test data + drive-cycle operating cloud)
ax = axes3[0]
p_curve = np.linspace(0, 1213, 400)
eta_curve = motor_eta_vec(p_curve) * 100
ax.plot(p_curve, eta_curve, color='navy', linewidth=2.5, label='BAFANG test data (61 pts)')
ax.axvline(motor_power_rated, color='red',    linestyle='--', linewidth=1.5,
           label=f'Rated: {motor_power_rated:.0f} W  (η={motor_eta_vec(motor_power_rated)*100:.1f}%)')
ax.axvline(FC_MAX_POWER,      color='darkorange', linestyle='--', linewidth=1.5,
           label=f'FC max: {FC_MAX_POWER} W')
# Operating cloud from drive cycle
ax.scatter(P_motor[P_motor > 5], eta_m[P_motor > 5] * 100,
           s=4, color='crimson', alpha=0.35, zorder=3, label='Drive-cycle operating pts')
ax.set_xlabel('Motor shaft output power (W)', fontsize=11)
ax.set_ylabel('Motor efficiency (%)', fontsize=11)
ax.set_title('η vs P_out — Test Data + Drive Cycle', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, 1300])
ax.set_ylim([0, 95])

# Panel 2: Torque–speed characteristic (all 106 test points, coloured by efficiency)
ax = axes3[1]
sc = ax.scatter(_M_T_ALL, _M_N_ALL, c=_M_ETA_ALL, cmap='RdYlGn',
                vmin=0, vmax=85, s=30, zorder=3)
cb = plt.colorbar(sc, ax=ax)
cb.set_label('Efficiency (%)', fontsize=10)
# Mark rated (peak-power) point
ax.scatter([31.4], [369], color='navy', s=120, marker='*', zorder=5,
           label=f'Rated: 31.4 N·m, 369 r/min\n1213 W, η=83.4%')
ax.set_xlabel('Torque (N·m)', fontsize=11)
ax.set_ylabel('Speed (r/min)', fontsize=11)
ax.set_title('Torque–Speed Map (colour = efficiency)', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)

# Panel 3: Instantaneous motor efficiency over the drive cycle
ax = axes3[2]
moving_mask = P_motor > 10
ax.plot(t_min_u, eta_m * 100, color='steelblue', linewidth=1.0, alpha=0.7,
        label='Instantaneous η')
ax.axhline(mean_motor_eta * 100, color='red', linestyle='--', linewidth=2,
           label=f'Mean (running): {mean_motor_eta*100:.1f}%')
ax.axhline(83.4, color='green', linestyle=':', linewidth=1.5,
           label='Peak (datasheet): 83.4%')
ax.fill_between(t_min_u, eta_m * 100, mean_motor_eta * 100,
                where=eta_m > mean_motor_eta, color='green', alpha=0.15)
ax.fill_between(t_min_u, eta_m * 100, mean_motor_eta * 100,
                where=eta_m < mean_motor_eta, color='red',   alpha=0.15)
ax.set_xlabel('Time (minutes)', fontsize=11)
ax.set_ylabel('Motor efficiency (%)', fontsize=11)
ax.set_title('Instantaneous η Over Drive Cycle', fontsize=12, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_xlim([0, t_min_u[-1]])
ax.set_ylim([0, 95])

plt.tight_layout()
eff_path = os.path.join(results_dir, 'motor_efficiency_map.png')
fig3.savefig(eff_path, dpi=150, bbox_inches='tight')
print(f"Saved: {eff_path}")
