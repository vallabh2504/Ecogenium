# FCS Strategy — Fuel-Cell System & Energy Management

All modelling, simulation, and strategy work for the Hydraix I fuel-cell powertrain (SEM 2026).

## Folder Map

```
fcs-strategy/
│
├── ems/                                ← Python EMS simulation suite
│   ├── ems_core.py                     Shared physics engine — vehicle model, FC/SC
│   │                                   constants, build_demand_profile(), simulate_race()
│   │
│   ├── strategy_a_lut.py               Strategy A: Continuous LUT         158.8 km/m³
│   ├── strategy_b_fsm.py               Strategy B: Rule-Based FSM          154.6 km/m³
│   ├── strategy_c_ecms.py              Strategy C: ECMS / PMP              158.9 km/m³
│   ├── strategy_d_lut4x4.py            Strategy D: 4×4 Discrete Map        153.5 km/m³
│   ├── strategy_e_aecms.py             Strategy E: Adaptive ECMS           158.8 km/m³
│   ├── strategy_f_lpf.py               Strategy F: LPF-EMS                 148.2 km/m³
│   ├── strategy_g_pi.py                Strategy G: FF + SOC-PI (ADAPTIVE)  150.0 km/m³
│   │
│   ├── strategy_g_pulse_glide.py       Pulse-and-glide drive cycle generator
│   │                                   + Strategy G comparison (260.7 km/m³)
│   ├── test_strategy_g_robustness.py   Strategy G tested on WLTC & NEDC
│   │
│   ├── plot_fc_efficiency_map.py       FC efficiency map visualisation
│   ├── plot_flowcharts.py              System architecture flowcharts
│   └── visualize_lut_3d_v3.py         3D LUT surface plot (Strategy A)
│
├── matlab/                             ← Velocity profiles & drive-cycle data
│   ├── canonical_with_incline.csv      ★ PRIMARY input — GPS lap with road gradient,
│   │                                     used by all strategies via ems_core.py
│   ├── canonical_drive_cycle_35min.csv   Alternative flat profile
│   ├── canonical_drive_cycle_35min_1.csv Alternative flat profile v2
│   ├── race1_cleaned.csv               Raw GPS race data (cleaned)
│   ├── race2_cleaned.csv
│   ├── race4_cleaned.csv
│   ├── ecogenium_main_analysis.m       MATLAB: canonical drive-cycle plots
│   ├── ecogenium_comparative_analysis.m MATLAB: Race 1/2/4 comparison
│   ├── ecogenium_energy_analysis.m     MATLAB: energy breakdown
│   ├── RUN_ALL_ANALYSIS.m              MATLAB: master runner
│   └── run_power_plot.py               Python: vehicle power + motor efficiency map
│
├── matlab-archive/                     Old v1 MATLAB scripts — reference only
│
├── fc-model/                           FC system polarisation model
│   ├── FuelCellEstimate_v3.py          Current model (LC 52.30 + BOP + efficiency)
│   ├── FuelCellEstimate_v2.py          Intermediate version
│   └── FuelCellEstimate.py             Original prototype
│
├── datasheets/
│   ├── FC_Manual.pdf                   balticFuelCells LC 52.30 stack manual
│   └── BAFANG_RM_G060.1000_motor_datasheet.pdf
│
└── results/                            Generated figures — do not edit by hand
    ├── ems/
    │   ├── strategy_a_lut_results.png
    │   ├── strategy_b_fsm_results.png
    │   ├── strategy_c_ecms_results.png
    │   ├── strategy_d_lut4x4_results.png
    │   ├── strategy_e_aecms_results.png
    │   ├── strategy_f_lpf_results.png
    │   ├── strategy_g_pi_results.png         4-panel: P_fc, SOC, I_fc, H₂ vs time
    │   ├── strategy_g_robustness.png         WLTC / NEDC robustness test
    │   ├── strategy_g_pulse_glide.png        P&G vs GPS comparison (4-row figure)
    │   ├── fc_efficiency_map.png
    │   ├── flowchart_system_architecture.png
    │   └── flowchart_strategy_a_deep_dive.png
    ├── sem_velocity_profile_1lap.png
    ├── motor_efficiency_map.png
    └── velocity_profile_calibrated.png
```

---

## How it all connects — data flow

```
GPS raw laps                    MATLAB preprocessing
race1/2/4_cleaned.csv  ──────►  ecogenium_energy_analysis.m
                                        │
                                        ▼
                          matlab/canonical_with_incline.csv
                          (one clean lap at 5 Hz, speed + incline)
                                        │
                                        ▼
                            ems_core.build_demand_profile()
                            • reads canonical CSV
                            • resamples to DT = 0.2 s (5 Hz)
                            • computes P_elec each step
                              (aero drag + rolling + gradient + accel)
                            • scales to 1318.18 m/lap (14.5 km / 11)
                            → returns (t_arr [s], P_dem_arr [W])  1 lap
                                        │
                            ems_core.simulate_race(strategy_fn)
                            • tiles 11 laps
                            • calls strategy_fn(P_dem, SOC, P_fc_prev,
                                               t_in_lap, lap_idx)
                              every 0.2 s step
                            • enforces FC ramp (100 W/s), SC floor
                            → results dict: SOC, H₂, P_fc, P_sc, I_fc
                                        │
                            strategy_X_results.png (4-panel plot)
```

For **Strategy G** there are two extra scripts that run downstream of `strategy_g_pi.py`:

```
strategy_g_pi.py
    │
    ├──► test_strategy_g_robustness.py   tests on WLTC (×0.31) and NEDC (×0.33)
    │                                    scaled cycles without retuning
    │
    └──► strategy_g_pulse_glide.py       generates synthetic pulse-and-glide profile
                                         (lap-by-lap forward dynamics, brakes to v=0
                                          at each lap end), grid-sweeps 36 param
                                         combinations, runs Strategy G on best profile
```

---

## Running order

```bash
cd fcs-strategy/ems

# Individual strategies (fastest first)
python strategy_a_lut.py
python strategy_b_fsm.py
python strategy_c_ecms.py
python strategy_d_lut4x4.py
python strategy_e_aecms.py
python strategy_f_lpf.py
python strategy_g_pi.py                  # prints full cross-strategy comparison table

# Strategy G extras
python test_strategy_g_robustness.py
python strategy_g_pulse_glide.py         # slowest — 36-combination grid sweep
```

**Dependencies:** `numpy pandas matplotlib scipy`
```bash
pip install numpy pandas matplotlib scipy
```

---

## Drive-cycle details

| Property | Value |
|---|---|
| Source | Race 4 GPS master lap — cleaned, resampled, calibrated |
| Laps | 11 |
| Total distance | 14.5 km  (1318.18 m/lap) |
| Duration | ≈ 35 min  (2097 s) |
| DT (simulation step) | 0.2 s (5 Hz) |
| Peak speed | 32.6 km/h |
| Avg electrical demand | 259 W |
| Incline data | yes — from GPS altitude channel |

---

## Strategy G — algorithm summary

Strategy G is the only adaptive strategy: it requires no pre-tuned velocity profile and self-corrects in real time.

```
Every 0.2 s:
  P_filt  ← α × P_filt + (1−α) × P_dem          [LPF, τ = 15 s]
  err     ← SOC_ref − SOC
  integr  += err × DT   (anti-windup ± 75 W·s)
  P_fc    = P_filt + K_p × err + K_i × integr + P_lap_offset
  if P_dem < 25 W and SOC ≥ 0.55: P_fc = 0      [glide override]

At each lap boundary:
  deficit       = (0.60 − SOC) × 38 400 J
  P_lap_offset += 0.3 × deficit / T_LAP
  P_lap_offset  = clip(P_lap_offset, −200, +200 W)
```

Tuned values: **K_p = 1900 W/SOC**, K_i = 2.0 W/(SOC·s), τ = 15 s.

---

## Pulse-and-glide verification

Best parameters found by grid sweep: **v_low = 18 km/h, v_high = 30 km/h, P_pulse = 1000 W**

| Check | Result |
|---|---|
| Total distance | 14.496 km  (error 4 m) ✓ |
| Complete stops at lap ends | 12 stops detected ✓ |
| Min velocity | 0.00 km/h ✓ |
| Duration | 37.1 min |
| H₂ consumed | 4.999 g  →  **260.7 km/m³** |
| vs GPS constant-speed | 8.687 g  →  150.0 km/m³  (+74% improvement) |

Motor efficiency gain: **80.7%** mean (P&G, active-only) vs **65.2%** (constant-speed cruising).
