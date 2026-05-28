# Ecogenium — Hydraix I EMS Simulation
### Shell Eco-marathon 2026 · Energy Management System · SIM Package

> **Purpose of this folder:** Complete, self-contained simulation package for the
> Hydraix I fuel-cell/supercapacitor EMS. Hand this folder to any new developer
> or agent — all inputs, scripts, models, strategies, and results are here.

---

## Table of Contents

1. [Folder Structure](#1-folder-structure)
2. [File Descriptions](#2-file-descriptions)
3. [Simulation Flow — Block Diagram](#3-simulation-flow--block-diagram)
4. [Key Physical Constants & Parameters](#4-key-physical-constants--parameters)
5. [How to Run](#5-how-to-run)
6. [Strategy Descriptions](#6-strategy-descriptions)
7. [Results Summary](#7-results-summary)
8. [Result Plots Guide](#8-result-plots-guide)
9. [Files NOT in this Folder (Future Use)](#9-files-not-in-this-folder-future-use)
10. [Developer Notes for New Agents](#10-developer-notes-for-new-agents)

---

## 1. Folder Structure

```
SIM/
├── README.md                          ← you are here
├── run_test.py                        ← isolation test: confirms folder is self-contained ★
│
├── datasheets/                        ← physical hardware reference documents
│   ├── motor_lookup_table.xlsx        ← BAFANG 2D efficiency map (PRIMARY INPUT)
│   ├── sem_2025_eu.csv                ← SEM 2025 EU race route elevation profile
│   ├── HyCaps.pdf                     ← VINATech VEL13353R8257G LIC datasheet
│   ├── Hycaps.txt                     ← Team notes on HyCap configuration
│   ├── BAFANG_RM_G060.1000_motor_datasheet.pdf
│   └── FC_Manual.pdf                  ← Horizon LC52.30 fuel cell manual
│
├── scripts/                           ← runnable simulation scripts
│   ├── ems_core.py                    ← physics core (import this, don't run)
│   ├── combined_best_profile.py       ← MAIN simulation entry point ★
│   ├── grid_search_extended.py        ← VH/VL/PP optimisation grid search
│   └── profile_comparison.py          ← benchmark three driving profiles
│
├── models/                            ← standalone component models (importable independently)
│   ├── motor_model.py                 ← BAFANG 2D LUT: motor_lookup_2d(), motor_eta(), R_WHEEL ★
│   ├── supercap_model.py              ← SC bank model: sc_voltage(), sc_soc_update(), constants ★
│   └── FuelCellEstimate_v3.py         ← FC polarisation curve model (LC52.30)
│
├── strategies/                        ← standalone strategy reference scripts
│   ├── strategy_a_lut.py              ← Strategy A: 2D lookup table (old interface)
│   ├── strategy_b_fsm.py              ← Strategy B: finite-state machine
│   ├── strategy_c_ecms.py             ← Strategy C: ECMS
│   ├── strategy_d_lut4x4.py           ← Strategy D: 4×4 LUT
│   ├── strategy_e_aecms.py            ← Strategy E: Adaptive ECMS
│   ├── strategy_f_lpf.py              ← Strategy F: LPF feedforward only
│   └── strategy_g_pi.py               ← Strategy G: PI controller (earlier version)
│
├── data/                              ← input/output velocity profiles
│   ├── sem_combined_best.csv          ← CURRENT BEST profile (VH=9.0, VL=6.5, PP=700W)
│   ├── sem_2026_elev_aware.csv        ← MATLAB elevation-aware P&G reference
│   └── canonical_with_incline.csv     ← Real 2023/24 race telemetry (11 laps)
│
└── plots/                             ← all key result figures
    ├── combined_best_result.png       ← 6-strategy comparison ★ MAIN RESULT
    ├── grid_search_extended.png       ← VH/VL optimisation heatmap
    ├── profile_comparison.png         ← 3-profile benchmark
    ├── profile_velocity_comparison.png← velocity trace + speed histogram
    ├── FC_LC52_30_system_curves_v3.png← FC polarisation & efficiency curves
    └── strategy_g_slides.html         ← Reveal.js team presentation (open in browser)
```

---

## 2. File Descriptions

### datasheets/

| File | Description |
|------|-------------|
| `motor_lookup_table.xlsx` | **Primary motor input.** BAFANG RM G060.1000 hub motor, 2D grid: 36 speed points (5–40 km/h) × 35 torque points (1–35 Nm). Columns: `Speed_kmh`, `Torque_Nm`, `I_dc_A`, `V_eff_V`, `eta_v1_pct`. Used by `motor_lookup_2d()` in `ems_core.py`. |
| `sem_2025_eu.csv` | SEM 2025 European event GPS route. Columns: distance [m], elevation [m]. Used by `_load_route()` to compute per-step grade for every lap. Lap distance D_LAP = 1318 m (14.5 km / 11 laps). |
| `HyCaps.pdf` | VINATech VEL13353R8257G Lithium-Ion Capacitor (LIC) datasheet. Key specs: 3,800 F rated, 3.8 V max, **2.5 V minimum** (hard limit — below this damages the cell), DC ESR = 100 mΩ/cell. |
| `Hycaps.txt` | Internal team notes. Confirms 2026 configuration: **20P × 16S**. Notes from 2023/24 season (16S × 10P). Power test data: 59.7 V → 40 V @ 1 A → 14 Wh. |
| `BAFANG_RM_G060.1000_motor_datasheet.pdf` | Motor manufacturer datasheet. Peak torque, rated current, winding specs. |
| `FC_Manual.pdf` | Horizon LC52.30 fuel cell manual. Rated 52 W, polarisation curve, H₂ consumption map, operating limits. |

### scripts/

| File | Role | Run directly? |
|------|------|---------------|
| `ems_core.py` | Physics foundation. Defines all constants, loads motor LUT, implements `sc_voltage()`, `sc_terminal_voltage()`, `sc_soc_update()`, `fc_current()`, `fc_h2_rate()`, `motor_lookup_2d()`. Paths fixed for SIM folder: `MATLAB_DIR→../data/`, `RESULTS_DIR→../plots/`. **Import this, never run directly.** | No |
| `combined_best_profile.py` | **Main simulation.** Builds P&G velocity profiles via `build_profile()`, converts to motor electrical signals via `compute_motor_signals()`, runs all 6 EMS strategies via `simulate()`, finds best (VH, VL, PP) combination. Outputs `data/sem_combined_best.csv` and `plots/combined_best_result.png`. Wrapped in `if __name__ == '__main__':` so it can be imported as a module. | `python3 combined_best_profile.py` |
| `grid_search_extended.py` | Extends the P&G grid to VH ∈ [7–9.5 m/s], VL ∈ [5.5–8 m/s], PP ∈ [400–700 W]. Imports from `combined_best_profile.py`. Generates `plots/grid_search_extended.png` heatmap. | `python3 grid_search_extended.py` |
| `profile_comparison.py` | Benchmarks 3 driving profiles (Python P&G, MATLAB elev-aware, canonical telemetry) with identical Strategy G. Useful for A/B testing new profiles. | `python3 profile_comparison.py` |

### models/

| File | Description |
|------|-------------|
| `motor_model.py` | **Standalone 2D motor LUT.** Loads `datasheets/motor_lookup_table.xlsx` and exposes `motor_lookup_2d(speed_kmh, torque_nm) → (I_dc_A, V_eff_V, eta)`, `motor_eta(p_out_W)` (1D fallback), and `R_WHEEL = 0.295 m`. Import directly without needing `ems_core`. |
| `supercap_model.py` | **Standalone SC bank model.** Exports all SC constants (`SC_C`, `SC_V_MAX`, `SC_V_MIN`, `SC_ESR`, `SC_E_J`, `SC_SOC_0`, `SC_SOC_MIN`, `SC_SOC_MAX`) and three functions: `sc_voltage(soc)`, `sc_terminal_voltage(soc, I_A)`, `sc_soc_update(soc, P_W, dt)`. Import directly without needing `ems_core`. |
| `FuelCellEstimate_v3.py` | Electrochemical model of the Horizon LC52.30. Fits the polarisation curve (V = OCV − R_int × I − η_act) to measured data. Outputs `fc_voltage(I)`, `fc_power(I)`, `fc_h2_rate(I)`. The fitted coefficients are hardcoded into `ems_core.py` as the `_P_TAB` / `_I_TAB` inversion arrays. |

### strategies/

> **Note:** All 6 final strategies are implemented as `make_strat_*()` functions
> inside `combined_best_profile.py`. The files here are **standalone earlier versions**
> kept for reference, design history, and Simulink porting.

| File | Strategy | Interface | Status |
|------|----------|-----------|--------|
| `strategy_a_lut.py` | 2D heuristic lookup table. Axes: (P_dem/SC_E_J, SOC). Bilinear interpolation. | Old: `fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx)` | Reference |
| `strategy_b_fsm.py` | Finite-state machine: CHARGE / SUSTAIN / DISCHARGE modes based on SOC thresholds. | Old | Reference |
| `strategy_c_ecms.py` | Equivalent Consumption Minimisation Strategy. Minimises H₂ + equivalence_factor × SC_energy. | Old | Reference |
| `strategy_d_lut4x4.py` | 4×4 lookup table indexed by (SOC band, power band). | Old | Reference |
| `strategy_e_aecms.py` | Adaptive ECMS — updates equivalence factor online based on SOC drift. | Old | Reference |
| `strategy_f_lpf.py` | LPF feedforward only (no PI). FC tracks low-pass-filtered motor power. | Old | Reference |
| `strategy_g_pi.py` | Earlier Strategy G without the 150 W floor. Produces ~202 km/m³. | Old | Reference |

### data/

| File | Description |
|------|-------------|
| `sem_combined_best.csv` | **Current best P&G velocity profile.** Generated by `combined_best_profile.py` with VH=9.0 m/s, VL=6.5 m/s, PP=700 W. Columns: `time_s`, `velocity_ms`, `velocity_kmh`, `dist_in_lap_m`, `lap_num`, `elevation_m`, `grade`, `P_elec_W`. 11 laps, ~2100 s. |
| `sem_2026_elev_aware.csv` | Alternative profile generated by MATLAB agent. Near-constant velocity (mean 7.1 m/s, std 1.76 m/s). Achieved **224.6 km/m³** in benchmark — better than Python P&G but uses lower average speed which would fail the 35-min constraint in the current `verify()` check. Use for studying low-speed constant-velocity behaviour. |
| `canonical_with_incline.csv` | Real 2023/24 race telemetry. 10,148 rows, ~2100 s. Columns: `elapsed_sec`, `MotorRPM`, `Speed`, `Speed_kmh`, `Distance_km`, `inc_angle_deg`. High noise — do not use raw velocity gradient directly (produces spurious acceleration spikes → unrealistic H₂ demand). Smooth before use. |

### plots/

| File | What it shows |
|------|---------------|
| `combined_best_result.png` | **Main result.** 6 panels: velocity profile, SC SOC, FC power, SC power, cumulative H₂, strategy comparison table. Best profile: VH=9.0, VL=6.5, PP=700 W. |
| `grid_search_extended.png` | Heatmap of km/m³ over all (VH, VL) combinations at best PP. Valid cells (charge-sustaining) shown green. Red star = previous best; gold star = new best. |
| `profile_comparison.png` | 3×3 subplot comparing Python P&G, MATLAB elev-aware, and real telemetry profiles side-by-side (velocity, SOC, FC power). |
| `profile_velocity_comparison.png` | Lap 1 velocity traces + speed distribution histogram: Python P&G is bimodal (aggressive), MATLAB profile is unimodal (near-constant). |
| `FC_LC52_30_system_curves_v3.png` | FC stack voltage, power, and H₂ rate vs current. Shows peak electrical efficiency at ~120 W. |
| `strategy_g_slides.html` | Reveal.js 5 presentation (9 slides). Open in any browser. Covers inputs, block diagram, correction terms, energy balance, strategy comparison, and Simulink mapping. |

---

## 3. Simulation Flow — Block Diagram

```
╔══════════════════════════════════════════════════════════════════════╗
║                     PHYSICAL HARDWARE INPUTS                         ║
╠══════════════════════════════════════════════════════════════════════╣
║  datasheets/motor_lookup_table.xlsx      datasheets/sem_2025_eu.csv  ║
║  BAFANG RM G060.1000                     SEM 2025 EU route           ║
║  36×35 grid: Speed × Torque              GPS elevation every 5 m     ║
║  → I_dc_A, V_eff_V, eta_v1_pct          → elev(s), grade(s)         ║
╚══════════╤═══════════════════════════════════════╤═══════════════════╝
           │                                       │
           ▼                                       ▼
┌──────────────────────┐             ┌─────────────────────────────┐
│  ems_core.py          │             │  combined_best_profile.py   │
│  _load_motor_lut()    │             │  _load_route()              │
│  RegularGridInterp    │             │  D_LAP = 1318 m             │
│  motor_lookup_2d()    │◄────────────│  _elev_fn(s), _grade_fn(s)  │
└──────────┬────────────┘             └────────────┬────────────────┘
           │                                       │
           │                          ┌────────────▼────────────────┐
           │                          │  build_profile(VH, VL,      │
           │                          │               V_DH, PP)     │
           │                          │                             │
           │                          │  Finite-State Machine:      │
           │                          │  PULSE (motor on at PP W)   │
           │                          │  GLIDE (motor off, coast)   │
           │                          │  COAST_TO_STOP (end of lap) │
           │                          │  STOP (3 steps × 0.2 s)    │
           │                          │                             │
           │                          │  Outputs per time step:     │
           │                          │  va [m/s], ga [grade]       │
           │                          │  la [lap#], ca [coast flag] │
           │                          └────────────┬────────────────┘
           │                                       │
           └──────────────────┬────────────────────┘
                              ▼
              ┌───────────────────────────────┐
              │  compute_motor_signals(va, ga) │
              │                               │
              │  accel = ∇va / dt             │
              │  P_wheel = CRR·M·g·v          │
              │           + ½·Cd·Af·ρ·v³     │
              │           + M·a·v            │
              │           + M·g·grade·v      │
              │                               │
              │  Torque = P_wheel⁺/v · R_wheel│
              │  [Speed, Torque] → LUT        │
              │  P_elec = I_dc × V_eff  [W]   │
              └───────────────┬───────────────┘
                              │
              ┌───────────────▼───────────────┐
              │         simulate()            │
              │                               │
              │  For each time step k:        │
              │                               │
              │  1. U_oc = sc_voltage(SOC)    │  ← OCV from energy map
              │  2. I_m = P_elec / U_oc       │  ← first estimate
              │  3. U_sc = U_oc - I_m·ESR    │  ← terminal voltage
              │  4. I_m = P_elec / U_sc       │  ← corrected current
              │                               │
              │  5. P_fc = Strategy_fn(       │
              │       I_motor, U_sc,          │
              │       P_fc_prev,              │
              │       t_in_lap, lap_idx)      │
              │                               │
              │  6. P_sc = P_elec - P_fc      │  ← SC fills the gap
              │  7. I_sc = P_sc / U_sc        │
              │  8. SOC += I_sc·dt / (C·U_sc) │  ← charge balance
              │                               │
              │  9. I_fc = fc_current(P_fc)   │
              │  10. ΔH₂ = K_H2·I_fc·dt      │  ← Faraday's law
              └───────────────┬───────────────┘
                              │
              ┌───────────────▼───────────────┐
              │  OUTPUT METRICS               │
              │                               │
              │  m_H2_total  [g]              │
              │  distance    [km]  = 14.5 km  │
              │  km/m³ = 14.5 / (H2/H2_ρ)   │
              │  dSOC  = SOC_final - SOC_0    │
              │  Charge-sustaining: |dSOC|    │
              │                    ≤ 0.015    │
              └───────────────────────────────┘
```

### Strategy G Internal Flow (make_strat_g)

```
  INPUTS: I_motor [A], U_sc [V], P_fc_prev [W], t_in_lap [s], lap_idx

  Derived:
    P_motor = I_motor × U_sc            (actual bus power drawn)
    SOC = (U_sc² − V_min²)             (energy-based SOC from terminal V)
          ─────────────────
          (V_max² − V_min²)

  ┌─ Motor off? (I_motor < 0.1 A) ──► return G_FC_MIN = 150 W
  │
  ├─ LPF feedforward:
  │   lpf = α·lpf + (1−α)·P_motor      α = exp(−dt/τ),  τ = 15 s
  │
  ├─ Lap-offset correction (at each lap boundary):
  │   offset += 0.3 × (SOC₀ − SOC) × SC_E_J / 186
  │   offset = clip(offset, −400, +400)
  │
  ├─ Proportional term:   K_p × (SOC₀ − SOC)
  │
  ├─ Integral term:       K_i × ∫(SOC₀ − SOC)dt
  │   integral clamped to ±400/K_i  (allows ±400 W correction)
  │
  └─ P_fc = clip(lpf + offset + K_p·ΔSOC + K_i·∫ΔSOC,
                 G_FC_MIN=150 W,  FC_P_MAX)
```

---

## 4. Key Physical Constants & Parameters

### Vehicle (combined_best_profile.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MASS` | 170 kg | Vehicle + driver mass |
| `CRR` | 0.003 | Rolling resistance coefficient |
| `CD` | 0.25 | Drag coefficient |
| `AF` | 0.50 m² | Frontal area |
| `RHO` | 1.225 kg/m³ | Air density |
| `G` | 9.81 m/s² | Gravitational acceleration |
| `R_WHEEL` | 0.295 m | Wheel radius (verified: P = T·v/R) |
| `ETA_DT` | 0.97 | Drivetrain efficiency (chain + bearings) |

### Race Constraint (combined_best_profile.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `N_LAPS` | 11 | Number of laps |
| `D_LAP` | ~1318 m | Distance per lap (14.5 km / 11) |
| `TOTAL_KM` | 14.5 km | Total race distance |
| `DT` | 0.2 s | Simulation time step |
| `RESAMPLE_HZ` | 5 Hz | Profile sampling frequency |
| `N_STOP_STEPS` | 15 | Stop duration = 15 × 0.2 = 3 s |
| Max duration | 35.5 min | `verify()` hard upper bound |
| Min duration | 28.0 min | `verify()` hard lower bound |

### Supercapacitor Bank (ems_core.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Cell | VINATech VEL13353R8257G | Lithium-Ion Capacitor |
| Configuration | **20P × 16S** | 2026 season (20 parallel, 16 series) |
| `SC_C` | 312.5 F | Bank capacitance (250 F/cell × 20P / 16S ... wait: C_bank = C_cell × 20 / 1 = 5000F per group of 16S... actually C_cell=250F, 20P → C_par=5000F, 16S → C_bank = 5000/16 = 312.5 F) |
| `SC_V_MAX` | 60.8 V | 16 × 3.8 V |
| `SC_V_MIN` | 40.0 V | 16 × 2.5 V |
| `SC_E_J` | ~327,600 J | ½·C·(V_max²−V_min²) = **91.0 Wh** |
| `SC_ESR` | 0.080 Ω | 100 mΩ/cell × 16S / 20P |
| `SC_SOC_0` | 0.60 | Initial SOC (60%) |
| `sc_voltage(SOC)` | `√(V_min² + SOC·(V_max²−V_min²))` | OCV from energy-based SOC |

### Fuel Cell (ems_core.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Model | Horizon LC52.30 | 52 W rated PEM stack |
| `FC_P_MIN` | 100 W | Minimum stable FC output |
| `FC_P_MAX` | 620 W | Maximum FC output |
| `FC_RAMP` | 50 W/s | Max FC power ramp rate |
| `LHV_H2` | 33.3 Wh/g | Lower heating value of H₂ |
| `K_H2` | ~3.7×10⁻⁴ g/(A·s) | H₂ mass per coulomb (Faraday) |
| `H2_DENSITY` | 0.0899 kg/m³ | H₂ density at STP |
| Peak efficiency | ~59% | At ~120 W electrical output |

### Best P&G Profile Parameters

| Parameter | Current Best | Description |
|-----------|-------------|-------------|
| `VH` | 9.0 m/s (32.4 km/h) | Pulse target speed |
| `VL` | 6.5 m/s (23.4 km/h) | Glide lower bound |
| `PP` | 700 W | Motor pulse power |
| `V_DH` | 9.0 m/s | Downhill speed cap |
| Cycle structure | ~65s pulse + ~95s glide + ~75s coast + 3s stop | Per lap |

---

## 5. How to Run

### Prerequisites

```bash
pip install numpy scipy pandas matplotlib openpyxl
```

### Verify the folder is self-contained (run this first)

```bash
cd SIM
python3 run_test.py
```

This script:
- Verifies all imports resolve inside SIM/ with zero external dependencies
- Spot-checks motor LUT and SC model outputs
- Builds the best P&G profile and runs 5 strategies (B/C/D/A/G)
- Prints a results table and consistency checks

Expected output (all PASS, all charge-sustaining):
```
  Strategy           km/m³    H2[g]      dSOC   CS?
  G  LPF+PI+150W     211.4    6.164   +0.0078   YES ★
  D  Const-FC        210.1    6.203   -0.0119   YES
  A  2D-LUT          206.7    6.306   +0.0062   YES
  C  Strat-H         206.2    6.320   +0.0079   YES
  B  Rule-FSM        205.1    6.354   +0.0138   YES
```

### Run the main simulation

```bash
cd SIM/scripts
python3 combined_best_profile.py
```

Outputs:
- `../data/sem_combined_best.csv` — best velocity profile
- `../plots/combined_best_result.png` — 6-strategy comparison figure
- Console: km/m³, H₂ [g], dSOC for all strategies

### Run the extended grid search

```bash
cd SIM/scripts
python3 grid_search_extended.py
```

Outputs:
- `../plots/grid_search_extended.png` — km/m³ heatmap over (VH, VL)
- Console: full results table sorted by km/m³

### Run the profile comparison

```bash
cd SIM/scripts
python3 profile_comparison.py
```

Compares 3 profiles with Strategy G. Outputs `../plots/profile_comparison.png`.

### Use the standalone model files

```python
# From any script inside SIM/ — no ems_core needed
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'models'))
from motor_model import motor_lookup_2d, R_WHEEL
from supercap_model import sc_voltage, sc_terminal_voltage, SC_E_J, SC_SOC_0
```

### Important path note

All scripts inside `SIM/scripts/` use paths relative to their own location:
- `../datasheets/` → `SIM/datasheets/` ✓
- `../data/`       → `SIM/data/`       ✓  (fixed from original `../matlab/`)
- `../plots/`      → `SIM/plots/`      ✓  (fixed from original `../results/ems/`)

When writing new scripts, always start with:
```python
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
```

---

## 6. Strategy Descriptions

All 6 strategies are implemented in `scripts/combined_best_profile.py` as
`make_strat_*()` factory functions. Each returns `fn(I_motor, U_sc, P_fc_prev,
t_in_lap, lap_idx) → P_fc [W]`.

| Strategy | Function | Description | Best km/m³ |
|----------|----------|-------------|-----------|
| **G (winner)** | `make_strat_g(K_p, K_i, tau)` | LPF feedforward + SOC-PI + lap-offset + **150 W floor always-on** | **211.4** |
| H | `make_strat_h(K_p)` | Minimise H₂ subject to SOC band (rule-based + PI) | ~202.7 |
| Rule-based | `make_strat_rule(P_hi)` | Two-level: P_hi when SOC low, P_lo when SOC high | ~202.2 |
| A (2D LUT) | `make_strat_a(K_soc, P_base)` | Bilinear interpolation on (Δpower/E_sc, SOC) table | ~201.8 |
| Constant FC | `make_strat_constant(P_set)` | Single setpoint, no feedback | ~201.7 |
| PI only | `make_strat_pi(K_p)` | Pure proportional-integral, no feedforward | ~215.7 (not charge-sustaining) |

### Strategy G — Why it wins

1. **LPF feedforward** (`tau=15 s`): FC tracks the low-pass-filtered motor power
   demand, so it anticipates load changes rather than reacting to instantaneous spikes.

2. **150 W floor**: FC never drops below 150 W — not even during glide or coast-to-stop.
   This pre-charges the SC during glide so the next pulse draws less from the SC,
   keeping the FC closer to its 200–400 W operating band.

3. **PI integral with ±400 W headroom**: The integral clamp `±400/K_i` allows
   enough corrective authority to overcome the 150 W floor energy excess (~93 kJ/race).

4. **Lap-offset correction**: At each lap boundary, the offset is nudged by
   `0.3 × ΔSOC × E_sc / 186` — a slow feed-forward that drifts the operating
   point toward charge sustenance over multiple laps.

---

## 7. Results Summary

### Final Rankings (VH=9.0, VL=6.5, PP=700 W, 11 laps, 14.5 km)

| Rank | Strategy | km/m³ | H₂ [g] | dSOC | CS? |
|------|----------|-------|--------|------|-----|
| 1 | **Strategy G (150 W floor)** | **211.4** | 6.18 | +0.007 | ✓ |
| 2 | Strategy H | ~202.7 | ~6.43 | +0.002 | ✓ |
| 3 | Rule-Based | ~202.2 | ~6.45 | +0.005 | ✓ |
| 4 | Strategy A (2D LUT) | ~201.8 | ~6.46 | +0.007 | ✓ |
| 5 | Constant FC | ~201.7 | ~6.46 | +0.008 | ✓ |
| — | PI only | ~215.7 | ~6.04 | −0.074 | ✗ |

CS = Charge-Sustaining (`|dSOC| ≤ 0.015`)

### P&G Grid Search Best (Strategy G, charge-sustaining)

| Rank | VH [m/s] | VL [m/s] | PP [W] | km/m³ |
|------|----------|----------|--------|-------|
| 1 | 9.0 | **6.5** | 700 | **211.4** |
| 2 | 9.5 | 6.5 | 600 | 211.2 |
| 3 | 9.5 | 6.5 | 700 | 211.2 |
| 4 | 9.0 | 7.0 | 700 | 210.5 |

Note: Profiles with VH < 8.75 m/s fail the 35-min constraint (can't complete
14.5 km in time including coast-to-stop phases).

### Profile Benchmark (Strategy G identical)

| Profile | km/m³ | H₂ [g] | Motor η | Notes |
|---------|-------|--------|---------|-------|
| MATLAB elevation-aware | 224.6 | 5.81 | 48.0% | Near-constant ~7.1 m/s — fails 35-min if used in main sim |
| Python P&G | ~206.8 | 6.30 | 51.6% | Valid, charge-sustaining |
| Canonical 2023/24 telemetry | 88.8 | 14.7 | 48.6% | Raw noise → unusable directly |

---

## 8. Result Plots Guide

### combined_best_result.png

6 rows × 1 column:
- Row 1: Velocity [m/s] over time — P&G pattern clearly visible (9→6.5 m/s cycles)
- Row 2: SC SOC — should stay near 0.60, drift < 0.015 over 35 min
- Row 3: FC power [W] — Strategy G LPF trace with 150 W floor
- Row 4: SC power [W] — peaks during motor pulses, negative during pre-charge
- Row 5: Cumulative H₂ [g] — linear growth; final value → km/m³
- Row 6: Strategy comparison bar chart

### grid_search_extended.png

2D heatmap:
- X-axis: VH (pulse speed), Y-axis: VL (glide lower bound)
- Colour: km/m³ (green = better, red = worse)
- Grey cells: profile failed `verify()` — can't complete 14.5 km in 35 min
- Numbers in each cell: km/m³ value + best PP

### strategy_g_slides.html

9-slide Reveal.js presentation. Navigate with arrow keys or spacebar:
1. Title + stats
2. P&G motivation
3. Signal derivation (I_motor, U_sc from 2D LUT)
4. SVG block diagram (full signal flow)
5. Three correction terms explained
6. 150 W floor energy balance
7. 6-strategy comparison table
8. Simulink implementation mapping
9. Lessons learned

---

## 9. Files NOT in this Folder (Future Use)

These files exist in the broader repo (`fcs-strategy/`) but are not copied here.
They may be useful for future development:

| File / Path | What it is | Why useful |
|-------------|-----------|------------|
| `ems/agent1_speed_window_sweep.py` | Sweeps P&G speed window width | Basis for the current grid search |
| `ems/agent2_power_sweep.py` | Sweeps pulse power PP | PP sensitivity analysis |
| `ems/agent3_terrain_adaptive.py` | Grade-aware P&G switching | Could improve on fixed VH/VL on the SEM track |
| `ems/compare_motor_efficiency.py` | Motor LUT visualisation | Useful for explaining motor operating region to hardware team |
| `ems/visualize_lut_3d_v3.py` | 3D visualisation of Strategy A LUT surface | Useful for presentations |
| `ems/test_strategy_g_robustness.py` | Strategy G Monte-Carlo robustness test | Test sensitivity to parameter variation before ECU deployment |
| `ems/plot_fc_efficiency_map.py` | FC efficiency vs operating point | Useful for FC thermal management |
| `fc-model/FuelCellEstimate.py` | FC model v1 | Historical — use v3 |
| `fc-model/FuelCellEstimate_v2.py` | FC model v2 | Historical — use v3 |
| `matlab/canonical_with_incline.csv` | Real 2023/24 telemetry with grade | Needs Savitzky-Golay smoothing before use in simulation |
| `matlab/sem_agent1/2/3_best.csv` | MATLAB-generated P&G profiles | Could be used as alternative driving cycles |
| `matlab/ecogenium_main_analysis.m` | MATLAB telemetry analysis | Original analysis pipeline from 2024 |
| `datasheets/polandsemmap.pdf` | Poland SEM track map | Physical track reference |

### Future development hints

- **Simulink integration**: Strategy G's `fn(I_motor, U_sc, P_fc_prev, t_in_lap,
  lap_idx)` is a direct port to a Simulink function block. `I_motor` and `U_sc`
  are directly measurable on the car. No preprocessing required.

- **Terrain-adaptive P&G**: `agent3_terrain_adaptive.py` adjusts VH/VL per lap
  segment based on grade. On the SEM 2025 EU track (which has an ~0.3% grade),
  this could recover 3–5 km/m³.

- **Lower-speed profile**: The MATLAB elevation-aware profile (224.6 km/m³) beats
  P&G by ~13 km/m³ but fails the 35-min time constraint. If the race allows a
  longer time window, this near-constant-velocity approach should be revisited.

- **SC pack sizing**: Current 20P × 16S gives 91 Wh usable. Adding 4 more parallel
  strings (24P) would reduce ESR to 67 mΩ, reduce voltage sag, and allow deeper
  discharge — potentially worth 5–8 km/m³ if SC was the constraint.

---

## 10. Developer Notes for New Agents

> Read this section if you are an AI agent starting a new session on this codebase.

### The critical call chain

```python
# 1. Build velocity profile (one call per grid point)
ta, va, Pa, sa, la, ea, ga, ca = build_profile(VH, VL, V_DH, PP, P_BO)

# 2. Convert to electrical motor signals (vectorised, fast)
P_elec, V_eff, Torque_Nm = compute_motor_signals(va, ga)

# 3. Run EMS strategy simulation
result = simulate(make_strat_g(K_p, K_i=2.0), P_elec, la, ca)

# 4. Compute km/m³
km3 = TOTAL_KM / (result['m_H2'] / H2_DENSITY)
```

### Array conventions

| Array | Shape | Units | Description |
|-------|-------|-------|-------------|
| `va` | (N,) | m/s | Velocity at each time step |
| `ga` | (N,) | — | Grade = sin(θ) at each step (from route CSV) |
| `la` | (N,) | int | Lap number 1–11 |
| `ca` | (N,) | bool | True = coast-to-stop or stop step |
| `P_elec` | (N,) | W | Electrical bus power from motor |
| `SOC` | (N,) | 0–1 | SC state of charge |

### Strategy function signature

```python
def fn(I_motor: float,   # DC current drawn from SC bus [A]
       U_sc: float,      # SC terminal voltage under load [V]
       P_fc_prev: float, # FC power at previous time step [W]
       t_in_lap: float,  # elapsed time within current lap [s]
       lap_idx: int,     # current lap index 0–10
       ) -> float:       # FC power setpoint [W]
```

### Charge sustenance check

```python
abs(result['dSOC']) <= 0.015   # |SOC_final - SOC_initial| < 1.5%
```

A strategy that is NOT charge-sustaining is invalid for competition — it either
runs out of SC charge or wastes energy over-charging.

### Key gotcha: coast-to-stop vs glide

`ca[k] = True` flags **coast-to-stop and hard-brake steps only** — NOT the glide
phase. During glide (`ca=False`, `I_motor ≈ 0`), `simulate()` enforces
`fc_min = G_FC_MIN = 150 W` (the floor). During coast-to-stop (`ca=True`),
`fc_min = 0` (FC is allowed to fully stop).

### ESR voltage sag (important for power balance)

```python
U_sc = sc_voltage(SOC) - I_motor * SC_ESR
# At typical I_motor = 4.6 A: sag = 4.6 × 0.080 = 0.37 V on a 50 V bus (0.7%)
# Simulation uses 2-step iterative correction for accuracy
```

### H₂ consumption formula

```python
I_fc = fc_current(P_fc)         # A, from polarisation curve
dH2  = K_H2 * I_fc * DT        # g per time step
# K_H2 ≈ 3.7×10⁻⁴ g/(A·s) — derived from Faraday's law for H₂
```

---

*Last updated: SEM 2026 pre-season simulation campaign.*
*Branch: `claude/fcs-strategy-folder-VHFN1` · Repo: `vallabh2504/ecogenium`*
