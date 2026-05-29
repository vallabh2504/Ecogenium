# Ecogenium — Hydraix I EMS Simulation
### Shell Eco-marathon 2026 · Energy Management System · SIM Package

> **Purpose of this folder:** Complete, self-contained simulation package for the
> Hydraix I fuel-cell/supercapacitor EMS. Hand this folder to any new developer
> or agent — all inputs, scripts, models, strategies, and results are here.

> **Model corrections (latest revision):**
> 1. **FC identity fixed** — the stack is a **balticFuelCells LC 52.30** (52 cells × 30 cm², ~1040 W gross), *not* a "Horizon 52 W" unit (confirmed against `FC_Manual.pdf`).
> 2. **FC→bus DC-DC converter loss** added (`ETA_DCDC = 0.95`); `P_fc` is FC net output, bus receives `P_fc × 0.95`.
> 3. **Charge-sustaining normalisation** — strategies are ranked on dSOC=0-normalised H₂ (`km_per_m3()` / `m_H2_norm`), removing tolerance-band artefacts.
> 4. **Motor iron-loss variant** is `v1` (optimistic); the main sim now prints a v1/v2/v3 km/m³ sensitivity band (≈ 200 / 164 / 139).
> 5. **SC efficiency** `SC_ETA=0.97` is now applied as √0.97 per direction (true 3 % round-trip).
> 6. Vehicle constants in this README corrected to match the code; 1D `motor_eta` deprecated.
>
> **Vehicle update (team-confirmed) + EMS recs #2/#3:**
> 7. **MASS 180 kg, AF 1.35 m²** (CdA ≈ 0.2025 m² — a relatively high-drag body).
>    Average bus demand ≈ 219 W; FC-net average ≈ 230 W.
> 8. **Strategy-G floor set to FC_P_MIN (100 W)** — membrane-safe and robust
>    across drag regimes (never over-charges the SC).
> 9. **#3 FC η-band cap implemented (`FC_ETA_BAND_HI` ≈ 385 W) but OFF by
>    default.** It caps only the feedforward baseline; the SOC-PI can still exceed
>    it. For this duty cycle the average demand is already inside the η-band, so
>    capping merely routes peak energy through the SC (≈3 % loss) with **no net
>    gain** (cap ON = 175.5 km/m³ but not charge-sustaining; cap OFF = 174.6,
>    CS). Useful only for genuinely low-demand vehicles/tracks — left as opt-in.
> 10. **#2 terrain-adaptive P&G** (`k_grade`) added and swept. On the near-flat
>    SEM 2025 track it gives **no gain** (k_grade=60 → 170.7 vs flat 174.6) —
>    the track is too flat (~0.3 % grade). Kept for hillier circuits.
> 11. Lap time is dominated by the forced coast-to-stop + glide structure, so the
>    feasible charge-sustaining band is **VH=9.0, VL=6.5, PP=800 W** (~35 min).
>
> **Current headline: Strategy G ≈ 174.6 km/m³** (normalised, converter loss,
> MASS=180 kg, AF=1.35 m²). Lower than the AF=0.8 figure (~205) because of the
> larger frontal area. The motor iron-loss assumption (v1) still dominates:
> v1=174.6, v2=144.7, v3=123.8 km/m³.

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
│   └── FC_Manual.pdf                  ← balticFuelCells LC 52.30 fuel cell manual (~1040 W)
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
| `FC_Manual.pdf` | **balticFuelCells GmbH FC Stack LC 52.30** user manual (Issue 05/2024). ⚠️ "52.30" denotes **52 cells × 30 cm² active area — NOT 52 W**. Extended-system nameplate ≈ **1040 W gross / ~1016 W net @ 37.5 A**; peak system η ≈ **59 % @ 120 W**. Polarisation curve, H₂ map, operating limits. (Earlier docs mislabelled this as a "Horizon 52 W" stack — incorrect.) |

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
| `FuelCellEstimate_v3.py` | Electrochemical model of the **balticFuelCells LC 52.30** (Chamberlin–Kim polarisation, 52 cells, 30 cm², calibrated to the 1040 W gross nameplate). Outputs stack voltage/power, BOP, and `fc_h2_rate(I)`. The inverted P_net→I curve is hardcoded into `ems_core.py` as the `_P_TAB` / `_I_TAB` arrays. Verified: P_gross@37.5A=1044 W, V@40A=27.1 V, 19.0 SLPM, peak η_sys 59 % @ 120 W. |
| `motor_model.py` (note) | Uses the **`v1` iron-loss variant** of the LUT (lowest iron loss → most optimistic efficiency, peak ~74 %). `v2`/`v3` are higher-loss models; the main sim reports a v1/v2/v3 km/m³ sensitivity band. The 1D `motor_eta()` (peak ~83 %) is **deprecated/legacy** and disagrees with the 2D LUT — do not use it for new work. |

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
              │  6. P_sc = P_elec            │  ← SC fills the gap
              │       − P_fc·η_dcdc           │    (η_dcdc=0.95 boost loss)
              │  7. I_sc = P_sc / U_sc        │
              │  8. SOC += I_sc·dt / (C·U_sc) │  ← charge balance (√η/dir)
              │                               │
              │  9. I_fc = fc_current(P_fc)   │  ← P_fc = FC net (stack side)
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

> ⚠️ Values below are the **actual code constants** in `combined_best_profile.py`
> (and `profile_comparison.py`). A previous version of this table listed stale
> values (170 kg, CRR 0.003, Cd 0.25, Af 0.50, η_dt 0.97) — corrected here.

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MASS` | 180 kg | Vehicle + driver mass (team-confirmed) |
| `CRR` | 0.006 | Rolling resistance coefficient |
| `CD` | 0.15 | Drag coefficient |
| `AF` | 1.35 m² | Frontal area (team-confirmed; CdA = 0.2025 m²) |
| `RHO` | 1.225 kg/m³ | Air density |
| `G` | 9.81 m/s² | Gravitational acceleration |
| `R_WHEEL` | 0.295 m | Wheel radius (verified: P = T·v/R) |
| `ETA_DT` | 0.95 | Drivetrain efficiency (chain + bearings) |

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
| `SC_ETA` | 0.97 | **Round-trip** efficiency — applied as √0.97 ≈ 0.985 per direction in `sc_soc_update()`, so a full charge→discharge cycle loses exactly 3 % |
| `SC_SOC_0` | 0.60 | Initial SOC (60%) |
| `sc_voltage(SOC)` | `√(V_min² + SOC·(V_max²−V_min²))` | OCV from energy-based SOC |

### Power-electronics losses (ems_core.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `ETA_DCDC` | 0.95 | **FC → bus boost-converter efficiency.** FC stack runs ~27–46 V, SC bus 40–60.8 V, so a boost stage is required. `P_fc` is the FC net output (sets current/H₂); the bus receives `P_fc × ETA_DCDC`. |

### Fuel Cell (ems_core.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Model | **balticFuelCells LC 52.30** | 52-cell × 30 cm² PEM stack (~1040 W gross), **not** a 52 W unit |
| `FC_P_MIN` | 100 W | Minimum stable FC net output (membrane idle floor) |
| `FC_P_MAX` | 1013 W | Maximum FC net output (≈37.5 A) |
| `FC_RAMP` | 100 W/s | Max FC power ramp rate |
| `LHV_H2` | 33.3 Wh/g (120 kJ/g) | Lower heating value of H₂ |
| `K_H2` | 5.43×10⁻⁴ g/(A·s) | H₂ mass per coulomb = N·M_H2/(2F), N=52 (Faraday) |
| `H2_DENSITY` | 0.0899 kg/m³ (89.88 g/m³) | H₂ density at STP |
| Peak efficiency | ~59% | At ~120 W net output |

### Best P&G Profile Parameters

> For the high-drag (CdA≈0.20) vehicle. VH<9.0 m/s overruns the 35.5-min cap
> (coast-to-stop + glide structure dominates lap time); higher PP is needed to
> reach VH against the larger drag.

| Parameter | Current Best | Description |
|-----------|-------------|-------------|
| `VH` | 9.0 m/s (32.4 km/h) | Pulse target speed |
| `VL` | 6.5 m/s (23.4 km/h) | Glide lower bound |
| `PP` | 800 W | Motor pulse power |
| `V_DH` | 10.0 m/s | Downhill speed cap |
| `k_grade` | 0 (flat best) | Terrain-adaptive gain — no gain on this near-flat track |
| Race time | ~35.0 min | Within the 28–35.5 min window |

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

Expected output (km/m³ is **charge-sustaining-normalised**, includes the FC→bus
converter loss η=0.95; vehicle MASS=180 kg, AF=1.35 m²; VH=9.0/VL=6.5/PP=700):
```
  Strategy           km/m³    H2[g]      dSOC   CS?
  G  LPF+PI+floor    172.0    7.648   +0.0133   YES ★
  B  Rule-FSM        170.5    7.563   -0.0148   YES
  D  Const-FC        170.5    7.581   -0.0116   YES
  A  2D-LUT          170.5    7.563   -0.0150   YES
  C  Strat-H         170.3    7.609   -0.0077   YES
```
> Note: Strategy G wins by ~1.5 km/m³ via its LPF feedforward + SOC-PI. The
> high-drag body keeps average demand (~230 W net) inside the FC η-band, so all
> reasonable strategies cluster within ~2 km/m³. `run_test` uses PP=700 W; the
> grid-search best is PP=800 W → 174.6 km/m³.

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

> km/m³ below are **charge-sustaining-normalised** (dSOC=0) and include the
> FC→bus converter loss (η=0.95). Vehicle (180 kg, AF 1.35 m²), grid point
> VH=9.0, VL=6.5, PP=800 W.

| Strategy | Function | Description | Best km/m³ |
|----------|----------|-------------|-----------|
| **G (winner)** | `make_strat_g(K_p, K_i, tau, p_eta_cap)` | LPF feedforward + SOC-PI + lap-offset + always-on FC floor (FC_P_MIN); optional η-band cap (#3, off) | **174.6** |
| A (2D LUT) | `make_strat_a(K_soc, P_base)` | Bilinear interpolation on (Δpower/E_sc, SOC) table | ~172.9 |
| Constant FC | `make_strat_constant(P_set)` | Single setpoint, no feedback | ~172.5 |
| Rule-based | `make_strat_rule(P_hi)` | Two-level: P_hi when SOC low, P_lo when SOC high | ~172.5 |
| H | `make_strat_h(P_set)` | Large-SC optimal constant dispatch + soft SOC term | ~172.4 |
| PI only | `make_strat_pi(K_p)` | Pure proportional-integral, no feedforward | ~172.8 (not charge-sustaining) |

### Strategy G — Why it wins

1. **LPF feedforward** (`tau=15 s`): FC tracks the low-pass-filtered motor power
   demand, so it anticipates load changes rather than reacting to instantaneous spikes.

2. **Always-on FC floor (FC_P_MIN = 100 W)**: FC never fully stops during glide.
   This pre-charges the SC so the next pulse draws less from the SC. The floor is
   set to the membrane-safe minimum — for this low-demand vehicle a higher floor
   (e.g. the old 150 W) would over-charge the SC.

3. **η-band cap (#3, `FC_ETA_BAND_HI` ≈ 388 W)**: the FC command is capped at the
   upper edge of the high-efficiency band, so the stack stays at 53–59 % η and the
   SC buffers demand peaks. `simulate()`'s SC-floor protection overrides the cap
   only if the SC would otherwise hit its hard floor.

4. **PI integral + lap-offset**: integral clamp `±400/K_i` and a per-lap offset
   nudge (`0.3 × ΔSOC × E_sc / 186`) drift the operating point toward charge
   sustenance over multiple laps.

---

## 7. Results Summary

### Final Rankings (VH=9.0, VL=6.5, PP=800 W, 11 laps, 14.5 km; MASS=180, AF=1.35)

> **km/m³ are charge-sustaining-normalised (dSOC=0) and include the FC→bus
> converter loss (η_dcdc=0.95) and the corrected SC round-trip (√0.97/dir).**
> High-drag vehicle (CdA=0.2025). These supersede the AF=0.8 figures (~205).

| Rank | Strategy | km/m³ | H₂ [g] | dSOC | CS? |
|------|----------|-------|--------|------|-----|
| 1 | **Strategy G** | **174.6** | 7.46 | +0.013 | ✓ |
| 2 | Strategy A (2D LUT) | ~172.9 | ~7.45 | −0.016 | ✗ |
| 3 | Constant FC | ~172.5 | ~7.61 | +0.011 | ✓ |
| 3 | Rule-Based | ~172.5 | ~7.59 | +0.007 | ✓ |
| 5 | Strategy H | ~172.4 | ~7.59 | +0.007 | ✓ |
| — | PI only | ~172.8 | ~6.99 | −0.101 | ✗ |

CS = Charge-Sustaining (`|dSOC| ≤ 0.015`). Strategy G leads by only ~1.5 km/m³ —
the high-drag duty cycle keeps average demand inside the FC η-band, so all
reasonable strategies cluster tightly.

### Motor iron-loss sensitivity (Strategy G, best profile)

| Variant | km/m³ | Note |
|---------|-------|------|
| v1 (optimistic, default) | **174.6** | lowest iron loss |
| v2 | ~144.7 | mid |
| v3 (pessimistic) | ~123.8 | highest iron loss |

> The iron-loss assumption dominates the result far more than any EMS choice
> (±25 % vs ±1 %) — motor bench-validation remains the top priority.

### EMS rec #3 — FC η-band cap (off by default)

| Config | km/m³ | CS? |
|--------|-------|-----|
| cap OFF (default) | **174.6** | ✓ |
| cap ON (≈385 W) | 175.5 | ✗ |

> Capping the FC at the η-band edge does **not** help this vehicle: average
> demand is already in-band, so capping only pushes peak energy through the SC
> (≈3 % round-trip loss), cancelling the efficiency gain — and it breaks charge
> sustenance. Left as opt-in (`p_eta_cap`) for genuinely low-demand duty cycles.

### Terrain-adaptive P&G (#2) sweep

| k_grade | km/m³ | dSOC | Note |
|---------|-------|------|------|
| 0 (flat) | **174.6** | +0.013 | best |
| 60 | 170.7 | +0.014 | no gain |
| 80 | 169.7 | +0.015 | no gain |

> The SEM 2025 EU track is too flat (~0.3 % grade) for terrain adaptation to pay
> off. The mechanism is implemented and ready for hillier circuits.

### P&G Grid Search Best (Strategy G, charge-sustaining)

> Normalised km/m³ for the high-drag vehicle (MASS=180, AF=1.35). VH<9.0 m/s
> overruns the 35.5-min cap. Re-run `grid_search_extended.py` for the full
> refreshed heatmap.

| Rank | VH [m/s] | VL [m/s] | PP [W] | km/m³ (normalised) |
|------|----------|----------|--------|-------|
| 1 | 9.0 | **6.5** | 800 | **174.6** |
| 2 | 9.0 | 7.0 | 800 | 173.3 |
| 3 | 9.0 | 6.5 | 700 | 172.0 |
| ~ | 9.5 | 6.5–7.5 | 600–800 | ~160–170 (faster → more drag loss) |

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
- Row 3: FC power [W] — Strategy G LPF trace with FC_P_MIN (100 W) floor
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

# 4. Compute km/m³  (use the charge-sustaining-normalised helper)
km3 = km_per_m3(result)            # uses result['m_H2_norm'] (dSOC=0 basis)
# result also carries 'm_H2' (raw), 'E_fc_J', 'dSOC'
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
I_fc = fc_current(P_fc)         # A, from polarisation curve (P_fc = FC net, stack side)
dH2  = K_H2 * I_fc * DT        # g per time step
# K_H2 = N·M_H2/(2F) = 52×2.016/(2×96485) ≈ 5.43×10⁻⁴ g/(A·s)  — Faraday's law, N=52 cells
# Note: P_fc delivers P_fc × ETA_DCDC (0.95) to the SC bus via the boost converter.
```

---

*Last updated: SEM 2026 pre-season simulation campaign.*
*Branch: `claude/fcs-strategy-folder-VHFN1` · Repo: `vallabh2504/ecogenium`*
