# Ecogenium — Hydraix I · Shell Eco-marathon 2026

Technical repository for the Ecogenium hydrogen fuel-cell vehicle competing in Shell Eco-marathon Urban Concept (Silesia Ring, 2026).

## Repository Structure

```
Ecogenium/
├── fcs-strategy/                   Fuel-cell system & energy management strategies
│   ├── ems/                        Python EMS simulation suite (Strategies A–G)
│   ├── matlab/                     Drive-cycle analysis scripts + GPS velocity data
│   ├── matlab-archive/             Reference-only v1 MATLAB scripts (superseded)
│   ├── fc-model/                   Fuel-cell polarisation model (Python)
│   ├── datasheets/                 Component datasheets (FC stack, BAFANG motor)
│   └── results/                    Generated output figures
│
└── telematics/                     On-car data acquisition & pit-wall system
    ├── Hardware/                   PCB design files (Altium — TelematicsBoardV1)
    ├── Software/CarSide/           RP2040 on-car firmware (C++)
    ├── Software/ServerSide/        Pit-wall Qt widget + Docker stack
    ├── cloud-setup/                Next.js / Supabase cloud telemetry pipeline
    └── docs/                       Calypso radio configuration
```

## Key Parameters (Hydraix I)

| Parameter | Value | Source |
|---|---|---|
| Vehicle mass | 175 kg | Measured estimate |
| Drag coefficient × frontal area | C_D × A_f = 0.15 × 0.8 m² | Vehicle model |
| Rolling resistance | C_rr = 0.006 | Vehicle model |
| Motor | BAFANG RM G060.1000 | Datasheet (`datasheets/`) |
| Motor max power | 1213 W | Factory test sheet |
| Motor peak efficiency | 83.4 % @ 1213 W output | Factory test sheet |
| FC stack | balticFuelCells LC 52.30 | `FC_Manual.pdf` |
| FC max power | 1013 W | `ems_core.py` calibrated |
| Supercapacitor energy | 38 400 J (SOC 0 → 1) | SC bank spec |
| SC initial SOC | 0.60 | Race start |
| Drive cycle | 11 laps × 1318 m = 14.5 km, ≈35 min | Silesia Ring SEM course |
| Avg electrical demand (GPS) | 259 W | `ems_core.build_demand_profile()` |
| H₂ density at STP | 89.88 g/m³ | Physical constant |

## Energy Management Strategy Results

All strategies simulated over the full 11-lap race (14.5 km). km/m³ = 14.5 km ÷ (H₂ consumed [g] ÷ 89.88 g/m³).

| Strategy | Type | H₂ [g] | ΔSOC | km/m³ | Adaptive? |
|---|---|---|---|---|---|
| A — Continuous LUT | FC efficiency map look-up | 8.204 | ≈ 0 | 158.8 | No |
| C — ECMS / PMP | Equivalent consumption minimisation | 8.197 | ≈ 0 | **158.9** | No |
| E — Adaptive ECMS | Online s-factor update | 8.206 | ≈ 0 | 158.8 | Partial |
| B — Rule-Based FSM | SOC hysteresis thresholds | 8.425 | ≈ 0 | 154.6 | No |
| D — 4×4 Discrete Map | Binned power mapping | 8.487 | ≈ 0 | 153.5 | No |
| G — FF + SOC-PI | Feedforward + integral SOC control | 8.687 | +0.007 | 150.0 | **Yes** |
| F — LPF-EMS | Low-pass filtered feedforward | 8.775 | ≈ 0 | 148.2 | No |
| **G + Pulse-and-Glide** | P&G drive cycle + Strategy G | **4.999** | −0.048 | **260.7** | **Yes** |

> Strategy G is the only one that requires no pre-tuned velocity profile — it adapts to any drive cycle in real time using a SOC proportional-integral correction loop.

> Pulse-and-glide (v_low = 18 km/h → v_high = 30 km/h, P_pulse = 1000 W) cuts H₂ consumption by 42% vs GPS constant-speed by operating the BAFANG motor at its peak efficiency point (83.4%) rather than at partial load (65%).

## Quick Start

### Run all EMS strategies (Python)
```bash
cd fcs-strategy/ems
python strategy_a_lut.py          # Strategy A
python strategy_b_fsm.py          # Strategy B
python strategy_c_ecms.py         # Strategy C
python strategy_d_lut4x4.py       # Strategy D
python strategy_e_aecms.py        # Strategy E
python strategy_f_lpf.py          # Strategy F
python strategy_g_pi.py           # Strategy G — also prints cross-strategy table
python test_strategy_g_robustness.py   # G on WLTC + NEDC
python strategy_g_pulse_glide.py  # Pulse-and-glide grid sweep + comparison
```

Outputs land in `fcs-strategy/results/ems/`.

**Dependencies:** `numpy`, `pandas`, `matplotlib`, `scipy`
```bash
pip install numpy pandas matplotlib scipy
```

### Drive-cycle analysis (MATLAB)
```matlab
cd fcs-strategy/matlab
RUN_ALL_ANALYSIS    % runs all three MATLAB modules in sequence
```

### FC system model (Python)
```bash
cd fcs-strategy/fc-model
python FuelCellEstimate_v3.py
```

## Branch Policy

| Branch | Purpose |
|---|---|
| `main` | Stable, reviewed work |
| `claude/fcs-strategy-folder-*` | FCS analysis development |

---

*Competition: Shell Eco-marathon Europe 2026, Urban Concept H₂ Fuel Cell, Silesia Ring*
