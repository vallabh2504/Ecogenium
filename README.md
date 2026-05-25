# Ecogenium — Hydraix I · Shell Eco-marathon 2026

Technical repository for the Ecogenium hydrogen fuel-cell vehicle competing in Shell Eco-marathon Urban Concept (Silesia Ring, 2026).

## Repository Structure

```
Ecogenium/
├── fcs-strategy/          Fuel-cell system & vehicle power strategy
│   ├── matlab/            Live MATLAB + Python analysis scripts (run these)
│   ├── matlab-archive/    Reference-only v1 MATLAB scripts (superseded)
│   ├── fc-model/          Fuel-cell polarisation model (Python)
│   ├── datasheets/        Component datasheets (FC stack manual)
│   └── results/           Generated output figures
│
└── telematics/            On-car data acquisition & pit-wall system
    ├── hardware/           PCB design files (Altium — TelematicsBoardV1)
    ├── firmware/           RP2040 on-car firmware (C++)
    ├── server/             Pit-wall Qt widget + Docker stack
    ├── cloud-setup/        Next.js / Supabase cloud telemetry pipeline
    └── docs/               Calypso radio configuration
```

## Quick Start

### Power & Efficiency Analysis (Python)
```bash
cd fcs-strategy/matlab
python run_power_plot.py
# → fcs-strategy/results/*.png
```

### Drive Cycle Analysis (MATLAB)
```matlab
cd fcs-strategy/matlab
RUN_ALL_ANALYSIS   % runs all three modules in sequence
```

### Fuel-Cell System Model (Python)
```bash
cd fcs-strategy/fc-model
python FuelCellEstimate_v3.py
```

## Key Parameters (Hydraix I)

| Parameter | Value | Source |
|---|---|---|
| Vehicle mass | 175 kg | Measured estimate |
| Motor | BAFANG RM G060.1000 | Datasheet (datasheets/) |
| Motor rated power | 1213 W | Factory test sheet |
| Motor peak efficiency | 83.4 % | Factory test sheet |
| FC stack | balticFuelCells LC 52.30 | FC_Manual.pdf |
| FC nominal power | 1040 W | FC_Manual.pdf |
| Drive cycle | 11 laps × 14.5 km, 35 min | Silesia Ring SEM course |
| Peak electrical demand | 1205 W | run_power_plot.py |
| Energy per lap | 13.73 Wh | run_power_plot.py |

## Race Strategy Summary

The vehicle uses a **pulse-and-glide** strategy: full-throttle burst to cruise speed (~30 km/h) followed by motor-off coasting per lap. Key findings from the analysis:

- Peak electrical demand (1205 W) **slightly exceeds** FC nominal max (1040 W) — a small energy buffer (supercapacitor or Li cell) is recommended for the acceleration phase.
- Mean running power is ~274 W, well within FC capability.
- Motor efficiency at light cruise loads (100–300 W) is only 40–70% — the dominant loss in the drivetrain.

## Branch Policy

| Branch | Purpose |
|---|---|
| `main` | Stable, reviewed work |
| `claude/fcs-strategy-folder-*` | FCS analysis development |

---

*Competition: Shell Eco-marathon Europe 2026, Urban Concept H₂ Fuel Cell*
