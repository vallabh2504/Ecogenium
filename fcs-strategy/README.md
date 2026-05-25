# FCS Strategy — Fuel-Cell System & Vehicle Power Analysis

This folder contains all modelling and analysis work for the Hydraix I fuel-cell powertrain.

## Folder Map

```
fcs-strategy/
├── matlab/              ← START HERE for drive-cycle analysis
│   ├── run_power_plot.py          Python: vehicle power + BAFANG efficiency map
│   ├── RUN_ALL_ANALYSIS.m         MATLAB: master runner (calls all 3 modules)
│   ├── ecogenium_main_analysis.m  MATLAB: canonical drive-cycle plots
│   ├── ecogenium_comparative_analysis.m  MATLAB: Race1/2/4 comparison
│   ├── ecogenium_energy_analysis.m       MATLAB: energy breakdown
│   ├── canonical_with_incline.csv        GPS drive cycle (11-lap, incline data)
│   ├── race1_cleaned.csv, race2_cleaned.csv, race4_cleaned.csv
│   └── energy_analysis_results.csv       MATLAB output table
│
├── matlab-archive/      Old v1 MATLAB scripts — kept for reference only
│
├── fc-model/            Fuel-cell system polarisation model
│   ├── FuelCellEstimate_v3.py     Current model (LC 52.30 + BOP + efficiency)
│   ├── FuelCellEstimate_v2.py     Intermediate version
│   └── FuelCellEstimate.py        Original prototype
│
├── datasheets/
│   └── FC_Manual.pdf              balticFuelCells LC 52.30 stack manual
│
└── results/             Generated figures (do not edit by hand)
    ├── velocity_profile_calibrated.png   GPS velocity profile (calibrated)
    ├── power_vs_time_calibrated.png      Electrical power demand vs time
    ├── motor_efficiency_map.png          BAFANG η-vs-P curve + drive-cycle cloud
    ├── FC_LC52_30_system_curves_v3.png   FC polarisation curves
    └── archive/                          Superseded output images
```

## Running the Python Script

```bash
cd fcs-strategy/matlab          # data files must be in the same directory
python run_power_plot.py
```

Outputs three PNG files to `fcs-strategy/results/`.

**Dependencies:** `numpy`, `pandas`, `matplotlib`
```bash
pip install numpy pandas matplotlib
```

## Running the MATLAB Scripts

Open MATLAB, set the working directory to `fcs-strategy/matlab/`, then run:
```matlab
RUN_ALL_ANALYSIS
```

This calls all three analysis modules in sequence and generates Figures 1–5.
MATLAB R2020b or later recommended.

## Drive Cycle Details

| Property | Value |
|---|---|
| Source | Race 4 GPS master lap (t=2798–2998 s), compressed + replicated |
| Laps | 11 |
| Distance | 14.5 km (1318 m/lap) |
| Duration | 35 minutes |
| GPS calibration scale | 0.9064× (corrects odometry overcount) |
| Resampling | Uniform 5 Hz + 10-second velocity smooth |
| Peak speed | 32.6 km/h |

## Key Results (latest run)

| Metric | Value |
|---|---|
| Peak electrical demand | **1205 W** (FC max: 1040 W → buffer needed) |
| Mean power (moving) | 274 W |
| Energy per lap | 13.73 Wh |
| Total race energy | 151 Wh (0.151 kWh) |
| Motor η mean (running) | 59.2 % |
| Motor η at rated load | 83.4 % |
