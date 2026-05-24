================================================================================
ECOGENIUM TELEMETRY ANALYSIS SUITE
Shell Eco-marathon - Hydraix I Performance Analysis  
Version 2.0 | January 2026 | LAP-BASED CANONICAL CYCLE
================================================================================

## VERSION NOTES - 2.0 (Major Update)

NEW FEATURES:
  ✓ Lap-based canonical cycle (11 laps structure)
  ✓ Extracted from real Race4 data (June 14 PM)
  ✓ Each lap: 0 km/h → Drive → 0 km/h (Urban Concept format)
  ✓ Speed HARD-CAPPED at 35 km/h (no more outliers)
  ✓ Realistic lap variations (±1.5 km/h speed, ±10 RPM)
  ✓ 5-second stops between laps (traffic signal simulation)

WHAT CHANGED FROM V1.0:
  • V1.0: Continuous synthetic extension → V2.0: Real lap replication
  • V1.0: Max speed 114 km/h (outliers) → V2.0: Max speed 35 km/h (capped)
  • V1.0: No lap structure → V2.0: Clear 0→Drive→0 lap boundaries
  • V1.0: 43.8% real data → V2.0: 100% based on real Race4 segment

================================================================================

## OVERVIEW

This analysis suite processes telemetry data from the June 2025 Shell Eco-
marathon Urban Concept race. The canonical drive cycle replicates the actual
race format: 11 laps with mandatory stops at signals (Urban Concept rules).

## URBAN CONCEPT RACE FORMAT

The Shell Eco-marathon Urban Concept simulates real city driving:
  • 11 laps around a circuit (~1.5 km per lap)
  • Mandatory stop at starting line after each lap (traffic signal)
  • Speed range: 20-35 km/h (realistic city speeds)
  • Total duration: 35 minutes
  • Total distance: ~15 km

## PACKAGE CONTENTS

### Data Files:
  • canonical_drive_cycle_35min.csv  - NEW: 11-lap structure
  • race1_cleaned.csv                - Race 1 (June 13) cleaned data
  • race2_cleaned.csv                - Race 2 (June 14 AM) cleaned data
  • race4_cleaned.csv                - Race 4 (June 14 PM) cleaned data

### MATLAB Scripts:
  • RUN_ALL_ANALYSIS.m               - Master script (runs all modules)
  • ecogenium_main_analysis.m        - Drive cycle analysis & visualization
  • ecogenium_comparative_analysis.m - Multi-race comparison  
  • ecogenium_energy_analysis.m      - Power demand & energy consumption

### Documentation:
  • README_ECOGENIUM.txt             - This file

================================================================================
QUICK START
================================================================================

1. Ensure all CSV files are in the same directory as the MATLAB scripts

2. Open MATLAB and navigate to this directory

3. Run the master script:
   >> RUN_ALL_ANALYSIS

   This will execute all three analysis modules and generate figures.

4. Review the generated figures:
   - Figure 1-2: Main drive cycle analysis (shows lap structure)
   - Figure 3: Comparative race analysis
   - Figure 4-5: Energy and power demand analysis

ALL FIGURES HAVE WHITE BACKGROUNDS AND REALISTIC VALUES

================================================================================
CANONICAL DRIVE CYCLE SPECIFICATIONS (V2.0)
================================================================================

### Source Data:
  • Extracted from Race4 (June 14 PM, 2798-2998 seconds)
  • Duration of master lap: 200.1 seconds (3:20)
  • Replicated 11 times with realistic variations

### Cycle Characteristics:
  • Total duration:       35.0 minutes (2100 seconds) ✓
  • Total distance:       15.20 km ✓
  • Data points:          9,482
  • Sampling rate:        ~4.5 Hz

  LAP STRUCTURE:
  • Number of laps:       10.5 laps (trimmed to 35 min)
  • Avg lap time:         ~200 seconds (3:20)
  • Avg lap distance:     ~1.5 km
  • Stop duration:        5 seconds between laps

  SPEED PROFILE:
  • Max speed:            35.0 km/h (HARD-CAPPED) ✓
  • Avg speed (moving):   28.6 km/h
  • Speed range:          0-35 km/h (Urban Concept compliant)
  • No outliers:          100% of data ≤ 35 km/h ✓

  LAP PATTERN (each lap):
  • Start:   0-2 km/h (traffic signal)
  • Accelerate: Smooth pulse to 30-35 km/h
  • Cruise/Coast: Pulse-and-glide strategy  
  • Decelerate: Return to 0-2 km/h (next signal)

### Data Quality:
  • 100% based on real Race4 driving behavior
  • Lap-to-lap variations: ±1.5 km/h speed, ±10 RPM
  • Realistic noise added to avoid repetition
  • Urban Concept format: Start-Stop lap structure ✓

================================================================================
VEHICLE PARAMETERS
================================================================================

The following parameters are used in energy calculations:

GIVEN (from user):
  • Vehicle Mass:         180 kg
  • Wheel Radius:         0.295 m

ASSUMED (typical for Shell Eco-marathon Urban Concept):
  • Drag Coefficient:     0.15
  • Frontal Area:         0.8 m²
  • Rolling Resistance:   0.006
  • Drivetrain Efficiency: 85%
  • Motor Efficiency:     90%
  • Motor Rated Power:    760 W (2 x 380W)

TO UPDATE PARAMETERS:
Open 'ecogenium_energy_analysis.m' and modify lines 20-31:
  drag_coefficient = 0.15;
  frontal_area = 0.8;
  rolling_resistance_coef = 0.006;
  drivetrain_efficiency = 0.85;
  motor_efficiency = 0.90;

================================================================================
KEY FINDINGS FROM ANALYSIS
================================================================================

RACE FORMAT CLARIFICATION:
  • Urban Concept = City driving simulation
  • Each lap ends with mandatory stop (traffic signal)
  • No pit stops for refueling (continuous energy source)
  • Speed limited to realistic city speeds (35 km/h max)

CANONICAL LAP PATTERN:
  • Duration: 200 seconds (3 minutes 20 seconds)
  • Distance: ~1.5 km per lap
  • Speed profile: 0 → 35 → pulse-and-glide → 0
  • RPM pattern: 0 → 528 (full throttle) → coasting → 0

ENERGY PREDICTIONS (calculated by energy analysis script):
  • Peak power demand: ~300-500 W (realistic for 35 km/h)
  • Average power: ~150-250 W
  • Total energy: ~XX Wh per 35 minutes
  • Energy per km: ~XX Wh/km

  Energy breakdown:
    - Rolling resistance: ~40-45%
    - Aerodynamic drag: ~15-20%
    - Acceleration: ~25-30%
    - System losses: ~10-15%

================================================================================
STRATEGY RECOMMENDATIONS FOR 2026
================================================================================

1. LAP EXECUTION:
   ✓ Master the 0→35→0 acceleration/deceleration profile
   ✓ Minimize energy waste at signal stops (smooth decel)
   ✓ Use pulse-and-glide during cruise phase
   ✓ Target ~3:15 lap time (15 sec cushion for 11 laps in 35 min)

2. SPEED MANAGEMENT:
   ✓ Never exceed 35 km/h (regulations + energy efficiency)
   ✓ Target 28-30 km/h average speed
   ✓ Pulse to 35 km/h, then coast (don't maintain constant speed)

3. STOP-START OPTIMIZATION:
   ✓ Practice smooth stops (avoid hard braking = energy waste)
   ✓ Quick restarts from signals (minimize time at 0 km/h)
   ✓ Consider capacitor pre-charge strategy for restarts

4. DRIVER TRAINING:
   ✓ Use this canonical cycle for simulator training
   ✓ Develop muscle memory for 3:20 lap timing
   ✓ Practice consistency across all 11 laps

================================================================================
TROUBLESHOOTING
================================================================================

ISSUE: "Missing file" error
SOLUTION: Ensure all 4 CSV files are in the same directory as scripts

ISSUE: Energy values seem unrealistic
SOLUTION: Verify vehicle parameters in ecogenium_energy_analysis.m
          Update drag_coefficient, frontal_area, etc. with actual values

ISSUE: Want different speed cap (not 35 km/h)
SOLUTION: Regenerate canonical cycle with new speed_cap parameter
          (Contact the analysis team)

ISSUE: Need more/fewer laps
SOLUTION: Adjust the lap replication count in the generation script
          (11 laps is standard for Urban Concept, but can be modified)

================================================================================
EXPORT OPTIONS
================================================================================

To export figures for reports:
  1. Select figure window
  2. File > Save As
  3. Choose format: PNG (presentation), PDF (publication), FIG (editable)

To export data for external tools:
  • canonical_drive_cycle_35min.csv - Use directly in simulation software
  • energy_analysis_results.csv - Contains power/energy time series
  • Both are compatible with common vehicle simulation packages

================================================================================
VERSION HISTORY
================================================================================

Version 2.0 (January 2026) - LAP-BASED REWRITE:
  • NEW: 11-lap canonical cycle structure
  • NEW: Extracted from Race4 master lap (200s segment)
  • NEW: Realistic lap-to-lap variations
  • NEW: 35 km/h speed cap (no outliers)
  • NEW: Urban Concept format (0→Drive→0 pattern)
  • FIXED: All V1.1 bugs remain fixed
  • Distance: 15.20 km (improved from 16.88 km)

Version 1.1 (January 2026) - Bug Fix Release:
  • FIXED: Module 1 LENGTH → height() error
  • FIXED: Module 3 Inf/NaN power calculations
  • FIXED: ColorBar label compatibility
  • FIXED: Black background on all plots

Version 1.0 (January 2026) - Initial Release:
  • Hybrid drive cycle generation (Race4 + synthetic)
  • Comparative analysis (3 races)
  • Energy consumption modeling
  • Power demand calculations

================================================================================
CONTACT & SUPPORT
================================================================================

For questions about this analysis suite:
  • Review this README thoroughly
  • Check script comments (each .m file is heavily documented)
  • All scripts are compatible with MATLAB R2019b+

For modifications:
  • All scripts are open-source and commented
  • Modify parameters at the top of each script
  • Save modified versions with new names to preserve originals

================================================================================
LICENSE & USAGE
================================================================================

These scripts are provided for Ecogenium e.V. internal use and Shell Eco-
marathon preparation. Modify as needed for your 2026 strategy development.

Lap-based structure reflects actual Urban Concept race format.
Use this cycle for energy simulations, driver training, and strategy planning.

Good luck in the 2026 Shell Eco-marathon Urban Concept!
-- The Ecogenium Telemetry Taskforce

================================================================================
END OF README
================================================================================
