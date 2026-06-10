# Results — Hydraix I FCEV EMS (SEM Poland · Silesia Ring)

> Single source of truth for the **latest** simulation results. For the file
> structure, physics, and how-to-run, see [`README.md`](README.md). All numbers
> below use the team's **measured RPM×Torque motor map**
> (`datasheets/motor_eff_rpm_torque.xlsx`) and the **surveyed track CSV**
> (`datasheets/sem_2025_eu.csv`). Last refreshed: this revision.

---

## TL;DR

- **EMS strategy: Strategy G** (LPF feedforward + SOC-PI + lap offset). Chosen not
  for a headline number but because it is the **only adaptive, charge-sustaining**
  controller — it holds the supercap charge-neutral for *any* velocity profile, even
  imperfect/inconsistent laps. Inputs `I_mot, U_sc` → output `P_fc`.
- **Velocity profile: pulse-and-glide is optimal on the flat, cornerless model.**
  Grid-best **VH 9.0 / VL 7.0 / PP 1600 → 245.5 km/m³** (but 35.3 min, at the time
  edge); **time-safe VH 9.5 / VL 7.0 / PP 1200 → 238.8 km/m³** (34.8 min).
- **Corners change the answer at low grip:** below ~0.45 g sustained cornering grip,
  a steady ~30 km/h cruise beats P&G; above it, P&G wins. **Open decision: measure
  Hydraix I's real corner grip.**
- **Mileage was lifted from ~165 → ~235–245 km/m³** purely by replacing the old
  optimistic motor LUT with the measured map. The motor efficiency was the dominant
  lever; the EMS is only a ±1 % lever.

---

## 1. Configuration (all results assume this)

| Item | Value |
|------|-------|
| Vehicle | MASS 180 kg (incl. driver), frontal area AF 1.35 m², Cd 0.15, Crr 0.006, R_wheel 0.295 m, drivetrain η 0.95 |
| Fuel cell | balticFuelCells LC 52.30, ~1013 W net, floor 100 W, ramp 100 W/s, peak η_sys 59 % @120 W |
| Supercap | HyCap 20P×16S, ≈91 Wh usable, 40–60.8 V, ESR-based sag, round-trip √0.97/dir |
| DC-DC | FC→bus η 0.95 |
| Motor | BAFANG RM G060.1000, 48 V, measured RPM×Torque η map (η ≈ 0.17–0.83, ridge ~83 % at ≥18 Nm) |
| Track | SEM 2025 EU, lap 1319.627 m × 11 = 14.5 km, elevation span 3.26 m (≈flat), tightest corner R ≈ 20 m |
| Constraints | ≤ 35 min, 11 laps, **full stop each lap**, **no regen** |

### Power-demand formula (per `compute_motor_signals`)
```
P_wheel = Crr·M·g·v + ½·Cd·AF·ρ·v³ + M·a·v + M·g·θ·v        (no regen: P_wheel⁺ = max(P_wheel,0))
T  = (P_wheel⁺ / v) · R_wheel        RPM = v · 60/(2π·R_wheel)
η  = η_motor(RPM, T)                 [measured map, bilinear]
P_dem = P_wheel⁺ / η                 I_mot = P_dem / 48 V        U_mot = U_sc
```
Worked check (steady, flat): 10 m/s → P_wheel 230 W, T 6.8 Nm, RPM 324, η 0.738, **P_dem 311.6 W**, I_mot 6.5 A.

---

## 2. EMS strategy comparison (Strategy G chosen)

Strategy G vs the field, identical profile (VH 9.5 / VL 7.0 / PP 1200, charge-sustaining,
CS-normalised km/m³). The four well-formed CS strategies cluster within ~1 km/m³ — real
physics (CS + 91 Wh SC pins total FC energy; FC η-curve is broad/flat).

| Rank | Strategy | km/m³ | dSOC | CS? | Note |
|------|----------|-------|------|-----|------|
| 1 | Constant FC | 240.0 | −0.011 | ✓ | needs a known cycle |
| 2 | Strategy H (large-SC fixed dispatch) | 239.4 | +0.004 | ✓ | needs a known cycle |
| 3 | Rule-based (hysteresis) | 239.2 | +0.013 | ✓ | needs a known cycle |
| 4 | **Strategy G (LPF + SOC-PI)** | **238.9** | −0.008 | ✓ | **★ adaptive — chosen** |
| 5 | PI-only (no LPF) | 228.1 | +0.001 | ✓ | genuinely worse (no FF) |
| — | Strategy A (2D LUT) | 224.8 | +0.054 | ✗ | can't charge-sustain here |

**Why G over the tied Constant/H/Rule:** they assume a known drive cycle; G's feedforward
+ SOC-PI hold charge sustenance for any profile and reject imperfect laps. That adaptivity
is the deciding factor, not the ±1 km/m³ headline.

---

## 3. Velocity-profile optimization

### 3a. Profile families (cornerless, measured map) — `profile_optimization.png`
All families CS-tuned with Strategy G under the race constraints:

| Family | Best km/m³ | Verdict |
|--------|-----------|---------|
| **Pulse-and-glide** | **238.3** (VH9.5/VL7.0/PP1200) | **winner** |
| Constant cruise (~30 km/h) + coast-to-stop | 234.3 | −1.7 % |
| Elevation-adaptive | ≤ 236.6 | track too flat to help |
| Constant-power cruise | < P&G | worse |

P&G wins on the measured map because pulses load the motor onto the 83 % η ridge while
glides cost zero motor energy; steady cruise sits in the worse low-torque region.

### 3b. Grid search (measured map) — `grid_search_extended.png`
Mileage rises as cruise speed falls (less aero), so the optimum sits at the low-VH / 35-min edge:

| VH | VL | PP | km/m³ | dur | Note |
|----|----|----|-------|-----|------|
| **9.0** | 7.0 | 1600 | **245.5** | 35.3 min | grid-best — **at the time edge (>35.0)** |
| 9.0 | 6.5 | 1600 | 243.9 | 35.5 min | also at the edge |
| 9.5 | 7.0 | 1200 | 238.8 | 34.8 min | **time-safe pick** |
| 10.0 | 6.0 | 1400 | 237.2 | 34.6 min | deck profile |

**Time-margin trade-off:** lower VH wins on mileage but eats the time margin. Use VH 9.0
if 35 min has slack; VH 9.5 if it's hard. Below VH 9.0 overruns the cap entirely.

---

## 4. Corner-aware analysis — `corner_aware_profile.png`

Folding corner speed limits (`v_corner = √(a_lat·R)`, R from the surveyed UTM geometry)
into the comparison **changes which profile wins**, because braking for corners on a
no-regen car is pure KE loss:

| Profile (best of family) | cornerless | 0.3 g | 0.4 g | 0.5 g |
|--------------------------|-----------|-------|-------|-------|
| **Pulse-and-glide** | **238.3** | 218.0 | 230.9 | **236.6** |
| **Steady cruise (~30 km/h)** | 234.3 | **229.3** | **233.7** | 233.7 |

- **Crossover ≈ 0.45 g.** ≤ 0.4 g → steady cruise wins; ≥ 0.5 g → P&G wins.
- P&G peaks at 35–40 km/h, so corners (cap 28–36 km/h) force repeated braking → up to
  −8.5 % at low grip. The ~30 km/h cruise already runs under most caps → barely affected.
- **Open decision:** measure Hydraix I's real sustained corner grip — it picks the winner.

---

## 5. Data integrity (audited)

| Core file | Status |
|-----------|--------|
| `motor_eff_rpm_torque.xlsx` | ✅ Authoritative — byte-identical to the team's `motor_model_strategy_team.xlsx`. Used by **every** code path (mileage, legacy scripts, plot overlay). |
| `sem_2025_eu.csv` | ✅ True surveyed track (distance, elevation, UTM x/y, lat/lon). Distance + elevation feed the sim; UTM feeds corner radii. |
| FC / supercap curves | ✅ Hardcoded in `ems_core.py` from the datasheets. |
| `datasheets/archive/motor_lookup_table.xlsx` | 📦 Old 2D LUT — archived, **no code loads it**. |

---

## 6. Figures index (`plots/`)

| File | Shows |
|------|-------|
| `combined_best_result.png` | Main 6-panel run (velocity, SOC, FC/SC power, H₂, strategy table) |
| `grid_search_extended.png` | Refreshed P&G heatmap over (VH, VL) — best at low VH=9.0 |
| `profile_optimization.png` | P&G vs cruise/elevation/constant-power families + ranking |
| `corner_aware_profile.png` | Track map by corner cap + 0.3/0.4/0.5 g caps + winning corner-legal profile |
| `FC_LC52_30_system_curves_v3.png` | FC voltage/power/H₂ vs current |
| `strategy_g_slides.html` | 16-slide deck (Strategy G + the field + flowcharts + flag-map) |

---

## 7. Open questions / candidate next steps

1. **Corner grip number** — measuring Hydraix I's sustained cornering acceleration
   resolves the P&G-vs-cruise decision (§4).
2. **35-min limit: hard or soft?** — picks VH 9.0 (245.5) vs VH 9.5 (238.8) (§3b).
3. **Surveyed velocity-profile families beyond P&G/cruise** (informational survey done):
   most promising to simulate next are a **DP/PMP optimal reference** over the real
   track (provable ceiling; needs the curvature CSV) and **coast-into-corner /
   power-out** (highest-confidence real-world saving on a no-regen, corner-constrained
   car). Pending the user's go-ahead.
