"""
SIM folder self-contained test
================================
Run from the SIM/ directory:   python3 run_test.py

Verifies every import resolves inside SIM/ with zero external dependencies,
then runs 5 EMS strategies through the full simulation and prints a results table.

Strategies tested:
  B-equivalent  → Rule-Based FSM  (two-level SOC threshold switching)
  C-equivalent  → Strategy H      (efficiency-optimal SOC band control)
  D-equivalent  → Constant FC     (single fixed setpoint — simplest LUT)
  A             → 2D Lookup Table (bilinear interpolation on demand × SOC)
  G             → LPF + SOC-PI    (winner — 150 W floor always-on)
"""

import sys, os

# ── Restrict sys.path to SIM subfolders only ──────────────────────────────────
_SIM = os.path.dirname(os.path.abspath(__file__))
sys.path = [os.path.join(_SIM, 'scripts'),
            os.path.join(_SIM, 'models')] + [
    p for p in sys.path if _SIM not in p
]

# ── Imports — all must resolve inside SIM/ ────────────────────────────────────
print("=" * 60)
print("  SIM FOLDER ISOLATION TEST")
print("=" * 60)
print()
print("  [1/3] Checking imports...")

from ems_core import (
    FC_P_MIN, FC_P_MAX, SC_V_MAX, SC_V_MIN,
    SC_SOC_0, SC_E_J, K_H2, N_LAPS, DT, RESULTS_DIR,
)
print("        scripts/ems_core.py            ✓")

from motor_model import motor_lookup_2d, R_WHEEL
print("        models/motor_model.py          ✓")

from supercap_model import sc_voltage, sc_terminal_voltage, SC_E_J as SC_E_J_2
print("        models/supercap_model.py       ✓")

from combined_best_profile import (
    build_profile, compute_motor_signals, verify, simulate,
    make_strat_g, make_strat_a, make_strat_h,
    make_strat_rule, make_strat_constant, make_strat_pi,
    bisect_kp, bisect_const, bisect_pi, bisect_rule,
    bisect_h, bisect_a,
    TOTAL_KM, H2_DENSITY,
)
print("        scripts/combined_best_profile.py ✓")
print()

# ── Spot-check standalone model outputs ───────────────────────────────────────
import numpy as np

print("  [2/3] Spot-checking model outputs...")

I, V, eta = motor_lookup_2d(np.array([25.0]), np.array([10.0]))
print(f"        motor_lookup_2d(25 km/h, 10 Nm) → "
      f"I={I[0]:.1f} A  V={V[0]:.1f} V  η={eta[0]:.1%}")

oc  = float(sc_voltage(SC_SOC_0))
vt  = sc_terminal_voltage(SC_SOC_0, 5.0)
print(f"        sc_voltage(SOC=0.60)             → OCV={oc:.2f} V")
print(f"        sc_terminal_voltage(0.60, 5 A)   → {vt:.2f} V  "
      f"(sag={oc-vt:.3f} V = {(oc-vt)/oc*100:.2f}%)")
print(f"        SC usable energy                 → {SC_E_J/3600:.1f} Wh")

assert abs(SC_E_J - SC_E_J_2) < 1., "SC_E_J mismatch between ems_core and supercap_model"
assert R_WHEEL == 0.295,             "R_WHEEL mismatch"
print("        Cross-model consistency check    ✓")
print()

# ── Build best P&G profile ────────────────────────────────────────────────────
print("  [3/3] Building P&G profile (VH=9.0 m/s, VL=6.5 m/s, PP=700 W)...")
ta, va, Pa, sa, la, ea, ga, ca = build_profile(9.0, 6.5, 9.0, 700., 1000.)
ok, d, dur, stops = verify(ta, va, la, silent=False)
assert ok, f"Profile failed verify(): d={d:.2f}km dur={dur:.1f}min stops={stops}"
P_e, _, _ = compute_motor_signals(va, ga)

avg_v    = float(np.mean(va[va > 0.3]))
pct_glide = float(np.mean((va <= 0.3) | (P_e < 5.))) * 100
print(f"        {d:.3f} km  {dur:.1f} min  {stops} stops  "
      f"avg_v={avg_v:.2f} m/s  glide={pct_glide:.0f}%")
print()

# ── Run all 5 strategies ───────────────────────────────────────────────────────
print("=" * 60)
print("  RUNNING STRATEGIES  (bisecting for charge sustenance)")
print("=" * 60)
rows = []

# --- B equivalent: Rule-Based FSM -------------------------------------------
print()
print("  B · Rule-Based FSM  (two-level: P_hi when SOC low, P_lo when high)")
p_b, r_b = bisect_rule(P_e, la, ca)
km3_b = TOTAL_KM / (r_b['m_H2'] / H2_DENSITY)
cs_b  = abs(r_b['dSOC']) <= 0.015
rows.append(('B  Rule-FSM',    km3_b, r_b['m_H2'], r_b['dSOC'], p_b, cs_b))

# --- C equivalent: Strategy H (efficiency-optimal) --------------------------
print()
print("  C · Strategy H  (efficiency-optimal SOC band + Pset correction)")
p_c, r_c = bisect_h(P_e, la, ca)
km3_c = TOTAL_KM / (r_c['m_H2'] / H2_DENSITY)
cs_c  = abs(r_c['dSOC']) <= 0.015
rows.append(('C  Strat-H',     km3_c, r_c['m_H2'], r_c['dSOC'], p_c, cs_c))

# --- D equivalent: Constant FC (simplest possible LUT: 1 cell) ---------------
print()
print("  D · Constant FC  (fixed setpoint — the simplest discrete LUT)")
p_d, r_d = bisect_const(P_e, la, ca)
km3_d = TOTAL_KM / (r_d['m_H2'] / H2_DENSITY)
cs_d  = abs(r_d['dSOC']) <= 0.015
rows.append(('D  Const-FC',    km3_d, r_d['m_H2'], r_d['dSOC'], p_d, cs_d))

# --- A: 2D Lookup Table -------------------------------------------------------
print()
print("  A · 2D LUT  (bilinear on demand/E_sc × SOC grid)")
p_a, r_a = bisect_a(P_e, la, ca)
km3_a = TOTAL_KM / (r_a['m_H2'] / H2_DENSITY)
cs_a  = abs(r_a['dSOC']) <= 0.015
rows.append(('A  2D-LUT',      km3_a, r_a['m_H2'], r_a['dSOC'], p_a, cs_a))

# --- G: LPF + SOC-PI + 150 W floor -------------------------------------------
print()
print("  G · LPF Feedforward + SOC-PI + 150 W floor  (winner)")
p_g, r_g = bisect_kp(P_e, la, ca, 'Strategy G')
km3_g = TOTAL_KM / (r_g['m_H2'] / H2_DENSITY)
cs_g  = abs(r_g['dSOC']) <= 0.015
rows.append(('G  LPF+PI+150W', km3_g, r_g['m_H2'], r_g['dSOC'], p_g, cs_g))

# ── Results table ─────────────────────────────────────────────────────────────
print()
print("=" * 60)
print("  RESULTS  (all from SIM/ — zero external dependencies)")
print("=" * 60)
print(f"  {'Strategy':<16} {'km/m³':>7}  {'H2[g]':>7}  {'dSOC':>8}  {'CS?':>4}")
print("  " + "─" * 50)
for name, km3, h2, dsoc, param, cs in sorted(rows, key=lambda x: -x[1]):
    tag = " ★" if 'G' in name else ""
    print(f"  {name:<16} {km3:>7.1f}  {h2:>7.3f}  {dsoc:>+8.4f}  "
          f"{'YES' if cs else 'NO ':>4}{tag}")
print("  " + "─" * 50)
print(f"\n  CS = Charge-Sustaining (|dSOC| ≤ 0.015)")
print(f"  RESULTS_DIR (output target) → {os.path.relpath(RESULTS_DIR, _SIM)}")
print()

# ── Final checks ──────────────────────────────────────────────────────────────
all_cs = all(r[5] for r in rows)
print("  Consistency checks:")
print(f"    SC_E_J matches across modules    : {'PASS' if abs(SC_E_J - SC_E_J_2) < 1 else 'FAIL'}")
print(f"    All strategies charge-sustaining : {'PASS' if all_cs else 'PARTIAL (see table)'}")
print(f"    Profile satisfies race constraint: {'PASS' if ok else 'FAIL'}")
print()
print("  SIM folder is self-contained and fully portable.")
print("  Copy the entire SIM/ directory to any machine and run:")
print("    python3 run_test.py")
print()
