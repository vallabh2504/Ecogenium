"""
grid_search_extended.py — Extended P&G grid search over (VH, VL, PP)

Finds optimal pulse-and-glide setpoint for Strategy G by minimising aerodynamic
drag at lower peak speeds while maintaining the hard constraint:
  - 14.5 km in ≤ 35 minutes over 11 laps with coast-to-stop at end of every lap.

VH range: 7.0–9.5 m/s
VL range: 5.5–8.0 m/s
PP range: 400–700 W
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import all needed pieces from the now-importable combined_best_profile
from combined_best_profile import (
    build_profile, compute_motor_signals, verify, simulate, make_strat_g,
    TOTAL_KM, H2_DENSITY, DT, N_LAPS, bisect_kp,
)
from ems_core import (
    SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX, SC_E_J, SC_V_MIN, SC_V_MAX,
    FC_P_MIN, FC_P_MAX, FC_RAMP, K_H2, fc_current, sc_soc_update,
    sc_voltage, sc_terminal_voltage, motor_lookup_2d, R_WHEEL,
    MATLAB_DIR, RESULTS_DIR,
)
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Extended grid search parameters ───────────────────────────────────────────
VH_VALS = [7.0, 7.5, 8.0, 8.5, 9.0, 9.5]
VL_VALS = [5.5, 6.0, 6.5, 7.0, 7.5, 8.0]
PP_VALS = [400., 500., 600., 700.]
V_DH    = 9.0   # downhill speed cap — keep fixed

print("=== Extended P&G Grid Search — Strategy G ===")
print(f"VH: {VH_VALS}")
print(f"VL: {VL_VALS}")
print(f"PP: {PP_VALS}")
print(f"V_DH: {V_DH} m/s (fixed)")
print(f"Constraint: 14.5 km / 35 min / 11 laps / coast-to-stop")
print()

results = []
total_combos = sum(1 for VH in VH_VALS for VL in VL_VALS if VL < VH - 0.5) * len(PP_VALS)
done = 0

for VH in VH_VALS:
    for VL in VL_VALS:
        if VL >= VH - 0.5:   # need at least 0.5 m/s band
            continue
        for PP in PP_VALS:
            done += 1
            print(f"[{done}/{total_combos}] VH={VH} VL={VL} PP={PP:.0f}W ...")
            ta, va, Pa, sa, la, ea, ga, ca = build_profile(VH, VL, V_DH, PP, 1000.)
            ok, d, dur, stops = verify(ta, va, la)
            if not ok:
                print(f"  SKIP VH={VH} VL={VL} PP={PP:.0f}W  ({d:.2f}km {dur:.1f}min {stops}stops)")
                continue
            P_e, _, _ = compute_motor_signals(va, ga)
            avg_v = float(np.mean(va[va > 0.3]))
            motor_on = float(np.mean(va > 0.3)) * 100.
            Kp, r = bisect_kp(P_e, la, ca, f"VH={VH} VL={VL} PP={PP:.0f}W")
            km3 = TOTAL_KM / (r['m_H2'] / H2_DENSITY)
            cs  = abs(r['dSOC']) <= 0.015
            results.append(dict(VH=VH, VL=VL, PP=PP, Kp=Kp,
                                H2=r['m_H2'], km3=km3, dSOC=r['dSOC'],
                                cs=cs, d=d, dur=dur, avg_v=avg_v, motor_on=motor_on))
            print(f"  VH={VH} VL={VL} PP={PP:.0f}W  km/m³={km3:.1f}  H2={r['m_H2']:.3f}g  "
                  f"dSOC={r['dSOC']:+.4f}  CS={'YES' if cs else 'NO'}  "
                  f"avg_v={avg_v:.2f}m/s  dur={dur:.1f}min")

print(f"\nGrid search complete: {len(results)} profiles evaluated.")

# ── Heatmap figure ─────────────────────────────────────────────────────────────
from matplotlib.colors import Normalize
import matplotlib.cm as cm

# Build heatmap grid — best charge-sustaining PP per (VH, VL) cell
vh_u = sorted(set(r['VH'] for r in results if r['cs']))
vl_u = sorted(set(r['VL'] for r in results if r['cs']))

if vh_u and vl_u:
    grid = np.full((len(vl_u), len(vh_u)), np.nan)
    best_km3_grid = {}
    for r in results:
        if not r['cs']:
            continue
        vi = vh_u.index(r['VH'])
        li = vl_u.index(r['VL'])
        if np.isnan(grid[li, vi]) or r['km3'] > grid[li, vi]:
            grid[li, vi] = r['km3']
            best_km3_grid[(r['VH'], r['VL'])] = r

    fig, ax = plt.subplots(figsize=(10, 7))
    im = ax.imshow(grid, aspect='auto', origin='lower',
                   extent=[vh_u[0]-0.25, vh_u[-1]+0.25, vl_u[0]-0.25, vl_u[-1]+0.25],
                   cmap='RdYlGn')
    plt.colorbar(im, ax=ax, label='km/m³ (Strategy G, charge-sustaining)')
    ax.set_xlabel('V_HI [m/s]')
    ax.set_ylabel('V_LO [m/s]')
    ax.set_title('Strategy G km/m³ heatmap over (VH, VL) — best PP per cell')

    # Annotate each cell
    for li, vl in enumerate(vl_u):
        for vi, vh in enumerate(vh_u):
            if not np.isnan(grid[li, vi]):
                r = best_km3_grid.get((vh, vl))
                ax.text(vh, vl, f"{grid[li,vi]:.0f}\nPP={r['PP']:.0f}W",
                        ha='center', va='center', fontsize=7)

    # Mark current best and new best
    new_best = max((r for r in results if r['cs']), key=lambda x: x['km3'], default=None)
    ax.plot(9.0, 7.0, 'r*', ms=15, label='Current (VH=9, VL=7)')
    if new_best:
        ax.plot(new_best['VH'], new_best['VL'], 'g*', ms=15,
                label=f"New best VH={new_best['VH']} VL={new_best['VL']} PP={new_best['PP']:.0f}W → {new_best['km3']:.1f}")
    ax.legend(fontsize=9)

    plt.tight_layout()
    out_png = os.path.join(RESULTS_DIR, 'grid_search_extended.png')
    plt.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"\nHeatmap saved → {out_png}")
else:
    print("\nNo charge-sustaining results found — skipping heatmap.")
    new_best = None

# ── Summary table ──────────────────────────────────────────────────────────────
cs_results = [r for r in results if r['cs']]
cs_results.sort(key=lambda x: x['km3'], reverse=True)

print("\n=== Extended Grid Search Results (Strategy G, charge-sustaining only) ===")
hdr = f"{'Rank':>4} | {'VH':>4} | {'VL':>4} | {'PP':>5} | {'km/m³':>6} | {'H2[g]':>6} | {'dSOC':>7} | {'K_p':>6} | {'dur[min]':>8} | avg_v"
sep = "-" * len(hdr)
print(hdr)
print(sep)

CURRENT_BEST_KM3 = 206.8
for rank, r in enumerate(cs_results[:15], 1):
    marker = " <-- CURRENT" if (abs(r['VH']-9.0)<0.01 and abs(r['VL']-7.0)<0.01 and abs(r['PP']-600.)<1.) else ""
    print(f"  {rank:2d} | {r['VH']:4.1f} | {r['VL']:4.1f} | {r['PP']:5.0f} | "
          f"{r['km3']:6.1f} | {r['H2']:6.3f} | {r['dSOC']:+7.4f} | "
          f"{r['Kp']:6.0f} | {r['dur']:8.1f} | {r['avg_v']:.2f}m/s{marker}")

# ── Save new best profile CSV (only if it beats 206.8 km/m³) ──────────────────
new_best_cs = max((r for r in results if r['cs']), key=lambda x: x['km3'], default=None)

if new_best_cs and new_best_cs['km3'] > CURRENT_BEST_KM3:
    ta, va, Pa, sa, la, ea, ga, ca = build_profile(
        new_best_cs['VH'], new_best_cs['VL'], V_DH, new_best_cs['PP'], 1000.)
    df_out = pd.DataFrame({
        'time_s': ta, 'velocity_ms': va, 'velocity_kmh': va*3.6,
        'dist_in_lap_m': sa, 'lap_num': la, 'elevation_m': ea,
        'grade': ga, 'P_elec_W': Pa,
    })
    csv_path = os.path.join(MATLAB_DIR, 'sem_combined_best.csv')
    df_out.to_csv(csv_path, index=False)
    print(f"\nNew best profile saved → {csv_path}")
    print(f"  VH={new_best_cs['VH']} VL={new_best_cs['VL']} PP={new_best_cs['PP']:.0f}W  "
          f"km/m³={new_best_cs['km3']:.1f}")
else:
    print("\nCurrent profile (VH=9, VL=7, PP=600W) remains best.")
