"""
FC System Efficiency Map — balticFuelCells LC 52.30
Hydraix I · SEM 2026

Shows η_sys and η_stack vs current / net power with all key operating
points and efficiency zones annotated.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "FCS Strategy"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D
from matplotlib import gridspec

from FuelCellEstimate_v3 import (
    P_net, eta_system_net, eta_stack_gross,
    stack_voltage, P_stack_gross, P_BOP,
    P_blower, H2_mass_flow_gs,
    I_max, I_nom, N, A,
    E0, b, J0, ASR, J_L,
    P_PUMP_FIXED, P_ELECTRONICS,
)
from ems_core import fc_current, RESULTS_DIR

# ── Sweep ─────────────────────────────────────────────────────────────────────
I = np.linspace(0.05, I_max, 10_000)
Pn     = P_net(I)
Es     = eta_system_net(I)
Eg     = eta_stack_gross(I)
Vstack = stack_voltage(I)
Pgross = P_stack_gross(I)
Pbop   = P_BOP(I)
Pbl    = P_blower(I)

# Key scalar points
pk       = int(np.argmax(Es))
P_PEAK   = float(Pn[pk]);    I_PEAK   = float(I[pk]);    ETA_PEAK = float(Es[pk])
P_NOM    = float(Pn[np.argmin(np.abs(I - I_nom))])
I_STRAT  = float(fc_current(268.0))           # Strategy A operating current
P_STRAT  = float(P_net(np.array([I_STRAT]))[0])
ETA_STRAT= float(eta_system_net(np.array([I_STRAT]))[0])

# 50 % and 55 % efficiency crossings
def _crossing(threshold, branch='desc'):
    crosses = np.where(np.diff(np.sign(Es - threshold)))[0]
    results = []
    for idx in crosses:
        p = float(np.interp(threshold,
                            [Es[idx], Es[idx+1]], [Pn[idx], Pn[idx+1]]))
        ascending = Es[idx] < Es[idx+1]
        results.append((p, ascending))
    return results

c50 = _crossing(50.0)
c55 = _crossing(55.0)
P50_asc  = next((p for p, asc in c50 if asc),  None)
P50_desc = next((p for p, asc in c50 if not asc), None)
P55_asc  = next((p for p, asc in c55 if asc),  None)
P55_desc = next((p for p, asc in c55 if not asc), None)

print(f"Peak η_sys : {ETA_PEAK:.2f}%  @ I={I_PEAK:.2f}A  P={P_PEAK:.1f}W")
print(f"η>55% window : {P55_asc:.1f} – {P55_desc:.1f} W")
print(f"η>50% window : {P50_asc:.1f} – {P50_desc:.1f} W")
print(f"Strategy A op : I={I_STRAT:.2f}A  P={P_STRAT:.1f}W  η={ETA_STRAT:.2f}%")

# ── Figure ────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(18, 13), dpi=160)
fig.patch.set_facecolor('#0D1117')

gs = gridspec.GridSpec(2, 2, figure=fig,
                       hspace=0.38, wspace=0.32,
                       left=0.07, right=0.96,
                       top=0.91, bottom=0.07)

ax_eff  = fig.add_subplot(gs[0, :])   # top — main efficiency map (full width)
ax_vi   = fig.add_subplot(gs[1, 0])   # bottom-left — V-I / P-I
ax_bop  = fig.add_subplot(gs[1, 1])   # bottom-right — BOP breakdown

DARK = '#0D1117'
GRID_C = '#1E2535'

def _ax_style(ax):
    ax.set_facecolor('#111827')
    ax.tick_params(colors='#BBBBBB', labelsize=9)
    for sp in ax.spines.values():
        sp.set_edgecolor('#2A3550')
    ax.grid(True, color=GRID_C, lw=0.5, ls='--')
    ax.title.set_color('white')
    ax.xaxis.label.set_color('#BBBBBB')
    ax.yaxis.label.set_color('#BBBBBB')

# ═══════════════════════════════════════════════════════════════════════════════
# TOP PANEL — Efficiency Map
# ═══════════════════════════════════════════════════════════════════════════════
_ax_style(ax_eff)

# Efficiency zone fills
if P55_asc and P55_desc:
    mask55 = (Pn >= P55_asc) & (Pn <= P55_desc)
    ax_eff.fill_between(Pn, Es, 55.0, where=mask55,
                        color='#00C853', alpha=0.12, zorder=1,
                        label='η_sys > 55 %  (premium zone)')
if P50_asc and P50_desc:
    mask50l = (Pn >= P50_asc) & (Pn < P55_asc)
    mask50r = (Pn > P55_desc) & (Pn <= P50_desc)
    for mask in [mask50l, mask50r]:
        ax_eff.fill_between(Pn, Es, 50.0, where=mask,
                            color='#FFD600', alpha=0.10, zorder=1)
    ax_eff.fill_between(Pn, 50.0, 0.0,
                        where=(Pn >= P50_asc) & (Pn <= P50_desc),
                        color='#E65100', alpha=0.07, zorder=1)

# Curves
ax_eff.plot(Pn, Eg, color='#42A5F5', lw=1.6, ls='--', alpha=0.75,
            label='η_stack gross (LHV)')
ax_eff.plot(Pn, Es, color='#00E676', lw=2.8, zorder=4,
            label='η_sys net (LHV)  — with BOP')

# Threshold lines
for thresh, col, lbl in [(55.0, '#00C853', '55 %'), (50.0, '#FFD600', '50 %')]:
    ax_eff.axhline(thresh, color=col, lw=1.0, ls=':', alpha=0.7, label=f'η = {lbl}')

# Vertical markers
marker_pts = [
    (P_PEAK,   I_PEAK,   ETA_PEAK,  '#00E676', '★ Peak η_sys',        'above', True),
    (P_STRAT,  I_STRAT,  ETA_STRAT, '#FFEB3B', '● Strategy A op.',    'below', True),
    (P_NOM,    I_nom,    float(eta_system_net(np.array([I_nom]))[0]),
                                    '#EF5350', '▲ FC rated (37.5 A)', 'above', True),
]

for P_m, I_m, eta_m, col, label, va, draw_v in marker_pts:
    ax_eff.scatter([P_m], [eta_m], color=col, s=90, zorder=8,
                   edgecolors='white', linewidths=0.8)
    if draw_v:
        ax_eff.axvline(P_m, color=col, lw=0.8, ls='--', alpha=0.45)
    offset = 1.8 if va == 'above' else -3.5
    ax_eff.annotate(
        f'{label}\n{P_m:.0f} W  |  {I_m:.1f} A  |  η={eta_m:.1f}%',
        xy=(P_m, eta_m), xytext=(P_m + 25, eta_m + offset),
        color=col, fontsize=8.5,
        arrowprops=dict(arrowstyle='->', color=col, lw=1.0),
        bbox=dict(boxstyle='round,pad=0.3', fc='#0D1117', ec=col, lw=0.7, alpha=0.9)
    )

# Strategy A operating band shading
ax_eff.axvspan(120.0, 273.0, color='#FFEB3B', alpha=0.07, zorder=2,
               label='Strategy A race band  (120–273 W)')
ax_eff.axvline(268.0, color='#FFEB3B', lw=1.4, ls='-', alpha=0.6)

# BOP crossover — where P_blower > P_pump+electronics
P_bop_fixed = P_PUMP_FIXED + P_ELECTRONICS
I_bl_dom = np.argmin(np.abs(Pbl - P_bop_fixed))
ax_eff.axvline(float(Pn[I_bl_dom]), color='#FF7043', lw=0.9, ls=':', alpha=0.5,
               label='Blower dominates BOP')

ax_eff.set_xlim(0, float(Pn.max()) * 1.03)
ax_eff.set_ylim(0, 75)
ax_eff.set_xlabel('P_net  [W]', fontsize=11, fontweight='bold')
ax_eff.set_ylabel('Efficiency  [%]', fontsize=11, fontweight='bold')
ax_eff.set_title(
    'Fuel Cell System Efficiency Map  —  balticFuelCells LC 52.30\n'
    f'Chamberlin–Kim polarization · Boreasa C65B2 blower · DC-LT2 pump · '
    f'N={N} cells · A={A} cm²',
    fontsize=11, fontweight='bold', pad=10, color='white'
)

# Secondary x-axis for current
ax_eff_I = ax_eff.twiny()
ax_eff_I.set_facecolor('#111827')
ax_eff_I.set_xlim(ax_eff.get_xlim())
I_ticks = [0, 5, 10, 15, 20, 25, 30, 37.5, 40]
P_ticks = [float(P_net(np.array([i]))[0]) if i > 0 else 0.0 for i in I_ticks]
ax_eff_I.set_xticks(P_ticks)
ax_eff_I.set_xticklabels([f'{i:.0f}' for i in I_ticks])
ax_eff_I.set_xlabel('Stack current  I_fc  [A]', color='#BBBBBB', fontsize=10)
ax_eff_I.tick_params(colors='#BBBBBB', labelsize=9)
ax_eff_I.spines['top'].set_edgecolor('#2A3550')

# Legend
leg = ax_eff.legend(loc='lower left', fontsize=8.5,
                    facecolor='#1A1A2E', edgecolor='#333355',
                    labelcolor='white', framealpha=0.95, ncol=2)

# Annotation boxes for efficiency zones
for P_c, eta_c, label_c, col_c in [
        ((P55_asc + P_PEAK) / 2,     57.5, 'η > 55 %\nPremium zone', '#00C853'),
        ((P_PEAK + P55_desc) / 2,    56.5, 'η > 55 %\nPremium zone', '#00C853'),
        ((P55_desc + P50_desc) / 2,  52.0, 'η 50–55 %\nAcceptable', '#FFD600'),
]:
    ax_eff.text(P_c, eta_c, label_c, color=col_c, fontsize=7.5,
                ha='center', va='center', style='italic',
                bbox=dict(boxstyle='round,pad=0.3', fc='#0D1117',
                          ec=col_c, lw=0.6, alpha=0.85))

# ═══════════════════════════════════════════════════════════════════════════════
# BOTTOM-LEFT — Polarization (V–I) and Power curves
# ═══════════════════════════════════════════════════════════════════════════════
_ax_style(ax_vi)
ax_vi_r = ax_vi.twinx()
ax_vi_r.set_facecolor('#111827')
ax_vi_r.tick_params(colors='#BBBBBB', labelsize=9)
ax_vi_r.spines['right'].set_edgecolor('#2A3550')

ax_vi.plot(I, Vstack,  color='#42A5F5', lw=2.2, label='V_stack')
ax_vi_r.plot(I, Pgross, color='#EF5350', lw=1.8, ls='--', alpha=0.8, label='P_gross')
ax_vi_r.plot(I, Pn,     color='#00E676', lw=2.2, label='P_net')
ax_vi_r.plot(I, Pbop,   color='#FF7043', lw=1.4, ls=':', alpha=0.9, label='P_BOP')

ax_vi.axvline(I_STRAT, color='#FFEB3B', lw=1.2, ls='--', alpha=0.7,
              label=f'Strat A  {I_STRAT:.1f} A')
ax_vi.axvline(I_nom,   color='#EF5350', lw=1.0, ls=':', alpha=0.6,
              label=f'I_nom {I_nom} A')
ax_vi.axhline(26.0, color='#42A5F5', lw=0.8, ls=':', alpha=0.5, label='V_min 26 V')

ax_vi.set_xlim(0, I_max);  ax_vi.set_ylim(0, 58)
ax_vi_r.set_ylim(0, 1200)
ax_vi.set_xlabel('Stack current  [A]', fontsize=10, fontweight='bold')
ax_vi.set_ylabel('Stack voltage  [V]', color='#42A5F5', fontsize=10)
ax_vi_r.set_ylabel('Power  [W]', color='#EF5350', fontsize=10)
ax_vi_r.yaxis.label.set_color('#EF5350')
ax_vi.set_title('Polarization & Power Curves', fontsize=10, fontweight='bold')

h1, l1 = ax_vi.get_legend_handles_labels()
h2, l2 = ax_vi_r.get_legend_handles_labels()
ax_vi.legend(h1 + h2, l1 + l2, fontsize=7.5, facecolor='#1A1A2E',
             edgecolor='#333355', labelcolor='white', loc='upper right', framealpha=0.9)

# ═══════════════════════════════════════════════════════════════════════════════
# BOTTOM-RIGHT — BOP Breakdown
# ═══════════════════════════════════════════════════════════════════════════════
_ax_style(ax_bop)
P_pump_arr = np.full_like(I, P_PUMP_FIXED)
P_elec_arr = np.full_like(I, P_ELECTRONICS)

ax_bop.stackplot(I, Pbl, P_pump_arr, P_elec_arr,
                 labels=[f'Boreasa C65B2 blower (variable speed)',
                         f'DC-LT2 pump  (fixed {P_PUMP_FIXED} W)',
                         f'Electronics / ECU  (fixed {P_ELECTRONICS} W)'],
                 colors=['#E65100', '#1565C0', '#6abf69'], alpha=0.85)
ax_bop.plot(I, Pbop, color='white', lw=2.0, zorder=5, label='Total P_BOP')
ax_bop.plot(I, Pgross * 0.05, color='#EF5350', lw=1.2, ls='--',
            alpha=0.7, label='5 % of P_gross (ref)')

ax_bop.axvline(I_STRAT, color='#FFEB3B', lw=1.2, ls='--', alpha=0.7,
               label=f'Strat A  {I_STRAT:.1f} A')
ax_bop.axvline(I_nom,   color='#EF5350', lw=1.0, ls=':', alpha=0.6,
               label=f'I_nom {I_nom} A')

# Annotate Strategy A BOP cost
P_bop_strat = float(P_BOP(np.array([I_STRAT]))[0])
ax_bop.annotate(
    f'BOP @ Strat A:\n{P_bop_strat:.1f} W  '
    f'({P_bop_strat/float(P_stack_gross(np.array([I_STRAT]))[0])*100:.1f}% of P_gross)',
    xy=(I_STRAT, P_bop_strat),
    xytext=(I_STRAT + 4, P_bop_strat + 15),
    color='#FFEB3B', fontsize=8,
    arrowprops=dict(arrowstyle='->', color='#FFEB3B', lw=1.0),
    bbox=dict(boxstyle='round,pad=0.3', fc='#0D1117', ec='#FFEB3B', lw=0.7)
)

ax_bop.set_xlim(0, I_max);  ax_bop.set_ylim(0, 60)
ax_bop.set_xlabel('Stack current  [A]', fontsize=10, fontweight='bold')
ax_bop.set_ylabel('BOP parasitic power  [W]', fontsize=10, fontweight='bold')
ax_bop.set_title('BOP Parasitic Load Breakdown', fontsize=10, fontweight='bold')
ax_bop.legend(fontsize=7.5, facecolor='#1A1A2E', edgecolor='#333355',
              labelcolor='white', loc='upper left', framealpha=0.9)

# ═══════════════════════════════════════════════════════════════════════════════
# Master title + summary box
# ═══════════════════════════════════════════════════════════════════════════════
fig.suptitle(
    'FC System Efficiency Map  ·  balticFuelCells LC 52.30  ·  Hydraix I SEM 2026',
    color='white', fontsize=14, fontweight='bold', y=0.975, fontfamily='monospace'
)

summary = (
    f'Peak η_sys = {ETA_PEAK:.1f}%  @  {P_PEAK:.0f} W  /  {I_PEAK:.1f} A     ·     '
    f'η > 55% window: {P55_asc:.0f} – {P55_desc:.0f} W     ·     '
    f'η > 50% window: {P50_asc:.0f} – {P50_desc:.0f} W\n'
    f'Strategy A operating point:  {P_STRAT:.0f} W  /  {I_STRAT:.1f} A  →  '
    f'η_sys = {ETA_STRAT:.1f}%   (−{ETA_PEAK-ETA_STRAT:.1f}% from peak)     ·     '
    f'BOP at Strat A: {P_bop_strat:.1f} W  ({P_bop_strat/float(P_stack_gross(np.array([I_STRAT]))[0])*100:.1f}% of gross)'
)
fig.text(0.50, 0.935, summary, color='#AAAAAA', fontsize=8.8,
         ha='center', va='top', style='italic',
         bbox=dict(boxstyle='round,pad=0.4', fc='#1A1A2E',
                   ec='#333355', lw=0.8, alpha=0.92))

# ── Save ──────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS_DIR, exist_ok=True)
out = os.path.join(RESULTS_DIR, 'fc_efficiency_map.png')
fig.savefig(out, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
plt.close(fig)
print(f'\nSaved → {out}')
