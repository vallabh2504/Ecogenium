"""
Professional 3D Lookup Table Visualiser — Strategy A (Corrected Scaling)
Hydraix I · SEM 2026 · EMS Team

Root cause of 'flat' appearance in v2:
  Table output spans only 220–308 W (88 W range) on a 0–1013 W Z-axis
  → variation is 8.7 % of axis height → invisible.

Fixes applied here:
  1. Z-axis zoomed to actual table range [FC_P_MIN, table_max+20] = [100, 330 W]
  2. Colormap vmax set to the table maximum (~308 W) so the full blue→green→red
     gradient is used across the surface, not squeezed into the bottom 30 %.
  3. Contour lines projected onto the base plane so the tilt is readable.

Colormap anchors (same physical meaning as v2, remapped to table range):
  Blue   →  0 W        (FC off — outside operating range, reference)
  Green  →  120.22 W   (peak system efficiency 59.0 % from FuelCellEstimate_v3)
  Red    →  308 W      (table maximum — extreme low-SOC + hard SC discharge)

Physical FC rated maximum (1013 W from manual) is annotated separately.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "FCS Strategy"))

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import matplotlib.patches as mpatches
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D           # noqa: F401
from scipy.interpolate import RegularGridInterpolator

from ems_core import (
    simulate_race, FC_P_MIN, FC_P_MAX,
    SC_E_J, SC_SOC_0, DT, RESULTS_DIR
)

# ── FC model v3 — peak efficiency anchor ─────────────────────────────────────
try:
    from FuelCellEstimate_v3 import P_net as fc_pnet, eta_system_net, I_max as FC_I_MAX
    _I  = np.linspace(0.5, FC_I_MAX, 10_000)
    _pk = int(np.argmax(eta_system_net(_I)))
    P_PEAK_EFF = float(fc_pnet(_I)[_pk])
    ETA_PEAK   = float(eta_system_net(_I)[_pk])
    _HAVE_FC   = True
    print(f"FC model — peak η_sys = {ETA_PEAK:.2f}%  @ {P_PEAK_EFF:.2f} W")
except Exception as e:
    P_PEAK_EFF, ETA_PEAK, _HAVE_FC = 120.22, 59.0, False
    print(f"[warn] FC model import failed ({e}); using defaults.")

# ── Strategy A tuned parameters ───────────────────────────────────────────────
K_SOC       = 68.8
K_RATE      = 500.0
P_BASE      = 259.0
U_DEADBAND  = 0.020

# ── Build high-res table ──────────────────────────────────────────────────────
N_SOC, N_U  = 300, 300
soc_plot    = np.linspace(0.10, 0.95, N_SOC)
u_plot      = np.linspace(-0.050, 0.050, N_U)
SOC_M, U_M  = np.meshgrid(soc_plot, u_plot)

def lut(u_a, soc_a):
    d_soc  = K_SOC * (0.60 - soc_a)
    abs_u  = np.abs(u_a)
    d_rate = np.where(abs_u > U_DEADBAND,
                      K_RATE*(abs_u - U_DEADBAND)*np.sign(u_a), 0.0)
    return np.clip(P_BASE + d_soc + d_rate, FC_P_MIN, FC_P_MAX)

PFC_M = lut(U_M, SOC_M)
TABLE_MIN = float(PFC_M.min())
TABLE_MAX = float(PFC_M.max())
print(f"Table range: {TABLE_MIN:.1f} – {TABLE_MAX:.1f} W  (span = {TABLE_MAX-TABLE_MIN:.1f} W)")

# ── Colormap: Blue → Green → Red, scaled to TABLE range ──────────────────────
# vmin = 0 W (FC off, physical reference)
# vmax = TABLE_MAX (max the strategy can actually output)
# Green anchor placed at P_PEAK_EFF / TABLE_MAX in normalised space
CMAP_VMIN = 0.0
CMAP_VMAX = TABLE_MAX          # ← KEY CHANGE from v2 (was 1013 W)

f_pk   = P_PEAK_EFF / CMAP_VMAX   # green anchor position in [0,1]
f_base = P_BASE      / CMAP_VMAX  # P_base reference line position

CMAP = mcolors.LinearSegmentedColormap.from_list(
    'bGr_zoomed',
    [
        (0.000,             '#0D47A1'),   # deep blue  — 0 W
        (f_pk * 0.50,       '#1565C0'),   # mid blue   — ramp up
        (f_pk,              '#00C853'),   # vivid green — peak η_sys
        (f_pk + (1.0-f_pk)*0.25, '#FFD600'),  # yellow
        (f_pk + (1.0-f_pk)*0.60, '#FF6D00'),  # orange
        (1.000,             '#B71C1C'),   # deep red   — table max
    ],
    N=1024
)
norm = mcolors.Normalize(vmin=CMAP_VMIN, vmax=CMAP_VMAX)

# ── Simulate to get race operating scatter ────────────────────────────────────
print("Running simulation …")
u_g   = np.linspace(-0.050, 0.050, 21)
soc_g = np.linspace(0.10, 0.95, 18)
T_orig = np.array([[lut(u_i, soc_j) for soc_j in soc_g] for u_i in u_g])
_itp   = RegularGridInterpolator((u_g, soc_g), T_orig,
                                  method='linear', bounds_error=False, fill_value=None)

def strat(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
    u = (P_dem - P_fc_prev) / SC_E_J
    return float(_itp([[u, SOC]])[0])

res    = simulate_race(strat, SOC_0=SC_SOC_0, verbose=False)
P_fc_s = res['P_fc']
u_s    = (res['P_dem'] - P_fc_s) / SC_E_J
SOC_s  = res['SOC']
P_fc_sim_min = float(P_fc_s.min())
P_fc_sim_max = float(P_fc_s.max())
print(f"  Simulation P_fc range: {P_fc_sim_min:.1f}–{P_fc_sim_max:.1f} W  "
      f"ΔSOC={res['delta_SOC']:+.4f}  H2={res['m_H2_total']:.3f} g")

# ── Figure ────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(24, 12), dpi=160)
fig.patch.set_facecolor('#0D1117')

gs = gridspec.GridSpec(1, 3, width_ratios=[2.3, 1.7, 0.055],
                       wspace=0.04, left=0.03, right=0.96,
                       top=0.89, bottom=0.07)
ax3d  = fig.add_subplot(gs[0], projection='3d')
ax2d  = fig.add_subplot(gs[1])
ax_cb = fig.add_subplot(gs[2])

# ── Z-axis limits — zoomed to actual table range ──────────────────────────────
Z_LO = FC_P_MIN - 10     # 90 W  (just below FC_P_MIN)
Z_HI = TABLE_MAX + 22    # ~332 W

# ── ① 3-D Surface ─────────────────────────────────────────────────────────────
face_c = CMAP(norm(PFC_M))
ax3d.plot_surface(SOC_M, U_M, PFC_M,
                  facecolors=face_c, rstride=2, cstride=2,
                  alpha=0.93, linewidth=0, antialiased=True, shade=True)

# Wire mesh (denser — shape is now visible)
ax3d.plot_wireframe(SOC_M, U_M, PFC_M,
                    rstride=15, cstride=15,
                    color='white', alpha=0.12, linewidth=0.4)

# --- Contour lines projected on the base plane (Z = Z_LO) ---
contour_levels = np.linspace(TABLE_MIN, TABLE_MAX, 10)
ax3d.contour(SOC_M, U_M, PFC_M,
             levels=contour_levels,
             zdir='z', offset=Z_LO,
             cmap=CMAP, norm=norm, linewidths=1.0, alpha=0.55)

# --- Horizontal reference planes ---
for P_lvl, col, ls in [
        (P_PEAK_EFF, '#00C853', '--'),
        (P_BASE,     '#FFEB3B', ':'),
        (TABLE_MAX,  '#EF5350', '--'),
]:
    # Edge lines at this power level
    ax3d.plot([soc_plot[0], soc_plot[-1]], [u_plot[0],  u_plot[0]],  [P_lvl, P_lvl],
              color=col, lw=1.0, ls=ls, alpha=0.65)
    ax3d.plot([soc_plot[0], soc_plot[0]],  [u_plot[0],  u_plot[-1]], [P_lvl, P_lvl],
              color=col, lw=1.0, ls=ls, alpha=0.45)

# --- Operating scatter ---
step = max(1, len(u_s) // 2500)
ax3d.scatter(SOC_s[::step], u_s[::step], P_fc_s[::step],
             c=P_fc_s[::step], cmap=CMAP, norm=norm,
             s=6, alpha=0.90, zorder=10, linewidths=0, depthshade=False)

# --- Z label annotation showing the zoom ---
ax3d.text2D(0.01, 0.5,
            '⬆ Z-axis zoomed to operating range\n'
            f'   [{FC_P_MIN:.0f} – {TABLE_MAX:.0f} W]\n'
            f'   FC rated max = 1013 W (off-chart)',
            transform=ax3d.transAxes, color='#AAAAAA',
            fontsize=8, va='center', rotation=90)

# --- Axis styling ---
ax3d.set_xlabel('SOC  [—]',               labelpad=10, color='white', fontsize=11, fontweight='bold')
ax3d.set_ylabel('u = P_sc / E_sc  [s⁻¹]', labelpad=12, color='white', fontsize=11, fontweight='bold')
ax3d.set_zlabel('P_fc  [W]',               labelpad=8,  color='white', fontsize=11, fontweight='bold')
ax3d.set_xlim(0.10, 0.95)
ax3d.set_ylim(-0.055, 0.055)
ax3d.set_zlim(Z_LO, Z_HI)
ax3d.set_xticks([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
ax3d.set_yticks([-0.04, -0.02, 0, 0.02, 0.04])

z_ticks = sorted(set([int(round(v)) for v in
    [FC_P_MIN, P_PEAK_EFF, 180, 220, P_BASE, 280, TABLE_MAX]]))
ax3d.set_zticks(z_ticks)
ax3d.zaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))

for axis in [ax3d.xaxis, ax3d.yaxis, ax3d.zaxis]:
    axis.label.set_color('white')
    [t.set_color('#BBBBBB') for t in axis.get_ticklabels()]
ax3d.xaxis.pane.fill = False; ax3d.xaxis.pane.set_edgecolor('#2A2A2A')
ax3d.yaxis.pane.fill = False; ax3d.yaxis.pane.set_edgecolor('#2A2A2A')
ax3d.zaxis.pane.fill = False; ax3d.zaxis.pane.set_edgecolor('#2A2A2A')
ax3d.grid(True, color='#1E1E1E', linewidth=0.25)
ax3d.view_init(elev=28, azim=-50)
ax3d.set_facecolor('#0D1117')

# Z-level side labels
for P_lvl, col, lbl in [
        (P_PEAK_EFF, '#00C853', f'{P_PEAK_EFF:.0f} W  peak η'),
        (P_BASE,     '#FFEB3B', f'{P_BASE:.0f} W  base'),
        (TABLE_MAX,  '#EF5350', f'{TABLE_MAX:.0f} W  table max'),
]:
    ax3d.text(0.10, -0.060, P_lvl, lbl,
              color=col, fontsize=8, ha='left', va='center')

ax3d.set_title(
    '3-D Lookup Table  —  P_fc(SOC, u)\n'
    r'$P_{fc} = \mathrm{clip}\!\left('
    r'259 + 68.8\cdot(0.60-\mathrm{SOC}) + 500\cdot(|u|-0.02)\cdot\mathrm{sgn}(u),\;'
    r'100,\;1013\right)$',
    color='white', fontsize=9.5, pad=14
)

# ── ② Top-down contour ────────────────────────────────────────────────────────
ax2d.set_facecolor('#0D1117')

levels_fill = np.linspace(CMAP_VMIN, CMAP_VMAX, 40)
cf = ax2d.contourf(SOC_M, U_M, PFC_M, levels=levels_fill, cmap=CMAP, norm=norm)

ct_vals = sorted({FC_P_MIN, P_PEAK_EFF, 180, 220, P_BASE, 280, TABLE_MAX})
ct = ax2d.contour(SOC_M, U_M, PFC_M, levels=ct_vals,
                  colors='white', linewidths=0.7, alpha=0.50)
ax2d.clabel(ct, fmt=lambda v: f'{v:.0f} W', fontsize=7.5, inline=True,
            inline_spacing=3, colors='white')

# Operating scatter
ax2d.scatter(SOC_s[::step], u_s[::step],
             c=P_fc_s[::step], cmap=CMAP, norm=norm,
             s=2.5, alpha=0.60, linewidths=0, zorder=6)

# Operating region box
soc_lo, soc_hi = np.percentile(SOC_s, 0.5),  np.percentile(SOC_s, 99.5)
u_lo,   u_hi   = np.percentile(u_s,   0.5),  np.percentile(u_s,   99.5)
rect = plt.Rectangle(
    (soc_lo, u_lo), soc_hi-soc_lo, u_hi-u_lo,
    lw=1.8, edgecolor='#FFEB3B', facecolor='none', ls='-', zorder=9
)
ax2d.add_patch(rect)
ax2d.text(soc_hi + 0.004, (u_lo+u_hi)/2,
          f'Race band\nSOC {soc_lo:.3f}–{soc_hi:.3f}\n'
          f'P_fc {P_fc_sim_min:.0f}–{P_fc_sim_max:.0f} W\n'
          f'(only {P_fc_sim_max-P_fc_sim_min:.0f} W swing\nacross entire race)',
          color='#FFEB3B', fontsize=7.8, va='center',
          bbox=dict(boxstyle='round,pad=0.35', fc='#0D1117', ec='#FFEB3B', lw=0.9))

# Reference lines
ax2d.axvline(0.60, color='#80DEEA', lw=1.0, ls='--', alpha=0.8, label='SOC₀ = 0.60')
ax2d.axhline( U_DEADBAND, color='#FFD600', lw=0.8, ls=':', alpha=0.7,
              label=f'|u| deadband ±{U_DEADBAND} s⁻¹')
ax2d.axhline(-U_DEADBAND, color='#FFD600', lw=0.8, ls=':', alpha=0.7)

# Gradient arrows explaining the two input effects
ax2d.annotate('', xy=(0.15, 0.038), xytext=(0.15, -0.038),
    arrowprops=dict(arrowstyle='<->', color='#00C853', lw=1.4))
ax2d.text(0.13, 0.0, 'K_rate\n±15 W', color='#00C853',
          fontsize=7.5, ha='right', va='center')

ax2d.annotate('', xy=(0.12, -0.048), xytext=(0.88, -0.048),
    arrowprops=dict(arrowstyle='<->', color='#EF5350', lw=1.4))
ax2d.text(0.50, -0.052, 'K_soc  ±29 W', color='#EF5350',
          fontsize=7.5, ha='center', va='top')

ax2d.set_xlim(0.10, 0.95)
ax2d.set_ylim(-0.058, 0.058)
ax2d.set_xlabel('SOC  [—]',               color='white', fontsize=11, fontweight='bold')
ax2d.set_ylabel('u = P_sc / E_sc  [s⁻¹]', color='white', fontsize=11, fontweight='bold')
ax2d.set_title('Top-Down Contour  +  Race Scatter\n'
               f'(colormap spans 0 – {TABLE_MAX:.0f} W  ·  '
               f'FC rated max 1013 W off-chart)',
               color='white', fontsize=9.5)
ax2d.tick_params(colors='#CCCCCC', labelsize=9)
for sp in ax2d.spines.values(): sp.set_edgecolor('#2A2A2A')
ax2d.legend(loc='lower right', fontsize=8.5, facecolor='#1A1A2E',
            edgecolor='#444', labelcolor='white', framealpha=0.9)

# --- FC efficiency inset ---
if _HAVE_FC:
    ax_ins = ax2d.inset_axes([0.01, 0.59, 0.44, 0.39])
    ax_ins.set_facecolor('#111827')
    _Pn = fc_pnet(_I)
    _Es = eta_system_net(_I)
    ax_ins.plot(_Pn, _Es, color='#69F0AE', lw=1.8, zorder=3)
    ax_ins.fill_between(_Pn, _Es, alpha=0.12, color='#69F0AE')
    # Anchor dots
    for P_a, col_a in [(0.0, '#1E88E5'), (P_PEAK_EFF, '#00C853'), (TABLE_MAX, '#EF5350')]:
        idx_a = np.argmin(np.abs(_Pn - max(P_a, 0.5)))
        ax_ins.scatter([_Pn[idx_a]], [_Es[idx_a]], color=col_a,
                       s=45, zorder=5, ec='white', lw=0.7)
    # Show where race operates
    ax_ins.axvspan(P_fc_sim_min, P_fc_sim_max, alpha=0.20, color='#FFEB3B',
                   label=f'Race {P_fc_sim_min:.0f}–{P_fc_sim_max:.0f} W')
    ax_ins.axvline(P_PEAK_EFF, color='#00C853', lw=0.9, ls='--', alpha=0.7)
    ax_ins.set_xlim(0, 350)
    ax_ins.set_ylim(35, 65)
    ax_ins.set_xlabel('P_net [W]', color='#AAAAAA', fontsize=7)
    ax_ins.set_ylabel('η_sys [%]', color='#AAAAAA', fontsize=7)
    ax_ins.set_title('FC System η (v3)', color='#AAAAAA', fontsize=7.5)
    ax_ins.tick_params(colors='#888888', labelsize=6.5)
    for sp in ax_ins.spines.values(): sp.set_edgecolor('#333333')
    ax_ins.grid(True, color='#1E293B', lw=0.3)
    ax_ins.legend(fontsize=6.5, facecolor='#111827', edgecolor='#333', labelcolor='white')

# ── Colour bar ────────────────────────────────────────────────────────────────
sm = cm.ScalarMappable(cmap=CMAP, norm=norm)
sm.set_array([])
cb = fig.colorbar(sm, cax=ax_cb)
cb.set_label('P_fc  [W]', color='white', fontsize=11, fontweight='bold', labelpad=10)
cb.set_ticks([0, P_PEAK_EFF, P_BASE, P_fc_sim_max, 280, TABLE_MAX])
cb.set_ticklabels([
    f'0\n(off)',
    f'{P_PEAK_EFF:.0f}\n(peak η\n{ETA_PEAK:.0f}%)',
    f'{P_BASE:.0f}\n(base)',
    f'{P_fc_sim_max:.0f}\n(sim max)',
    '280',
    f'{TABLE_MAX:.0f}\n(table max)'
])
cb.ax.tick_params(colors='white', labelsize=8)
cb.outline.set_edgecolor('#444444')
for v_m, c_m in [(0.0, '#1E88E5'), (P_PEAK_EFF, '#00C853'), (TABLE_MAX, '#EF5350')]:
    cb.ax.axhline(norm(max(v_m, 0.01)), color=c_m, lw=2.5, alpha=0.9)

# ── Legend patches ────────────────────────────────────────────────────────────
fig.legend(
    handles=[
        mpatches.Patch(color='#1E88E5', label='0 W — FC off (never commanded)'),
        mpatches.Patch(color='#00C853', label=f'{P_PEAK_EFF:.0f} W — peak η_sys '
                                               f'({ETA_PEAK:.1f}%) from FC model v3'),
        mpatches.Patch(color='#B71C1C', label=f'{TABLE_MAX:.0f} W — table maximum '
                                               f'(low SOC + hard SC discharge)'),
        mpatches.Patch(color='#FFEB3B', label=f'Race band: {P_fc_sim_min:.0f}–'
                                               f'{P_fc_sim_max:.0f} W  '
                                               f'(FC rated 1013 W is far off-chart)'),
    ],
    loc='lower center', ncol=2, fontsize=9,
    facecolor='#1A1A2E', edgecolor='#444', labelcolor='white',
    framealpha=0.92, bbox_to_anchor=(0.50, 0.005)
)

# ── Master title ──────────────────────────────────────────────────────────────
fig.suptitle(
    'Strategy A — 2D Lookup Table   ·   Hydraix I SEM 2026   ·   Z-axis zoomed to operating range',
    color='white', fontsize=14, fontweight='bold', y=0.975, fontfamily='monospace'
)
fig.text(
    0.50, 0.934,
    f'Table spans {TABLE_MIN:.0f}–{TABLE_MAX:.0f} W  (88 W range)  out of 0–1013 W physical FC envelope  ·  '
    f'Race uses only {P_fc_sim_min:.0f}–{P_fc_sim_max:.0f} W  ({P_fc_sim_max-P_fc_sim_min:.0f} W swing)  ·  '
    f'K_soc={K_SOC} W/SOC  K_rate={K_RATE:.0f} W·s  P_base={P_BASE:.0f} W',
    color='#AAAAAA', fontsize=8.5, ha='center', va='top', style='italic',
    bbox=dict(boxstyle='round,pad=0.4', fc='#1A1A2E', ec='#333355', lw=0.8, alpha=0.9)
)

# ── Save ──────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS_DIR, exist_ok=True)
out = os.path.join(RESULTS_DIR, 'strategy_a_lut_3d_v3.png')
fig.savefig(out, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
plt.close(fig)
print(f"\nSaved → {out}")
