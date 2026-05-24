"""
Professional 3D Visualisation — Strategy A Lookup Table
Hydraix I · SEM 2026 · EMS Team

Axes
────
  X : SOC              (SC state of charge, 0.10 – 0.95)
  Y : u = Psc / Esc   (SC power normalised by SC energy, 1/s)
  Z : P_fc             (fuel-cell power setpoint, W)

Colour band : blue (0 W) → red (1 013 W = FC rated max from datasheet)

Two panels
──────────
  Left  : 3-D surface (isometric view)
  Right : top-down contour + projected operating scatter from simulation
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.cm as cm
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401  (registers 3-D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
from scipy.interpolate import RegularGridInterpolator

from ems_core import (
    simulate_race, build_demand_profile,
    FC_P_MIN, FC_P_MAX,
    SC_E_J, SC_SOC_0,
    K_H2, DT, RESULTS_DIR
)

# ── Tuned parameters (from bisection result) ──────────────────────────────────
K_SOC_TUNED = 68.8      # W / unit-SOC  (bisection result)
K_RATE      = 500.0     # W·s           (fixed design choice)
P_BASE      = 259.0     # W             (race-average demand = 151 Wh / 0.5833 h)
U_DEADBAND  = 0.020     # 1/s           (K_rate activates beyond this)

FC_P_MAX_MANUAL = 1013.0   # W  from balticFuelCells LC 52.30 datasheet

# ── Build the lookup table on a high-resolution grid for plotting ─────────────
N_SOC = 200
N_U   = 200
soc_plot = np.linspace(0.10, 0.95, N_SOC)
u_plot   = np.linspace(-0.050, 0.050, N_U)
SOC_M, U_M = np.meshgrid(soc_plot, u_plot)   # shape (N_U, N_SOC)

def lut_value(u_arr, soc_arr, K_soc):
    delta_soc  = K_soc * (0.60 - soc_arr)
    abs_u      = np.abs(u_arr)
    delta_rate = np.where(
        abs_u > U_DEADBAND,
        K_RATE * (abs_u - U_DEADBAND) * np.sign(u_arr),
        0.0
    )
    return np.clip(P_BASE + delta_soc + delta_rate, FC_P_MIN, FC_P_MAX_MANUAL)

PFC_M = lut_value(U_M, SOC_M, K_SOC_TUNED)

# ── Run simulation to get the actual operating scatter ────────────────────────
print("Running simulation to extract operating points …")

# Recreate the same strategy function as strategy_a_lut.py
u_grid_orig   = np.linspace(-0.050, 0.050, 21)
soc_grid_orig = np.linspace(0.10, 0.95, 18)
TABLE_ORIG = np.zeros((21, 18))
for i, u_i in enumerate(u_grid_orig):
    for j, soc_j in enumerate(soc_grid_orig):
        d_soc  = K_SOC_TUNED * (0.60 - soc_j)
        abs_ui = abs(u_i)
        d_rate = (K_RATE * (abs_ui - U_DEADBAND) * np.sign(u_i)
                  if abs_ui > U_DEADBAND else 0.0)
        TABLE_ORIG[i, j] = np.clip(P_BASE + d_soc + d_rate, FC_P_MIN, FC_P_MAX_MANUAL)

_interp = RegularGridInterpolator(
    (u_grid_orig, soc_grid_orig), TABLE_ORIG,
    method='linear', bounds_error=False, fill_value=None
)

def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
    u = (P_dem - P_fc_prev) / SC_E_J
    return float(_interp([[u, SOC]])[0])

res = simulate_race(strategy_fn, SOC_0=SC_SOC_0, verbose=False)
t_sim   = res['t']
P_fc_s  = res['P_fc']
P_sc_s  = res['P_sc']
SOC_s   = res['SOC']
u_s     = (res['P_dem'] - P_fc_s) / SC_E_J   # u from simulation

print(f"  Simulation done — {len(t_sim):,} timesteps, "
      f"ΔSOC={res['delta_SOC']:+.4f}, H2={res['m_H2_total']:.3f} g")

# ── Colour map: blue (0 W) → white (≈506 W) → red (1013 W) ──────────────────
cmap_br = mcolors.LinearSegmentedColormap.from_list(
    'blue_red',
    [(0.0,  '#1565C0'),   # deep blue  = 0 W
     (0.25, '#42A5F5'),   # light blue = ~250 W
     (0.50, '#FFFFFF'),   # white      = 506 W
     (0.75, '#EF5350'),   # light red  = ~760 W
     (1.00, '#B71C1C')],  # deep red   = 1013 W
    N=512
)
norm_pfc = mcolors.Normalize(vmin=0.0, vmax=FC_P_MAX_MANUAL)

# ── Figure layout ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(22, 11), dpi=150)
fig.patch.set_facecolor('#0D1117')            # dark background

gs = gridspec.GridSpec(
    1, 3,
    width_ratios=[2.2, 1.5, 0.06],
    wspace=0.06,
    left=0.04, right=0.94, top=0.91, bottom=0.08
)

ax3d  = fig.add_subplot(gs[0], projection='3d')
ax2d  = fig.add_subplot(gs[1])
ax_cb = fig.add_subplot(gs[2])   # colour-bar axis

# ── ① 3-D surface ─────────────────────────────────────────────────────────────
surf = ax3d.plot_surface(
    SOC_M, U_M, PFC_M,
    facecolors=cmap_br(norm_pfc(PFC_M)),
    rstride=2, cstride=2,
    alpha=0.92, linewidth=0, antialiased=True, shade=True
)

# Grid lines on the surface edges
ax3d.plot_wireframe(
    SOC_M, U_M, PFC_M,
    rstride=20, cstride=20,
    color='white', alpha=0.12, linewidth=0.4
)

# Operating scatter (subsample to keep plot clean)
step = max(1, len(u_s) // 3000)
sc_scatter = ax3d.scatter(
    SOC_s[::step], u_s[::step], P_fc_s[::step],
    c=P_fc_s[::step], cmap=cmap_br, norm=norm_pfc,
    s=1.5, alpha=0.65, zorder=5, linewidths=0
)

# Reference planes
soc_lim = [soc_plot[0], soc_plot[-1]]
u_lim   = [u_plot[0], u_plot[-1]]
# SOC_0 vertical plane
ax3d.plot([0.60, 0.60], u_lim, [0, 0], color='#00E676', lw=1.2, ls='--', alpha=0.7)
ax3d.plot([0.60, 0.60], [u_lim[0], u_lim[0]], [0, FC_P_MAX_MANUAL],
          color='#00E676', lw=0.8, ls=':', alpha=0.5)

# Deadband ±U lines on the floor
for u_db in [-U_DEADBAND, U_DEADBAND]:
    ax3d.plot(soc_lim, [u_db, u_db], [FC_P_MIN, FC_P_MIN],
              color='#FFD600', lw=1.0, ls=':', alpha=0.6)

# Axis labels & style
ax3d.set_xlabel('SOC  [—]', labelpad=10, color='white', fontsize=11, fontweight='bold')
ax3d.set_ylabel('u = P_sc / E_sc  [s⁻¹]', labelpad=10, color='white', fontsize=11, fontweight='bold')
ax3d.set_zlabel('P_fc  [W]', labelpad=8, color='white', fontsize=11, fontweight='bold')
ax3d.set_xlim(0.10, 0.95)
ax3d.set_ylim(-0.055, 0.055)
ax3d.set_zlim(0, FC_P_MAX_MANUAL)
ax3d.set_xticks([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
ax3d.set_yticks([-0.04, -0.02, 0, 0.02, 0.04])
ax3d.set_zticks([0, 200, 400, 600, 800, 1013])
ax3d.zaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{v:.0f}'))

ax3d.tick_params(colors='#CCCCCC', labelsize=8.5)
ax3d.xaxis.pane.fill = False
ax3d.yaxis.pane.fill = False
ax3d.zaxis.pane.fill = False
ax3d.xaxis.pane.set_edgecolor('#333333')
ax3d.yaxis.pane.set_edgecolor('#333333')
ax3d.zaxis.pane.set_edgecolor('#333333')
ax3d.grid(True, color='#2a2a2a', linewidth=0.3)

# Viewing angle: slightly elevated isometric
ax3d.view_init(elev=28, azim=-50)
ax3d.set_facecolor('#0D1117')

# Axis tick label colours
for axis in [ax3d.xaxis, ax3d.yaxis, ax3d.zaxis]:
    axis.label.set_color('white')
    [t.set_color('#BBBBBB') for t in axis.get_ticklabels()]

# 3-D annotations
ax3d.text(0.60, -0.055, FC_P_MIN - 60, 'SOC₀ = 0.60', color='#00E676',
          fontsize=8.5, ha='center', style='italic')
ax3d.text(0.10, U_DEADBAND + 0.003, FC_P_MIN - 60, '|u| = 0.020\n(deadband)',
          color='#FFD600', fontsize=7.5, ha='left')
ax3d.text(0.10, -U_DEADBAND - 0.003, FC_P_MIN - 60, '',
          color='#FFD600', fontsize=7.5, ha='left')

ax3d.set_title(
    '3-D Lookup Table Surface\n'
    r'$P_{fc}\ =\ \mathrm{clip}\ (\ P_{base}\ +\ K_{soc}\!\cdot\!(0.60\!-\!\mathrm{SOC})$'
    r'$\ +\ K_{rate}\!\cdot\!(|u|\!-\!0.02)\!\cdot\!\mathrm{sgn}(u),\ \ P_{min},\ P_{max}\ )$',
    color='white', fontsize=10, pad=12
)

# ── ② Top-down contour + operating scatter ───────────────────────────────────
ax2d.set_facecolor('#0D1117')
levels = np.linspace(0, FC_P_MAX_MANUAL, 30)
cf = ax2d.contourf(SOC_M, U_M, PFC_M, levels=levels, cmap=cmap_br, norm=norm_pfc)
ct = ax2d.contour(SOC_M, U_M, PFC_M,
                  levels=[100, 200, 259, 300, 400, 500, 600, 700, 800, 900, 1013],
                  colors='white', linewidths=0.6, alpha=0.55)
ax2d.clabel(ct, fmt='%d W', fontsize=7.5, inline=True, inline_spacing=3,
            colors='white')

# Operating scatter on 2-D
ax2d.scatter(SOC_s[::step], u_s[::step],
             c='white', s=0.4, alpha=0.25, linewidths=0, zorder=5)

# Reference lines
ax2d.axvline(0.60, color='#00E676', lw=1.2, ls='--', label='SOC₀ = 0.60')
ax2d.axhline(U_DEADBAND,  color='#FFD600', lw=0.9, ls=':', alpha=0.8)
ax2d.axhline(-U_DEADBAND, color='#FFD600', lw=0.9, ls=':', alpha=0.8,
             label='|u| deadband ±0.02 s⁻¹')
ax2d.axhline(0, color='#AAAAAA', lw=0.6, ls='-', alpha=0.3)

# Operating region box (actual scatter bounds)
u_lo, u_hi   = np.percentile(u_s, 0.5), np.percentile(u_s, 99.5)
soc_lo, soc_hi = np.percentile(SOC_s, 0.5), np.percentile(SOC_s, 99.5)
rect = plt.Rectangle(
    (soc_lo, u_lo), soc_hi - soc_lo, u_hi - u_lo,
    linewidth=1.4, edgecolor='#76FF03', facecolor='none',
    linestyle='-', zorder=8, label=f'99% operating\nregion'
)
ax2d.add_patch(rect)
ax2d.text(soc_hi + 0.005, (u_lo + u_hi) / 2,
          f'SOC: {soc_lo:.3f}–{soc_hi:.3f}\n'
          f'u:   {u_lo:.3f}–{u_hi:.3f} s⁻¹\n'
          f'P_fc: {res["P_fc"].min():.0f}–{res["P_fc"].max():.0f} W',
          color='#76FF03', fontsize=7.5, va='center',
          bbox=dict(boxstyle='round,pad=0.3', facecolor='#0D1117', edgecolor='#76FF03', lw=0.8))

ax2d.set_xlim(0.10, 0.95)
ax2d.set_ylim(-0.055, 0.055)
ax2d.set_xlabel('SOC  [—]', color='white', fontsize=11, fontweight='bold')
ax2d.set_ylabel('u = P_sc / E_sc  [s⁻¹]', color='white', fontsize=11, fontweight='bold')
ax2d.set_title('Top-Down Contour  +  Simulation Operating Points\n(white scatter = actual race trajectory)',
               color='white', fontsize=10)
ax2d.tick_params(colors='#CCCCCC', labelsize=9)
for spine in ax2d.spines.values():
    spine.set_edgecolor('#333333')

leg = ax2d.legend(loc='lower right', fontsize=8.5, facecolor='#1A1A2E',
                  edgecolor='#444444', labelcolor='white', framealpha=0.85)

# ── Shared colour bar ─────────────────────────────────────────────────────────
sm = cm.ScalarMappable(cmap=cmap_br, norm=norm_pfc)
sm.set_array([])
cb = fig.colorbar(sm, cax=ax_cb)
cb.set_label('P_fc  [W]', color='white', fontsize=11, fontweight='bold', labelpad=10)
cb.set_ticks([0, 100, 259, 500, 700, 900, 1013])
cb.set_ticklabels(['0\n(off)', '100\n(min)', '259\n(base)', '500', '700', '900', '1013\n(max)'])
cb.ax.tick_params(colors='white', labelsize=8.5)
cb.ax.yaxis.set_tick_params(length=4)
cb.outline.set_edgecolor('#555555')
# Horizontal tick marks at key values
for val, label, col in [(100, '', '#4FC3F7'), (259, '', '#FFFFFF'), (1013, '', '#EF5350')]:
    cb.ax.axhline(norm_pfc(val), color=col, lw=2.0, alpha=0.7)

# ── Master title + parameter box ─────────────────────────────────────────────
fig.suptitle(
    'Strategy A — 2D Heuristic Lookup Table  ·  Hydraix I SEM 2026',
    color='white', fontsize=15, fontweight='bold', y=0.97,
    fontfamily='monospace'
)

param_text = (
    f'Tuned parameters:  K_soc = {K_SOC_TUNED} W/SOC   '
    f'K_rate = {K_RATE:.0f} W·s   '
    f'P_base = {P_BASE:.0f} W\n'
    f'Table: 21 u-pts × 18 SOC-pts   bilinear interpolation   '
    f'Race result: H₂ = {res["m_H2_total"]:.3f} g   ΔSOC = {res["delta_SOC"]:+.4f}'
)
fig.text(0.50, 0.925, param_text, color='#AAAAAA', fontsize=9,
         ha='center', va='top', style='italic',
         bbox=dict(boxstyle='round,pad=0.4', facecolor='#1A1A2E',
                   edgecolor='#333355', lw=0.8, alpha=0.9))

# ── Save ──────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS_DIR, exist_ok=True)
out_path = os.path.join(RESULTS_DIR, 'strategy_a_lut_3d.png')
fig.savefig(out_path, dpi=180, bbox_inches='tight',
            facecolor=fig.get_facecolor())
plt.close(fig)
print(f"Saved → {out_path}")
