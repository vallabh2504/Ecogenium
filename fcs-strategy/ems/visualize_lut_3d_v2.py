"""
Professional 3D Lookup Table Visualiser — Strategy A (Revised Colormap)
Hydraix I · SEM 2026 · EMS Team

Colormap anchors — derived from physical operating points:
  Blue   →  0 W          (FC fully off)
  Green  →  120.22 W     (peak system efficiency 59.0 % from FuelCellEstimate_v3.py)
  Red    →  1 013 W      (FC rated maximum from balticFuelCells LC 52.30 manual)

Layout:
  Left   — 3-D isometric surface
  Right  — top-down contour + operating scatter
  Inset  — FC efficiency curve with the three anchor marks
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
from matplotlib.ticker import MultipleLocator
from scipy.interpolate import RegularGridInterpolator

# ── Import FC model v3 to compute peak-efficiency anchor ─────────────────────
try:
    from FuelCellEstimate_v3 import (
        P_net as fc_pnet, eta_system_net, I_max as FC_I_MAX
    )
    _I_sweep = np.linspace(0.5, FC_I_MAX, 10_000)
    _Eff     = eta_system_net(_I_sweep)
    _Pnet    = fc_pnet(_I_sweep)
    _pk      = int(np.argmax(_Eff))
    P_PEAK_EFF    = float(_Pnet[_pk])    # W at peak system efficiency
    ETA_PEAK      = float(_Eff[_pk])     # % peak system efficiency
    _HAVE_FC = True
    print(f"FC model loaded — peak η_sys = {ETA_PEAK:.2f}%  @ P_net = {P_PEAK_EFF:.2f} W")
except Exception as e:
    print(f"[warn] FC model import failed ({e}); using hardcoded value.")
    P_PEAK_EFF = 120.22
    ETA_PEAK   = 59.0
    _HAVE_FC   = False

# ── EMS constants (from ems_core) ─────────────────────────────────────────────
from ems_core import (
    simulate_race, FC_P_MIN, FC_P_MAX,
    SC_E_J, SC_SOC_0, K_H2, DT, RESULTS_DIR
)

FC_P_MAX_MANUAL = 1013.0          # W from datasheet — red anchor

# ── Tuned Strategy A parameters ───────────────────────────────────────────────
K_SOC_TUNED = 68.8
K_RATE      = 500.0
P_BASE      = 259.0
U_DEADBAND  = 0.020

# ── Build high-res table for plotting ─────────────────────────────────────────
N_SOC = 250
N_U   = 250
soc_plot = np.linspace(0.10, 0.95, N_SOC)
u_plot   = np.linspace(-0.050, 0.050, N_U)
SOC_M, U_M = np.meshgrid(soc_plot, u_plot)

def lut_value(u_arr, soc_arr, K_soc):
    d_soc  = K_soc * (0.60 - soc_arr)
    abs_u  = np.abs(u_arr)
    d_rate = np.where(abs_u > U_DEADBAND,
                      K_RATE * (abs_u - U_DEADBAND) * np.sign(u_arr), 0.0)
    return np.clip(P_BASE + d_soc + d_rate, FC_P_MIN, FC_P_MAX_MANUAL)

PFC_M = lut_value(U_M, SOC_M, K_SOC_TUNED)

# ── Simulate to get actual operating scatter ───────────────────────────────────
print("Running simulation …")
u_grid_orig   = np.linspace(-0.050, 0.050, 21)
soc_grid_orig = np.linspace(0.10, 0.95, 18)
TABLE_ORIG = np.zeros((21, 18))
for i, u_i in enumerate(u_grid_orig):
    for j, soc_j in enumerate(soc_grid_orig):
        abs_ui = abs(u_i)
        d_r    = K_RATE*(abs_ui-U_DEADBAND)*np.sign(u_i) if abs_ui > U_DEADBAND else 0.0
        TABLE_ORIG[i, j] = np.clip(
            P_BASE + K_SOC_TUNED*(0.60 - soc_j) + d_r, FC_P_MIN, FC_P_MAX_MANUAL)

_interp = RegularGridInterpolator(
    (u_grid_orig, soc_grid_orig), TABLE_ORIG,
    method='linear', bounds_error=False, fill_value=None)

def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
    u = (P_dem - P_fc_prev) / SC_E_J
    return float(_interp([[u, SOC]])[0])

res       = simulate_race(strategy_fn, SOC_0=SC_SOC_0, verbose=False)
P_fc_s    = res['P_fc']
u_s       = (res['P_dem'] - P_fc_s) / SC_E_J
SOC_s     = res['SOC']
P_fc_max_sim = float(P_fc_s.max())
P_fc_min_sim = float(P_fc_s.min())
print(f"  Done — ΔSOC={res['delta_SOC']:+.4f}  H2={res['m_H2_total']:.3f} g")
print(f"  Simulation P_fc range: {P_fc_min_sim:.1f} W – {P_fc_max_sim:.1f} W")

# ── Three-anchor colormap: Blue → Green → Red ─────────────────────────────────
# Normalise anchor positions to [0, 1] on the [0, FC_P_MAX_MANUAL] scale.
f_peak = P_PEAK_EFF / FC_P_MAX_MANUAL   # ≈ 0.119

CMAP_3 = mcolors.LinearSegmentedColormap.from_list(
    'bGr_physics',
    [
        (0.000,         '#0D47A1'),   # deep blue   — 0 W (FC off)
        (f_peak * 0.50, '#1565C0'),   # mid blue    — transition
        (f_peak,        '#00C853'),   # vivid green — peak η_sys
        (f_peak * 1.60, '#FFD600'),   # yellow      — between peak & max
        (0.65,          '#FF6D00'),   # deep orange
        (1.000,         '#B71C1C'),   # deep red    — 1013 W (FC max)
    ],
    N=1024
)
norm_pfc = mcolors.Normalize(vmin=0.0, vmax=FC_P_MAX_MANUAL)

# ── Figure layout ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(24, 12), dpi=160)
fig.patch.set_facecolor('#0D1117')

gs_outer = gridspec.GridSpec(
    1, 3,
    width_ratios=[2.4, 1.6, 0.055],
    wspace=0.05,
    left=0.03, right=0.96, top=0.90, bottom=0.07
)

ax3d  = fig.add_subplot(gs_outer[0], projection='3d')
ax2d  = fig.add_subplot(gs_outer[1])
ax_cb = fig.add_subplot(gs_outer[2])

# ── ① 3-D surface ─────────────────────────────────────────────────────────────
face_colors = CMAP_3(norm_pfc(PFC_M))
surf = ax3d.plot_surface(
    SOC_M, U_M, PFC_M,
    facecolors=face_colors,
    rstride=2, cstride=2,
    alpha=0.90, linewidth=0, antialiased=True, shade=True
)

# Subtle wire mesh
ax3d.plot_wireframe(
    SOC_M, U_M, PFC_M,
    rstride=25, cstride=25,
    color='white', alpha=0.08, linewidth=0.35
)

# --- Horizontal iso-power planes at the three anchor levels ---
for P_lvl, col, lbl in [
        (0.0,            '#1E88E5', 'P = 0 W  (off)'),
        (P_PEAK_EFF,     '#00C853', f'P = {P_PEAK_EFF:.0f} W  (peak η)'),
        (FC_P_MAX_MANUAL,'#B71C1C', f'P = {FC_P_MAX_MANUAL:.0f} W  (FC max)'),
]:
    # Floor projection line at P_lvl on the surface edges
    ax3d.plot(
        [soc_plot[0], soc_plot[-1]], [u_plot[0], u_plot[0]], [P_lvl, P_lvl],
        color=col, lw=1.1, ls='--', alpha=0.55
    )
    ax3d.plot(
        [soc_plot[0], soc_plot[0]], [u_plot[0], u_plot[-1]], [P_lvl, P_lvl],
        color=col, lw=1.1, ls='--', alpha=0.55
    )

# Vertical reference at SOC_0
ax3d.plot([0.60, 0.60], [u_plot[0], u_plot[-1]], [0, 0],
          color='#80DEEA', lw=1.0, ls=':', alpha=0.6)

# Operating scatter
step = max(1, len(u_s) // 2500)
ax3d.scatter(
    SOC_s[::step], u_s[::step], P_fc_s[::step],
    c=P_fc_s[::step], cmap=CMAP_3, norm=norm_pfc,
    s=4, alpha=0.85, zorder=10, linewidths=0,
    depthshade=False
)

# ---- 3-D axis style ----
ax3d.set_xlabel('SOC  [—]',               labelpad=10, color='white', fontsize=11, fontweight='bold')
ax3d.set_ylabel('u = P_sc / E_sc  [s⁻¹]', labelpad=12, color='white', fontsize=11, fontweight='bold')
ax3d.set_zlabel('P_fc  [W]',               labelpad=8,  color='white', fontsize=11, fontweight='bold')
ax3d.set_xlim(0.10, 0.95)
ax3d.set_ylim(-0.055, 0.055)
ax3d.set_zlim(0, FC_P_MAX_MANUAL)
ax3d.set_xticks([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
ax3d.set_yticks([-0.04, -0.02, 0, 0.02, 0.04])
ax3d.set_zticks([0, 120, 259, 500, 700, 900, 1013])
ax3d.zaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))

for axis in [ax3d.xaxis, ax3d.yaxis, ax3d.zaxis]:
    axis.label.set_color('white')
    [t.set_color('#BBBBBB') for t in axis.get_ticklabels()]

ax3d.xaxis.pane.fill = False;  ax3d.xaxis.pane.set_edgecolor('#2A2A2A')
ax3d.yaxis.pane.fill = False;  ax3d.yaxis.pane.set_edgecolor('#2A2A2A')
ax3d.zaxis.pane.fill = False;  ax3d.zaxis.pane.set_edgecolor('#2A2A2A')
ax3d.grid(True, color='#1E1E1E', linewidth=0.25)
ax3d.view_init(elev=26, azim=-48)
ax3d.set_facecolor('#0D1117')

# Z-level annotations on the back wall
ax3d.text(0.95, 0.055, 0,            f'0 W',           color='#1E88E5', fontsize=8.5)
ax3d.text(0.95, 0.055, P_PEAK_EFF,   f'{P_PEAK_EFF:.0f} W',  color='#00C853', fontsize=8.5)
ax3d.text(0.95, 0.055, FC_P_MAX_MANUAL, '1013 W',      color='#EF5350', fontsize=8.5)

# Operating region annotation
ax3d.text(0.62, -0.055, P_fc_min_sim - 70,
          f'Race band\n{P_fc_min_sim:.0f}–{P_fc_max_sim:.0f} W',
          color='#FFEB3B', fontsize=8, ha='left',
          bbox=dict(boxstyle='round,pad=0.3', fc='#1A1A2E', ec='#FFEB3B', lw=0.8, alpha=0.85))

ax3d.set_title(
    '3-D Lookup Table  —  P_fc(SOC, u)\n'
    r'$P_{fc} = \mathrm{clip}\!\left('
    r'259 + K_{soc}(0.60 - \mathrm{SOC}) + K_{rate}(|u| - 0.02)\,\mathrm{sgn}(u),\;'
    r'P_{min},\;P_{max}\right)$',
    color='white', fontsize=9.5, pad=14
)

# ── ② Top-down contour + scatter ─────────────────────────────────────────────
ax2d.set_facecolor('#0D1117')

levels = np.unique(np.concatenate([
    np.linspace(0, FC_P_MAX_MANUAL, 35),
    [P_PEAK_EFF, P_BASE, P_fc_max_sim]
]))
cf = ax2d.contourf(SOC_M, U_M, PFC_M, levels=levels, cmap=CMAP_3, norm=norm_pfc)

# Labelled iso-power contours
ct_levels = [100, P_PEAK_EFF, 200, 259, 300, 400, 600, 800, 1013]
ct = ax2d.contour(SOC_M, U_M, PFC_M, levels=sorted(ct_levels),
                  colors='white', linewidths=0.55, alpha=0.40)
ax2d.clabel(ct, fmt=lambda v: f'{v:.0f} W', fontsize=7,
            inline=True, inline_spacing=2, colors='white')

# Operating scatter
ax2d.scatter(SOC_s[::step], u_s[::step],
             c=P_fc_s[::step], cmap=CMAP_3, norm=norm_pfc,
             s=2.0, alpha=0.55, linewidths=0, zorder=6)

# Operating region bounding box (99th percentile)
soc_lo, soc_hi = np.percentile(SOC_s, 0.5), np.percentile(SOC_s, 99.5)
u_lo,   u_hi   = np.percentile(u_s,   0.5), np.percentile(u_s,   99.5)
rect = plt.Rectangle(
    (soc_lo, u_lo), soc_hi - soc_lo, u_hi - u_lo,
    lw=1.8, edgecolor='#FFEB3B', facecolor='none',
    ls='-', zorder=9
)
ax2d.add_patch(rect)
ax2d.text(soc_hi + 0.004, (u_lo + u_hi) / 2,
          f'Race operating\nregion\n'
          f'SOC {soc_lo:.3f}–{soc_hi:.3f}\n'
          f'u   {u_lo:.4f}–{u_hi:.4f} s⁻¹\n'
          f'P_fc {P_fc_min_sim:.0f}–{P_fc_max_sim:.0f} W',
          color='#FFEB3B', fontsize=7.8, va='center',
          bbox=dict(boxstyle='round,pad=0.35', fc='#0D1117', ec='#FFEB3B', lw=0.9))

# Reference lines
ax2d.axvline(0.60, color='#80DEEA', lw=1.0, ls='--', alpha=0.8, label='SOC₀ = 0.60')
ax2d.axhline( U_DEADBAND, color='#FFD600', lw=0.8, ls=':', alpha=0.7)
ax2d.axhline(-U_DEADBAND, color='#FFD600', lw=0.8, ls=':', alpha=0.7,
             label='|u| deadband ±0.02 s⁻¹')

# Horizontal peak-efficiency P line
ax2d.axhline(0, color='#444', lw=0.5)

ax2d.set_xlim(0.10, 0.95)
ax2d.set_ylim(-0.055, 0.055)
ax2d.set_xlabel('SOC  [—]',               color='white', fontsize=11, fontweight='bold')
ax2d.set_ylabel('u = P_sc / E_sc  [s⁻¹]', color='white', fontsize=11, fontweight='bold')
ax2d.set_title('Top-Down Contour  +  Race Operating Scatter',
               color='white', fontsize=10)
ax2d.tick_params(colors='#CCCCCC', labelsize=9)
for sp in ax2d.spines.values():
    sp.set_edgecolor('#2A2A2A')

leg = ax2d.legend(loc='lower right', fontsize=8.5, facecolor='#1A1A2E',
                  edgecolor='#444', labelcolor='white', framealpha=0.9)

# --- FC efficiency inset (top-right of 2-D panel) ---
if _HAVE_FC:
    ax_ins = ax2d.inset_axes([0.01, 0.62, 0.42, 0.36])   # [x0, y0, w, h] in axes fraction
    ax_ins.set_facecolor('#111827')
    ax_ins.plot(_Pnet, _Eff, color='#69F0AE', lw=1.6, zorder=3)
    # Shade below curve
    ax_ins.fill_between(_Pnet, _Eff, alpha=0.15, color='#69F0AE')
    # Three anchor marks
    for P_a, col_a, label_a in [
            (0.0,            '#1E88E5', '0 W'),
            (P_PEAK_EFF,     '#00C853', f'{P_PEAK_EFF:.0f} W\n{ETA_PEAK:.1f}%'),
            (FC_P_MAX_MANUAL,'#EF5350', '1013 W'),
    ]:
        P_clamp = max(P_a, 0.01)
        if P_clamp <= _Pnet.max():
            idx_a = np.argmin(np.abs(_Pnet - P_clamp))
            ax_ins.scatter([_Pnet[idx_a]], [_Eff[idx_a]], color=col_a,
                           s=38, zorder=5, edgecolors='white', linewidths=0.6)
    ax_ins.axvline(P_PEAK_EFF, color='#00C853', lw=0.9, ls='--', alpha=0.7)
    ax_ins.set_xlim(0, 1100)
    ax_ins.set_ylim(0, 70)
    ax_ins.set_xlabel('P_net [W]', color='#AAAAAA', fontsize=7)
    ax_ins.set_ylabel('η_sys [%]', color='#AAAAAA', fontsize=7)
    ax_ins.set_title('FC System η  (v3)', color='#AAAAAA', fontsize=7.5)
    ax_ins.tick_params(colors='#888888', labelsize=6.5)
    for sp in ax_ins.spines.values():
        sp.set_edgecolor('#333333')
    ax_ins.grid(True, color='#1E293B', lw=0.3)

# ── Shared colour bar ─────────────────────────────────────────────────────────
sm = cm.ScalarMappable(cmap=CMAP_3, norm=norm_pfc)
sm.set_array([])
cb = fig.colorbar(sm, cax=ax_cb)
cb.set_label('P_fc  [W]', color='white', fontsize=11, fontweight='bold', labelpad=10)

tick_vals   = [0, P_PEAK_EFF, P_BASE, P_fc_max_sim, 500, 700, 900, FC_P_MAX_MANUAL]
tick_labels = ['0\n(off)',
               f'{P_PEAK_EFF:.0f}\n(peak η)',
               f'{P_BASE:.0f}\n(base)',
               f'{P_fc_max_sim:.0f}\n(sim max)',
               '500', '700', '900',
               f'{FC_P_MAX_MANUAL:.0f}\n(FC max)']
cb.set_ticks(sorted(tick_vals))
cb.set_ticklabels([tick_labels[tick_vals.index(v)] for v in sorted(tick_vals)])
cb.ax.tick_params(colors='white', labelsize=8)
cb.outline.set_edgecolor('#444444')

# Horizontal marks at the three physics anchors
for v_mark, c_mark in [
        (0.0,            '#1E88E5'),
        (P_PEAK_EFF,     '#00C853'),
        (FC_P_MAX_MANUAL,'#EF5350'),
]:
    cb.ax.axhline(norm_pfc(max(v_mark, 0.001)), color=c_mark, lw=2.5, alpha=0.9)

# ── Legend patches for colormap meaning ───────────────────────────────────────
patch_off  = mpatches.Patch(color='#1E88E5', label='0 W — FC off')
patch_peak = mpatches.Patch(color='#00C853',
                             label=f'{P_PEAK_EFF:.0f} W — peak η_sys ({ETA_PEAK:.1f}%)')
patch_max  = mpatches.Patch(color='#B71C1C',
                             label=f'{FC_P_MAX_MANUAL:.0f} W — FC rated max (manual)')
fig.legend(handles=[patch_off, patch_peak, patch_max],
           loc='lower center', ncol=3, fontsize=9.5,
           facecolor='#1A1A2E', edgecolor='#444', labelcolor='white',
           framealpha=0.92, bbox_to_anchor=(0.50, 0.005))

# ── Master title ──────────────────────────────────────────────────────────────
fig.suptitle(
    'Strategy A — Heuristic 2D Lookup Table   ·   Hydraix I SEM 2026',
    color='white', fontsize=15, fontweight='bold', y=0.975, fontfamily='monospace'
)
fig.text(
    0.50, 0.935,
    f'K_soc = {K_SOC_TUNED} W/SOC  ·  K_rate = {K_RATE:.0f} W·s  ·  '
    f'P_base = {P_BASE:.0f} W  ·  '
    f'Race: H₂ = {res["m_H2_total"]:.3f} g  ·  ΔSOC = {res["delta_SOC"]:+.4f}  ·  '
    f'P_fc range {P_fc_min_sim:.0f}–{P_fc_max_sim:.0f} W',
    color='#AAAAAA', fontsize=8.8, ha='center', va='top', style='italic',
    bbox=dict(boxstyle='round,pad=0.4', fc='#1A1A2E', ec='#333355', lw=0.8, alpha=0.9)
)

# ── Save ──────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS_DIR, exist_ok=True)
out_path = os.path.join(RESULTS_DIR, 'strategy_a_lut_3d_v2.png')
fig.savefig(out_path, dpi=180, bbox_inches='tight',
            facecolor=fig.get_facecolor())
plt.close(fig)
print(f"\nSaved → {out_path}")
