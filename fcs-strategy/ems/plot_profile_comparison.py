"""
Publication-quality comparison figures — Original vs Proposed velocity profiles.

  Original : Canonical GPS constant-speed drive cycle (canonical_with_incline.csv)
  Proposed : Elevation-aware Pulse-and-Glide profile   (sem_2026_elev_aware.csv)

Outputs (saved to results/ems/):
  plot1_velocity_time.png        Single-lap velocity vs time
  plot2_power_distribution.png   Motor power demand distribution + efficiency bands
  plot3_comparison_table.png     Head-to-head metrics table
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec
from scipy.ndimage import uniform_filter1d, gaussian_filter1d

from ems_core import (
    build_demand_profile, motor_eta,
    N_LAPS, DT, RESAMPLE_HZ, RESULTS_DIR, MATLAB_DIR,
)

# ── Publication rcParams ───────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.dpi':           300,
    'savefig.dpi':          300,
    'font.family':          'sans-serif',
    'font.sans-serif':      ['Arial', 'Helvetica Neue', 'DejaVu Sans'],
    'font.size':            9,
    'axes.labelsize':       10,
    'axes.titlesize':       10,
    'axes.titlepad':        6,
    'axes.labelpad':        4,
    'axes.linewidth':       0.8,
    'axes.spines.top':      False,
    'axes.spines.right':    False,
    'axes.grid':            True,
    'grid.linewidth':       0.35,
    'grid.color':           '#d8d8d8',
    'grid.alpha':           1.0,
    'xtick.labelsize':      8.5,
    'ytick.labelsize':      8.5,
    'xtick.direction':      'out',
    'ytick.direction':      'out',
    'xtick.major.size':     3.5,
    'ytick.major.size':     3.5,
    'xtick.major.width':    0.7,
    'ytick.major.width':    0.7,
    'lines.linewidth':      1.6,
    'lines.solid_capstyle': 'round',
    'legend.fontsize':      8.5,
    'legend.framealpha':    0.92,
    'legend.edgecolor':     '#cccccc',
    'legend.borderpad':     0.5,
    'legend.handlelength':  1.8,
    'figure.facecolor':     'white',
    'axes.facecolor':       '#fafafa',
})

# ── Colour palette (Wong colorblind-safe) ─────────────────────────────────────
C_ORIG  = '#0072B2'     # blue
C_PROP  = '#D55E00'     # vermillion
C_ELEV  = '#999999'     # neutral grey for elevation fill
C_GREEN = '#27AE60'     # improvement green
C_RED   = '#C0392B'     # worse red
C_NEUT  = '#555555'     # neutral dark grey

H2_DENSITY = 89.88      # g/m³ at STP
TOTAL_KM   = 14.5
ACTIVE_THR = 10.0       # W — below this: glide / motor idle

# Strategy G tuned results (from strategy_g_elev_compare.py)
H2_ORIG   = 8.669       # g
H2_PROP   = 4.281       # g
KM_M3_ORIG = TOTAL_KM / (H2_ORIG / H2_DENSITY)
KM_M3_PROP = TOTAL_KM / (H2_PROP / H2_DENSITY)


# ═══════════════════════════════════════════════════════════════════════════════
# Data loading
# ═══════════════════════════════════════════════════════════════════════════════
print("Loading profiles …")

# ── Original: one lap from canonical CSV ──────────────────────────────────────
raw = pd.read_csv(os.path.join(MATLAB_DIR, 'canonical_with_incline.csv'))
lap_dur_raw  = raw['elapsed_sec'].iloc[-1] / N_LAPS
raw_lap      = raw[raw['elapsed_sec'] <= lap_dur_raw].reset_index(drop=True)
t_raw        = raw_lap['elapsed_sec'].values
v_raw        = raw_lap['Speed_kmh'].values

smooth_pts   = int(RESAMPLE_HZ * 10)
t_1lap       = np.arange(t_raw[0], t_raw[-1], DT)
v_1lap       = np.interp(t_1lap, t_raw, v_raw)
v_1lap       = pd.Series(v_1lap).rolling(smooth_pts, center=True, min_periods=1).mean().values
t_orig_1lap  = t_1lap - t_1lap[0]                # zero-based [s]
v_orig_1lap  = v_1lap                             # [km/h]
T_lap_orig   = float(t_orig_1lap[-1])

# P_dem for full 11-lap race (calibrated via ems_core)
_t_lap_ref, P_1lap_ref = build_demand_profile()
P_orig_all   = np.tile(P_1lap_ref, N_LAPS)

# ── Proposed: read CSV ─────────────────────────────────────────────────────────
df_s         = pd.read_csv(os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv'))
t_sugg_all   = df_s['time_s'].values
v_sugg_all   = df_s['velocity_kmh'].values
P_sugg_all   = df_s['P_elec_W'].values
lap_num_all  = df_s['lap_num'].values
elev_all     = df_s['elevation_m'].values

# Single lap (lap 1)
mask_l1      = lap_num_all == 1
t_prop_1lap  = t_sugg_all[mask_l1] - t_sugg_all[mask_l1][0]  # zero-based [s]
v_prop_1lap  = v_sugg_all[mask_l1]                             # [km/h]
P_prop_1lap  = P_sugg_all[mask_l1]                             # [W]
elev_1lap    = elev_all[mask_l1]                               # [m]

# Glide mask for shading (motor off)
glide_mask   = P_prop_1lap < ACTIVE_THR


# ── Motor efficiency computation ──────────────────────────────────────────────
def p_elec_to_pout(p_elec):
    p_out = np.maximum(p_elec * 0.75, 0.0)
    for _ in range(8):
        eta = motor_eta(p_out)
        p_out = np.where(eta > 0.01, p_elec * eta, 0.0)
    return p_out

p_out_orig = p_elec_to_pout(P_orig_all)
p_out_prop = p_elec_to_pout(P_sugg_all)
eta_orig   = motor_eta(p_out_orig)
eta_prop   = motor_eta(p_out_prop)

act_o = P_orig_all > ACTIVE_THR
act_p = P_sugg_all > ACTIVE_THR

ETA_BANDS = [
    (0.00, 0.55, '#d73027', 'Poor  (< 55%)'),
    (0.55, 0.70, '#fc8d59', 'Low  (55–70%)'),
    (0.70, 0.78, '#fee090', 'Moderate (70–78%)'),
    (0.78, 0.83, '#91cf60', 'Good  (78–83%)'),
    (0.83, 1.00, '#1a9850', 'Peak  (> 83%)'),
]

def band_fracs(eta_arr, mask):
    e = eta_arr[mask]
    return [float(np.mean((e >= lo) & (e < hi)) * 100) for lo, hi, _, _ in ETA_BANDS]

bf_orig = band_fracs(eta_orig, act_o)
bf_prop = band_fracs(eta_prop, act_p)

# Summary scalars
dur_orig   = float(t_orig_1lap[-1] + DT) * N_LAPS / 60.0   # approx full race [min]
dur_prop   = float(t_sugg_all[-1]) / 60.0
dist_orig  = 14.500
dist_prop  = 14.516
act_frac_o = float(np.mean(act_o)) * 100
act_frac_p = float(np.mean(act_p)) * 100
mean_Pd_o  = float(np.mean(P_orig_all))
mean_Pd_p  = float(np.mean(P_sugg_all))
mean_Pact_o = float(np.mean(P_orig_all[act_o]))
mean_Pact_p = float(np.mean(P_sugg_all[act_p]))
mean_eta_o  = float(np.mean(eta_orig[act_o])) * 100
mean_eta_p  = float(np.mean(eta_prop[act_p])) * 100
gp_o = sum(bf_orig[3:]);  gp_p = sum(bf_prop[3:])
poor_o = bf_orig[0];      poor_p = bf_prop[0]
max_v_o = float(np.max(v_orig_1lap));  max_v_p = float(np.max(v_prop_1lap))
mean_v_o = dist_orig / (dur_orig / 60.0)
mean_v_p = dist_prop / (dur_prop / 60.0)

print(f"  Original  | dur={dur_orig:.1f} min  max_v={max_v_o:.1f} km/h  "
      f"η={mean_eta_o:.1f}%  active={act_frac_o:.1f}%")
print(f"  Proposed  | dur={dur_prop:.1f} min  max_v={max_v_p:.1f} km/h  "
      f"η={mean_eta_p:.1f}%  active={act_frac_p:.1f}%")


# ═══════════════════════════════════════════════════════════════════════════════
# PLOT 1 — Single-Lap Velocity vs Time
# ═══════════════════════════════════════════════════════════════════════════════
print("\nPlot 1: Single-lap velocity vs time …")

fig1, ax1 = plt.subplots(figsize=(7.0, 3.2))

# Glide phase shading (before lines, stays in background)
prev_state = glide_mask[0]
seg_start  = t_prop_1lap[0]
for k in range(1, len(t_prop_1lap)):
    if glide_mask[k] != prev_state or k == len(t_prop_1lap) - 1:
        seg_end = t_prop_1lap[k]
        if prev_state:   # this segment was a glide
            ax1.axvspan(seg_start, seg_end, color='#f5f5f5', zorder=0,
                        linewidth=0, alpha=0.95)
        seg_start  = t_prop_1lap[k]
        prev_state = glide_mask[k]

# Thin reference line at 0 km/h
ax1.axhline(0, color='#cccccc', lw=0.5, zorder=1)

# Mean speed reference lines
ax1.axhline(mean_v_o, color=C_ORIG, lw=0.9, ls=':', alpha=0.65, zorder=2)
ax1.axhline(mean_v_p, color=C_PROP, lw=0.9, ls=':', alpha=0.65, zorder=2)

# Main velocity traces
ax1.plot(t_orig_1lap, v_orig_1lap, color=C_ORIG, lw=1.8, zorder=4,
         label=f'Original  (mean {mean_v_o:.1f} km/h, peak {max_v_o:.1f} km/h)')
ax1.plot(t_prop_1lap, v_prop_1lap, color=C_PROP, lw=1.8, zorder=4,
         label=f'Proposed  (mean {mean_v_p:.1f} km/h, peak {max_v_p:.1f} km/h)')

# Annotate mean lines
t_ann = 0.97 * T_lap_orig
ax1.annotate(f'$\\bar{{v}}_\\mathrm{{orig}}$ = {mean_v_o:.1f}',
             xy=(t_ann, mean_v_o), xytext=(0, 5), textcoords='offset points',
             color=C_ORIG, fontsize=7.5, ha='right', va='bottom', style='italic')
ax1.annotate(f'$\\bar{{v}}_\\mathrm{{prop}}$ = {mean_v_p:.1f}',
             xy=(t_ann, mean_v_p), xytext=(0, -10), textcoords='offset points',
             color=C_PROP, fontsize=7.5, ha='right', va='top', style='italic')

# Glide legend patch
glide_patch = mpatches.Patch(facecolor='#e8e8e8', edgecolor='none',
                              label='Proposed: glide phase (motor off)')

handles, labels = ax1.get_legend_handles_labels()
ax1.legend(handles=handles + [glide_patch],
           labels=labels + ['Glide phase (motor off)'],
           loc='upper right', framealpha=0.92, fontsize=8.5)

ax1.set_xlabel('Time within lap  [s]', fontsize=10)
ax1.set_ylabel('Vehicle speed  [km/h]', fontsize=10)
ax1.set_title('Single-Lap Velocity Profile — Original vs Proposed',
              fontsize=10, fontweight='bold', pad=6)
ax1.set_xlim(0, max(t_orig_1lap[-1], t_prop_1lap[-1]) * 1.02)
ax1.set_ylim(-0.5, max(max_v_o, max_v_p) * 1.12)
ax1.yaxis.set_major_locator(mticker.MultipleLocator(5))
ax1.xaxis.set_major_locator(mticker.MultipleLocator(20))

# Secondary x-axis: distance proxy (Original only, approximate)
ax1_top = ax1.twiny()
ax1_top.set_xlim(ax1.get_xlim())
# tick positions every 200m, using mean speed to convert
mps_orig = mean_v_o / 3.6
tick_t   = [d / mps_orig for d in range(0, int(dist_orig / N_LAPS * 1000) + 1, 200)]
tick_lbl = [f'{d}' for d in range(0, int(dist_orig / N_LAPS * 1000) + 1, 200)]
ax1_top.set_xticks(tick_t)
ax1_top.set_xticklabels(tick_lbl, fontsize=7.5)
ax1_top.set_xlabel('Approximate lap distance  [m]', fontsize=8.5, labelpad=3)
ax1_top.spines['top'].set_visible(True)
ax1_top.spines['top'].set_linewidth(0.6)
ax1_top.tick_params(axis='x', direction='out', length=3, width=0.6)

fig1.tight_layout()
out1 = os.path.join(RESULTS_DIR, 'plot1_velocity_time.png')
fig1.savefig(out1, dpi=300, bbox_inches='tight', facecolor='white')
plt.close(fig1)
print(f"  Saved → {out1}")


# ═══════════════════════════════════════════════════════════════════════════════
# PLOT 2 — Motor Efficiency Operating-Point Comparison  (5-panel layout)
# ═══════════════════════════════════════════════════════════════════════════════
print("Plot 2: Motor efficiency operating-point comparison …")

# Active-only arrays for scatter / histograms
p_out_orig_act = p_out_orig[act_o]
p_out_prop_act = p_out_prop[act_p]
eta_orig_act   = eta_orig[act_o]
eta_prop_act   = eta_prop[act_p]
P_orig_act     = P_orig_all[act_o]
P_prop_act     = P_sugg_all[act_p]

# Weighted efficiency
weta_o = float(np.average(eta_orig_act, weights=p_out_orig_act)) if len(p_out_orig_act) else 0
weta_p = float(np.average(eta_prop_act, weights=p_out_prop_act)) if len(p_out_prop_act) else 0

# Motor losses
e_loss_o = float(np.sum(P_orig_act - p_out_orig_act)) * DT
e_loss_p = float(np.sum(P_prop_act - p_out_prop_act)) * DT

fig2 = plt.figure(figsize=(14, 11), dpi=300)
fig2.suptitle(
    'BAFANG RM G060.1000 Motor Efficiency — Operating-Point Comparison\n'
    'Original (Canonical GPS)  vs  Proposed (Elevation-Aware Pulse-and-Glide)',
    fontsize=11, fontweight='bold', y=0.985,
)
gs2 = fig2.add_gridspec(3, 2, hspace=0.50, wspace=0.30,
                         height_ratios=[1.45, 1.0, 1.0])
ax_eff  = fig2.add_subplot(gs2[0, :])   # full-width: η curve + clouds
ax_ph   = fig2.add_subplot(gs2[1, 0])   # P_elec histogram
ax_eh   = fig2.add_subplot(gs2[1, 1])   # η histogram
ax_bar2 = fig2.add_subplot(gs2[2, 0])   # efficiency-band bars
ax_tab2 = fig2.add_subplot(gs2[2, 1])   # summary metrics

# ── Row 1: BAFANG η curve + operating-point density ──────────────────────────
P_crv   = np.linspace(0, 1250, 1000)
eta_crv = motor_eta(P_crv)

for lo, hi, col, _ in ETA_BANDS:
    ax_eff.axhspan(lo, hi, color=col, alpha=0.18, zorder=0)

from ems_core import _M_POUT_S, _M_ETA_S
ax_eff.plot(_M_POUT_S, _M_ETA_S, 'k-', lw=2.2, zorder=5, label='BAFANG efficiency curve')
ax_eff.plot(_M_POUT_S, _M_ETA_S, 'wo', ms=3.0, zorder=6,
            markeredgecolor='k', markeredgewidth=0.4)

def _op_cloud(ax, p_out, eta_v, color, label):
    rng = np.random.default_rng(42)
    jx  = rng.normal(0, 4, len(p_out))
    jy  = rng.normal(0, 0.0015, len(eta_v))
    ax.scatter(p_out + jx, eta_v + jy,
               color=color, alpha=0.20, s=2.5, zorder=3, linewidths=0,
               label=f'{label}  (n={len(p_out):,})')
    xe = np.linspace(0, 1250, 80);  ye = np.linspace(0.15, 0.88, 60)
    H, xe2, ye2 = np.histogram2d(p_out, eta_v, bins=[xe, ye])
    Hs = gaussian_filter1d(gaussian_filter1d(H, sigma=2.0, axis=0), sigma=2.0, axis=1)
    Xc = 0.5*(xe2[:-1]+xe2[1:]);  Yc = 0.5*(ye2[:-1]+ye2[1:])
    lvls = np.percentile(Hs[Hs > 0], [50, 75, 90, 97])
    if len(np.unique(lvls)) > 1:
        ax.contour(Xc, Yc, Hs.T, levels=lvls,
                   colors=[color], linewidths=[0.5, 0.8, 1.1, 1.6], alpha=0.85, zorder=4)

_op_cloud(ax_eff, p_out_orig_act, eta_orig_act, C_ORIG, 'Original')
_op_cloud(ax_eff, p_out_prop_act, eta_prop_act, C_PROP, 'Proposed')

band_patches = [mpatches.Patch(color=c, alpha=0.45, label=n) for _, _, c, n in ETA_BANDS]
h2, l2 = ax_eff.get_legend_handles_labels()
ax_eff.legend(handles=h2 + band_patches, fontsize=7.5, loc='lower right', ncol=2,
              framealpha=0.92)
ax_eff.annotate('Peak η = 83.4%\n@ 1213 W',
                xy=(1213, 0.834), xytext=(940, 0.795), fontsize=8, color='#1a6e2e',
                arrowprops=dict(arrowstyle='->', color='#1a6e2e', lw=0.9),
                bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='#cccccc', alpha=0.85))
ax_eff.set_xlabel('Motor shaft output power  $P_\\mathrm{out}$  [W]', fontsize=10)
ax_eff.set_ylabel('Motor efficiency  η  [—]', fontsize=10)
ax_eff.set_title('Motor Efficiency Curve — Operating-Point Density  '
                 '(scatter + iso-density contours)', fontsize=10, fontweight='bold')
ax_eff.set_xlim(0, 1260);  ax_eff.set_ylim(0.15, 0.88)
ax_eff.grid(True, lw=0.3, alpha=0.5)

# ── Row 2 left: P_elec histogram (active only) ───────────────────────────────
bins_p = np.arange(0, 1321, 40)
ax_ph.hist(P_orig_act, bins=bins_p, color=C_ORIG, alpha=0.52, density=True,
           label=f'Original\nActive: {act_frac_o:.0f}%\nMean: {mean_Pact_o:.0f} W')
ax_ph.hist(P_prop_act, bins=bins_p, color=C_PROP, alpha=0.52, density=True,
           label=f'Proposed\nActive: {act_frac_p:.0f}%\nMean: {mean_Pact_p:.0f} W')
for arr, col in [(P_orig_act, C_ORIG), (P_prop_act, C_PROP)]:
    x_k = np.linspace(0, 1300, 500)
    bw  = 1.06 * np.std(arr) * len(arr)**(-0.2)
    kde = np.array([np.mean(np.exp(-0.5*((x-arr)/bw)**2)/(bw*np.sqrt(2*np.pi)))
                    for x in x_k])
    ax_ph.plot(x_k, kde, color=col, lw=1.8)
ax_ph.set_xlabel('Motor electrical power  $P_\\mathrm{elec}$  [W]', fontsize=9)
ax_ph.set_ylabel('Probability density', fontsize=9)
ax_ph.set_title('$P_\\mathrm{elec}$ Distribution — Active-Motor Timesteps',
                fontsize=9, fontweight='bold')
ax_ph.legend(fontsize=7.5, loc='upper left', framealpha=0.92)
ax_ph.set_xlim(0, 1300);  ax_ph.grid(True, lw=0.3)

# ── Row 2 right: η histogram (active only) ───────────────────────────────────
bins_e = np.arange(0.15, 0.876, 0.025)
ax_eh.hist(eta_orig_act, bins=bins_e, color=C_ORIG, alpha=0.52, density=True,
           label=f'Original\nMean η = {mean_eta_o:.1f}%')
ax_eh.hist(eta_prop_act, bins=bins_e, color=C_PROP, alpha=0.52, density=True,
           label=f'Proposed\nMean η = {mean_eta_p:.1f}%')
for lo, hi, col, _ in ETA_BANDS:
    ax_eh.axvspan(lo, hi, color=col, alpha=0.12, zorder=0)
ax_eh.axvline(mean_eta_o/100, color=C_ORIG, lw=1.8, ls='--')
ax_eh.axvline(mean_eta_p/100, color=C_PROP, lw=1.8, ls='--')
ax_eh.set_xlabel('Motor efficiency  η  [—]', fontsize=9)
ax_eh.set_ylabel('Probability density', fontsize=9)
ax_eh.set_title('Motor Efficiency Distribution — Active-Motor Timesteps',
                fontsize=9, fontweight='bold')
ax_eh.legend(fontsize=7.5, loc='upper left', framealpha=0.92)
ax_eh.set_xlim(0.15, 0.87);  ax_eh.grid(True, lw=0.3)

# ── Row 3 left: efficiency-band bar chart ─────────────────────────────────────
band_names2 = [b[3] for b in ETA_BANDS]
x2 = np.arange(len(ETA_BANDS));  bw2 = 0.35
b2o = ax_bar2.bar(x2 - bw2/2, bf_orig, bw2, color=C_ORIG, alpha=0.75, label='Original')
b2p = ax_bar2.bar(x2 + bw2/2, bf_prop, bw2, color=C_PROP, alpha=0.75, label='Proposed')
for bar, val in zip(b2o, bf_orig):
    if val > 1:
        ax_bar2.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.4,
                     f'{val:.0f}%', ha='center', va='bottom', fontsize=7)
for bar, val in zip(b2p, bf_prop):
    if val > 1:
        ax_bar2.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.4,
                     f'{val:.0f}%', ha='center', va='bottom', fontsize=7, color=C_PROP)
ax_bar2.set_xticks(x2)
ax_bar2.set_xticklabels(band_names2, fontsize=7.5, rotation=15, ha='right')
ax_bar2.set_ylabel('% of active-motor time', fontsize=9)
ax_bar2.set_title('Time Spent in Each Efficiency Band\n(active-motor timesteps only)',
                  fontsize=9, fontweight='bold')
ax_bar2.legend(fontsize=8.5, framealpha=0.92)
ax_bar2.set_ylim(0, max(max(bf_orig), max(bf_prop)) * 1.18)
ax_bar2.grid(True, axis='y', lw=0.3)

# ── Row 3 right: summary metrics table ───────────────────────────────────────
ax_tab2.axis('off')
tbl_rows = [
    ['Active-motor fraction',      f'{act_frac_o:.0f}%',          f'{act_frac_p:.0f}%',
     f'{act_frac_p - act_frac_o:+.0f}pp'],
    ['Mean $P_\\mathrm{elec}$ (active)', f'{mean_Pact_o:.0f} W',  f'{mean_Pact_p:.0f} W',
     f'{mean_Pact_p - mean_Pact_o:+.0f} W'],
    ['Mean η (simple avg)',         f'{mean_eta_o:.1f}%',          f'{mean_eta_p:.1f}%',
     f'{mean_eta_p - mean_eta_o:+.1f}pp'],
    ['Mean η (power-weighted)',     f'{weta_o*100:.1f}%',          f'{weta_p*100:.1f}%',
     f'{(weta_p-weta_o)*100:+.1f}pp'],
    ['Time in Good+Peak (> 78%)',   f'{gp_o:.0f}%',                f'{gp_p:.0f}%',
     f'{gp_p - gp_o:+.0f}pp'],
    ['Total motor losses',          f'{e_loss_o/3600:.2f} Wh',     f'{e_loss_p/3600:.2f} Wh',
     f'{(e_loss_p - e_loss_o)/3600:+.2f} Wh'],
]
tbl = ax_tab2.table(
    cellText=tbl_rows,
    colLabels=['Metric', 'Original', 'Proposed', 'Improvement'],
    cellLoc='center', loc='center', bbox=[0, 0, 1, 1],
)
tbl.auto_set_font_size(False);  tbl.set_fontsize(8)
for (r, c), cell in tbl.get_celld().items():
    if r == 0:
        cell.set_facecolor('#1A252F');  cell.set_text_props(color='white', fontweight='bold')
    elif r % 2 == 0:
        cell.set_facecolor('#f4f4f4')
    if c == 3 and r > 0:
        txt_v = cell.get_text().get_text()
        try:
            num = float(txt_v.replace('pp','').replace('W','').replace('Wh','').strip())
            # Active fraction: lower is better for P&G; losses: lower is better
            better_if_neg = r in (1, 6)   # active fraction row, losses row
            is_good = (num < 0) if better_if_neg else (num > 0)
            cell.set_facecolor('#d4edda' if is_good else '#f8d7da')
        except ValueError:
            pass
    cell.set_edgecolor('#cccccc')
ax_tab2.set_title('Summary Metrics', fontsize=9, fontweight='bold', pad=10)

fig2.tight_layout(rect=[0, 0, 1, 0.975])
out2 = os.path.join(RESULTS_DIR, 'plot2_power_distribution.png')
fig2.savefig(out2, dpi=300, bbox_inches='tight', facecolor='white')
plt.close(fig2)
print(f"  Saved → {out2}")


# ═══════════════════════════════════════════════════════════════════════════════
# PLOT 3 — Head-to-Head Comparison Table
# ═══════════════════════════════════════════════════════════════════════════════
print("Plot 3: Comparison table …")

# Each row: (category, metric_label, orig_str, prop_str, is_improvement)
#   is_improvement = True  → prop is better → green
#                  = False → prop is worse  → red
#                  = None  → neutral        → dark grey
rows = [
    # ── Drive Cycle ──────────────────────────────────────────────────────────
    ('Drive Cycle',  'Total distance',             '[km]',
     f'{dist_orig:.3f}',  f'{dist_prop:.3f}',  None),
    ('Drive Cycle',  'Race duration',              '[min]',
     f'{dur_orig:.1f}',   f'{dur_prop:.1f}',   True),
    ('Drive Cycle',  'Peak speed',                 '[km/h]',
     f'{max_v_o:.1f}',    f'{max_v_p:.1f}',    None),
    ('Drive Cycle',  'Mean speed (overall)',        '[km/h]',
     f'{mean_v_o:.1f}',   f'{mean_v_p:.1f}',   None),
    # ── Motor Operating Points ────────────────────────────────────────────────
    ('Motor',  'Active-motor fraction',        '[%]',
     f'{act_frac_o:.1f}',  f'{act_frac_p:.1f}',  True),
    ('Motor',  'Mean demand — all time',        '[W]',
     f'{mean_Pd_o:.0f}',   f'{mean_Pd_p:.0f}',   True),
    ('Motor',  'Mean demand — active only',     '[W]',
     f'{mean_Pact_o:.0f}', f'{mean_Pact_p:.0f}', True),
    ('Motor',  'Mean motor efficiency (active)', '[%]',
     f'{mean_eta_o:.1f}',  f'{mean_eta_p:.1f}',  True),
    ('Motor',  'Time in Good band (78–83%)',    '[%]',
     f'{bf_orig[3]:.1f}',  f'{bf_prop[3]:.1f}',  True),
    ('Motor',  'Time in Good + Peak (> 78%)',   '[%]',
     f'{gp_o:.1f}',        f'{gp_p:.1f}',         True),
    ('Motor',  'Time in Poor band (< 55%)',     '[%]',
     f'{poor_o:.1f}',      f'{poor_p:.1f}',       True),
    # ── Energy Management (Strategy G) ───────────────────────────────────────
    ('Strategy G',  'H₂ consumed',              '[g]',
     f'{H2_ORIG:.3f}',     f'{H2_PROP:.3f}',     True),
    ('Strategy G',  'H₂ saving vs Original',    '[%]',
     '—',                  f'{(1 - H2_PROP/H2_ORIG)*100:.1f}',  True),
    ('Strategy G',  'Fuel economy',             '[km/m³]',
     f'{KM_M3_ORIG:.1f}',  f'{KM_M3_PROP:.1f}',  True),
    ('Strategy G',  'Fuel economy improvement', '[km/m³]',
     '—',                  f'+{KM_M3_PROP - KM_M3_ORIG:.1f}',  True),
    ('Strategy G',  'Final ΔSOC',               '[ — ]',
     '≈ 0.000',            '≈ 0.000',             None),
]

# Layout constants
N_R    = len(rows)
FW     = 7.0
RH     = 0.36          # row height in data units
HDR_H  = 0.48          # header row height
CAT_H  = 0.30          # category separator height
PAD    = 0.05

# Column x positions and widths (as fractions of figure width)
# [category | metric | unit | original | proposed]
CX = np.array([0.000, 0.145, 0.590, 0.720, 0.855])
CW = np.array([0.145, 0.445, 0.130, 0.135, 0.145])

CATS = ['Drive Cycle', 'Motor', 'Strategy G']
CAT_COLORS = {
    'Drive Cycle': '#EBF5FB',
    'Motor':       '#FEF9E7',
    'Strategy G':  '#EAFAF1',
}
CAT_EDGE = {
    'Drive Cycle': '#AED6F1',
    'Motor':       '#F9E79F',
    'Strategy G':  '#A9DFBF',
}

# Compute total figure height
cat_counts = {c: sum(1 for r in rows if r[0] == c) for c in CATS}
total_h_units = (HDR_H + PAD +
                 sum(len(CATS) * CAT_H + cat_counts[c] * RH for c in CATS) +
                 len(CATS) * PAD)

# Use fixed height proportional to content
fig_h = 0.32 * N_R + 1.8
fig3, ax3 = plt.subplots(figsize=(FW, fig_h))
ax3.set_xlim(0, 1)
ax3.set_ylim(0, 1)
ax3.axis('off')

# Transform: map (row_idx) → y coordinate in axes fraction
total_rows_equiv = N_R + len(CATS) * 0.8 + 1.5   # header + cat rows + padding
def row_y(i):
    return 1.0 - (1.5 + i) / total_rows_equiv

def draw_cell(ax, x, y, w, h, text, fc, ec=None, ha='center', va='center',
              fontsize=9, bold=False, color='black', pad=0.005):
    if ec is None:
        ec = fc
    ax.add_patch(mpatches.FancyBboxPatch(
        (x + pad, y + pad), w - 2*pad, h - 2*pad,
        boxstyle='square,pad=0', fc=fc, ec=ec, lw=0.0,
        transform=ax.transAxes, clip_on=False))
    tx = x + (0.012 if ha == 'left' else w / 2)
    ax.text(tx, y + h / 2, text, transform=ax.transAxes,
            ha=ha, va='center', fontsize=fontsize,
            fontweight='bold' if bold else 'normal', color=color,
            clip_on=False)

# ── Full-width title rule ──────────────────────────────────────────────────────
title_y = row_y(-1.0)
row_h_ax = RH / (fig_h * total_rows_equiv / N_R * N_R)  # approximate

# Draw main title
fig3.suptitle('Original vs Proposed — Head-to-Head Performance Comparison',
              fontsize=11, fontweight='bold', y=0.99, va='top')

# ── Header row ────────────────────────────────────────────────────────────────
HDR_FC = '#1A252F'
hdr_y  = row_y(-0.65)
hdr_h  = 1.2 / total_rows_equiv
for ci, (cx, cw, lbl) in enumerate(zip(CX, CW,
        ['Category', 'Metric', 'Unit', 'Original', 'Proposed'])):
    ax3.add_patch(mpatches.Rectangle(
        (cx, hdr_y), cw, hdr_h,
        fc=HDR_FC, ec='none', transform=ax3.transAxes, clip_on=False))
    ax3.text(cx + cw / 2, hdr_y + hdr_h / 2, lbl,
             transform=ax3.transAxes, ha='center', va='center',
             fontsize=9.5, fontweight='bold', color='white', clip_on=False)

# Thick top and bottom rules on header
for rule_y in [hdr_y, hdr_y + hdr_h]:
    ax3.axhline(rule_y, color='#1A252F', lw=1.4)

# ── Data rows ─────────────────────────────────────────────────────────────────
row_h_ax = 0.92 / total_rows_equiv

prev_cat  = None
draw_idx  = 0   # y-position index (increments for both cat-headers and data rows)

for r_idx, (cat, metric, unit, v_o, v_p, impr) in enumerate(rows):
    if cat != prev_cat:
        # Category separator row
        cat_y = row_y(draw_idx + 0.1)
        c_h   = 0.72 / total_rows_equiv
        fc = CAT_COLORS[cat];  ec = CAT_EDGE[cat]
        ax3.add_patch(mpatches.Rectangle(
            (0.0, cat_y), 1.0, c_h,
            fc=fc, ec='none', transform=ax3.transAxes, clip_on=False))
        # Category label left-aligned, bold
        ax3.text(0.008, cat_y + c_h / 2, cat,
                 transform=ax3.transAxes, ha='left', va='center',
                 fontsize=8.5, fontweight='bold', color='#1A252F',
                 style='italic', clip_on=False)
        # Thin top rule for category
        ax3.axhline(cat_y + c_h, color='#c0c0c0', lw=0.7)
        draw_idx += 0.85
        prev_cat = cat

    y   = row_y(draw_idx)
    r_h = row_h_ax
    row_bg = '#ffffff' if r_idx % 2 == 0 else '#f7f9fa'

    # Category column (empty — category header above covers it)
    ax3.add_patch(mpatches.Rectangle(
        (CX[0], y), CW[0], r_h,
        fc=row_bg, ec='none', transform=ax3.transAxes, clip_on=False))

    # Metric column
    ax3.add_patch(mpatches.Rectangle(
        (CX[1], y), CW[1], r_h,
        fc=row_bg, ec='none', transform=ax3.transAxes, clip_on=False))
    ax3.text(CX[1] + 0.010, y + r_h / 2, metric,
             transform=ax3.transAxes, ha='left', va='center',
             fontsize=8.8, color='#222222', clip_on=False)

    # Unit column
    ax3.add_patch(mpatches.Rectangle(
        (CX[2], y), CW[2], r_h,
        fc=row_bg, ec='none', transform=ax3.transAxes, clip_on=False))
    ax3.text(CX[2] + CW[2] / 2, y + r_h / 2, unit,
             transform=ax3.transAxes, ha='center', va='center',
             fontsize=7.5, color='#777777', style='italic', clip_on=False)

    # Original column
    ax3.add_patch(mpatches.Rectangle(
        (CX[3], y), CW[3], r_h,
        fc=row_bg, ec='none', transform=ax3.transAxes, clip_on=False))
    ax3.text(CX[3] + CW[3] / 2, y + r_h / 2, v_o,
             transform=ax3.transAxes, ha='center', va='center',
             fontsize=8.8, color=C_ORIG, fontweight='bold', clip_on=False)

    # Proposed column — colour by improvement
    if impr is True:
        p_color = C_GREEN
        p_bg    = '#f0faf4'
    elif impr is False:
        p_color = C_RED
        p_bg    = '#fef0ef'
    else:
        p_color = C_NEUT
        p_bg    = row_bg

    ax3.add_patch(mpatches.Rectangle(
        (CX[4], y), CW[4], r_h,
        fc=p_bg, ec='none', transform=ax3.transAxes, clip_on=False))
    ax3.text(CX[4] + CW[4] / 2, y + r_h / 2, v_p,
             transform=ax3.transAxes, ha='center', va='center',
             fontsize=8.8, color=p_color, fontweight='bold', clip_on=False)

    # Row separator
    ax3.axhline(y, color='#e0e0e0', lw=0.4)

    draw_idx += 1

# Bottom rule
bottom_y = row_y(draw_idx)
ax3.axhline(bottom_y, color='#1A252F', lw=1.2)

# Vertical column separators (subtle)
for cx in CX[1:]:
    ax3.axvline(cx, color='#d5d5d5', lw=0.5, ymin=bottom_y)

# ── Legend for cell colours ───────────────────────────────────────────────────
legend_y = bottom_y - 0.055
ax3.text(CX[4] + CW[4] / 2, legend_y + 0.015,
         '■ green = improvement  ■ grey = neutral',
         transform=ax3.transAxes, ha='center', va='top',
         fontsize=7.0, color='#888888')

ax3.text(0.5, legend_y - 0.01,
         f'Strategy G (tuned FF+SOC-PI) · H₂ density = {H2_DENSITY} g/m³ at STP · '
         f'Race = {TOTAL_KM} km × 11 laps · Silesia Ring SEM 2026',
         transform=ax3.transAxes, ha='center', va='top',
         fontsize=7.0, color='#aaaaaa', style='italic')

fig3.tight_layout(rect=[0, 0, 1, 0.97])
out3 = os.path.join(RESULTS_DIR, 'plot3_comparison_table.png')
fig3.savefig(out3, dpi=300, bbox_inches='tight', facecolor='white')
plt.close(fig3)
print(f"  Saved → {out3}")

print("\nAll 3 publication-quality plots saved successfully.")
