"""
compare_motor_efficiency.py — Motor operating-point analysis.

Compares two drive profiles on the BAFANG RM G060.1000 efficiency map:
  1. Canonical GPS lap (canonical_with_incline.csv)  — constant-speed driving
  2. Elevation-aware P&G lap (sem_2026_elev_aware.csv) — pulse-and-glide

For each profile:
  - P_elec distribution (when motor is active)
  - P_out = solve P_elec * motor_eta(P_out) iteratively
  - η distribution at active operating points
  - Overlay on the full BAFANG efficiency curve
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from scipy.ndimage import gaussian_filter1d

from ems_core import (
    build_demand_profile, motor_eta,
    _M_POUT_S, _M_ETA_S,          # raw efficiency table
    RESULTS_DIR, N_LAPS, DT,
)

MATLAB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'matlab')
OUT_PNG    = os.path.join(RESULTS_DIR, 'motor_efficiency_comparison.png')

ETA_DT = 0.95   # drivetrain efficiency

# ── Efficiency regions ────────────────────────────────────────────────────────
# Used for shading and stats
ETA_BANDS = [
    (0.00, 0.55, '#d73027', 'Poor  (<55%)'),
    (0.55, 0.70, '#fc8d59', 'Low   (55–70%)'),
    (0.70, 0.78, '#fee090', 'Moderate (70–78%)'),
    (0.78, 0.83, '#91cf60', 'Good  (78–83%)'),
    (0.83, 1.00, '#1a9850', 'Peak  (>83%)'),
]


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _p_elec_to_p_out(p_elec_arr):
    """
    Iteratively solve P_out = P_elec * motor_eta(P_out).
    Start from P_out = P_elec * 0.75, converge in ~5 iterations.
    Returns P_out [W] array.
    """
    p_out = np.array(p_elec_arr, dtype=float) * 0.75
    for _ in range(8):
        eta = motor_eta(p_out)
        p_out = np.array(p_elec_arr, dtype=float) * eta
    return p_out


def _load_canonical():
    """Load GPS canonical profile (all 11 laps) → P_elec [W] array."""
    t_lap, P_lap = build_demand_profile()
    P_full = np.tile(P_lap, N_LAPS)
    return P_full


def _load_new_profile():
    """Load elevation-aware CSV → P_elec [W] array."""
    csv = os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv')
    df  = pd.read_csv(csv)
    return df['P_elec_W'].values


def _active(p_elec_arr, threshold=10.0):
    """Return only active-motor samples (P > threshold W)."""
    return p_elec_arr[p_elec_arr > threshold]


def _eta_stats(p_elec_arr, label):
    """Compute and print efficiency statistics for a profile."""
    P_active = _active(p_elec_arr)
    P_out    = _p_elec_to_p_out(P_active)
    eta_vals = motor_eta(P_out)

    frac_active = len(P_active) / len(p_elec_arr)
    mean_eta    = float(np.mean(eta_vals))
    median_eta  = float(np.median(eta_vals))

    band_fracs = {}
    for lo, hi, _, name in ETA_BANDS:
        mask = (eta_vals >= lo) & (eta_vals < hi)
        band_fracs[name] = float(np.sum(mask)) / len(eta_vals)

    print(f"\n  {label}")
    print(f"    Total samples         : {len(p_elec_arr)}")
    print(f"    Active-motor fraction : {frac_active*100:.1f}%")
    print(f"    Mean P_elec (active)  : {float(np.mean(P_active)):.0f} W")
    print(f"    Mean motor η (active) : {mean_eta*100:.1f}%")
    print(f"    Median motor η        : {median_eta*100:.1f}%")
    for name, frac in band_fracs.items():
        print(f"    {name:<20}: {frac*100:.1f}% of active time")

    return P_active, P_out, eta_vals, frac_active, mean_eta


# ═══════════════════════════════════════════════════════════════════════════════
# Load data
# ═══════════════════════════════════════════════════════════════════════════════

print("Loading profiles …")
P_can  = _load_canonical()
P_new  = _load_new_profile()

print("\n── Motor Operating-Point Analysis ────────────────────────────────────")
P_can_act,  P_can_out,  eta_can,  frac_can,  mean_eta_can  = _eta_stats(P_can,  'Canonical GPS (constant-speed)')
P_new_act,  P_new_out,  eta_new,  frac_new,  mean_eta_new  = _eta_stats(P_new,  'Elev-aware P&G (new profile)')


# ═══════════════════════════════════════════════════════════════════════════════
# Figure
# ═══════════════════════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(16, 13), dpi=150)
fig.suptitle(
    "BAFANG RM G060.1000 Motor Efficiency — Operating-Point Comparison\n"
    "Canonical GPS vs Elevation-Aware Pulse-and-Glide Profile",
    fontsize=12, fontweight='bold', y=0.985,
)

gs = fig.add_gridspec(3, 2, hspace=0.48, wspace=0.32,
                      height_ratios=[1.4, 1.0, 1.0])

ax_eff   = fig.add_subplot(gs[0, :])    # Row 1 full: efficiency curve + operating clouds
ax_phin  = fig.add_subplot(gs[1, 0])   # Row 2 left:  P_elec histogram
ax_etah  = fig.add_subplot(gs[1, 1])   # Row 2 right: η histogram
ax_bar   = fig.add_subplot(gs[2, 0])   # Row 3 left:  efficiency-band bar chart
ax_sum   = fig.add_subplot(gs[2, 1])   # Row 3 right: summary metrics

# ── Panel 1: BAFANG efficiency curve + operating clouds ──────────────────────
# Draw efficiency bands as background fills
P_curve = np.linspace(0, 1250, 1000)
eta_curve = motor_eta(P_curve)

for lo, hi, color, label in ETA_BANDS:
    mask = (eta_curve >= lo) & (eta_curve <= hi)
    # Fill band between lo and hi across the full x range
    ax_eff.axhspan(lo, hi, color=color, alpha=0.18, zorder=0)

# Draw the BAFANG efficiency curve
ax_eff.plot(_M_POUT_S, _M_ETA_S, 'k-', linewidth=2.5,
            zorder=5, label='BAFANG efficiency curve')
ax_eff.plot(_M_POUT_S, _M_ETA_S, 'wo', markersize=3,
            zorder=6, markeredgecolor='k', markeredgewidth=0.4)

# Operating-point density: use 2D histogram → contour
def _plot_op_cloud(ax, p_out, eta_vals, color, label, alpha=0.75):
    """Plot operating points as a scatter + marginal KDE line on efficiency curve."""
    # Jitter slightly so overlapping points are visible
    rng = np.random.default_rng(42)
    jx  = rng.normal(0, 5, len(p_out))
    jy  = rng.normal(0, 0.002, len(eta_vals))
    sc = ax.scatter(p_out + jx, eta_vals + jy,
                    color=color, alpha=alpha * 0.35, s=3, zorder=3,
                    linewidths=0, label=f'{label}  (n={len(p_out):,})')

    # Density contour via 2D histogram
    x_edges = np.linspace(0, 1250, 80)
    y_edges = np.linspace(0.15, 0.87, 60)
    H, xe, ye = np.histogram2d(p_out, eta_vals, bins=[x_edges, y_edges])
    H_smooth = gaussian_filter1d(gaussian_filter1d(H, sigma=2, axis=0), sigma=2, axis=1)
    Xc = 0.5 * (xe[:-1] + xe[1:])
    Yc = 0.5 * (ye[:-1] + ye[1:])
    levels = np.percentile(H_smooth[H_smooth > 0], [50, 75, 90, 97])
    if len(np.unique(levels)) > 1:
        ax.contour(Xc, Yc, H_smooth.T, levels=levels,
                   colors=[color], linewidths=[0.6, 0.9, 1.2, 1.8],
                   alpha=0.85, zorder=4)

_plot_op_cloud(ax_eff, P_can_out, eta_can, '#2166ac',
               'Canonical GPS', alpha=0.6)
_plot_op_cloud(ax_eff, P_new_out, eta_new, '#d6604d',
               'Elev-aware P&G', alpha=0.8)

# Band legend patches
band_patches = [mpatches.Patch(color=c, alpha=0.4, label=n)
                for _, _, c, n in ETA_BANDS]
handles, labels = ax_eff.get_legend_handles_labels()
ax_eff.legend(handles=handles + band_patches, fontsize=7,
              loc='lower right', ncol=2)

ax_eff.set_xlabel('Motor shaft output power P_out [W]', fontsize=9)
ax_eff.set_ylabel('Motor efficiency η  [—]', fontsize=9)
ax_eff.set_title(
    'Motor Efficiency Curve — Operating-Point Density  '
    '(scatter + iso-density contours)',
    fontsize=10, fontweight='bold',
)
ax_eff.set_xlim(0, 1260)
ax_eff.set_ylim(0.15, 0.88)
# Annotate peak
ax_eff.annotate('Peak η = 83.4%\n@ 1213 W',
                xy=(1213, 0.834), xytext=(950, 0.80),
                fontsize=8, color='darkgreen',
                arrowprops=dict(arrowstyle='->', color='darkgreen', lw=1.0),
                bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.7))
ax_eff.grid(True, lw=0.3, alpha=0.5)

# ── Panel 2: P_elec histogram (active-motor only) ───────────────────────────
bins_p = np.arange(0, 1301, 40)

ax_phin.hist(P_can_act, bins=bins_p, color='#2166ac', alpha=0.55,
             density=True, label=f'Canonical GPS\nActive: {frac_can*100:.0f}%\nMean: {np.mean(P_can_act):.0f} W')
ax_phin.hist(P_new_act, bins=bins_p, color='#d6604d', alpha=0.55,
             density=True, label=f'Elev-aware P&G\nActive: {frac_new*100:.0f}%\nMean: {np.mean(P_new_act):.0f} W')

# KDE overlay
for arr, color in [(P_can_act, '#2166ac'), (P_new_act, '#d6604d')]:
    x_kde  = np.linspace(0, 1300, 500)
    bw     = 1.06 * np.std(arr) * len(arr)**(-1/5)
    kde    = np.array([np.mean(np.exp(-0.5*((x-arr)/bw)**2)/(bw*np.sqrt(2*np.pi)))
                       for x in x_kde])
    ax_phin.plot(x_kde, kde, color=color, linewidth=1.8)

ax_phin.set_xlabel('Motor electrical power P_elec [W]', fontsize=9)
ax_phin.set_ylabel('Probability density', fontsize=9)
ax_phin.set_title('P_elec Distribution — Active-Motor Timesteps', fontsize=9, fontweight='bold')
ax_phin.legend(fontsize=7, loc='upper left')
ax_phin.set_xlim(0, 1300)
ax_phin.grid(True, lw=0.3)

# ── Panel 3: Motor efficiency histogram ─────────────────────────────────────
bins_e = np.arange(0.15, 0.875, 0.025)

ax_etah.hist(eta_can, bins=bins_e, color='#2166ac', alpha=0.55,
             density=True, label=f'Canonical GPS\nMean η={mean_eta_can*100:.1f}%')
ax_etah.hist(eta_new, bins=bins_e, color='#d6604d', alpha=0.55,
             density=True, label=f'Elev-aware P&G\nMean η={mean_eta_new*100:.1f}%')

# Shade efficiency bands on background
for lo, hi, color, _ in ETA_BANDS:
    ax_etah.axvspan(lo, hi, color=color, alpha=0.12, zorder=0)

ax_etah.axvline(mean_eta_can, color='#2166ac', lw=1.8, ls='--')
ax_etah.axvline(mean_eta_new, color='#d6604d', lw=1.8, ls='--')

ax_etah.set_xlabel('Motor efficiency η  [—]', fontsize=9)
ax_etah.set_ylabel('Probability density', fontsize=9)
ax_etah.set_title('Motor Efficiency Distribution — Active-Motor Timesteps', fontsize=9, fontweight='bold')
ax_etah.legend(fontsize=7, loc='upper left')
ax_etah.set_xlim(0.15, 0.87)
ax_etah.grid(True, lw=0.3)

# ── Panel 4: Efficiency-band occupancy bar chart ─────────────────────────────
band_names = [n for _, _, _, n in ETA_BANDS]
can_fracs  = []
new_fracs  = []

for lo, hi, _, _ in ETA_BANDS:
    can_fracs.append(float(np.sum((eta_can >= lo) & (eta_can < hi))) / len(eta_can))
    new_fracs.append(float(np.sum((eta_new >= lo) & (eta_new < hi))) / len(eta_new))

x_pos = np.arange(len(band_names))
w = 0.35
bars1 = ax_bar.bar(x_pos - w/2, [f*100 for f in can_fracs], w,
                   color='#2166ac', alpha=0.75, label='Canonical GPS')
bars2 = ax_bar.bar(x_pos + w/2, [f*100 for f in new_fracs], w,
                   color='#d6604d', alpha=0.75, label='Elev-aware P&G')

# Value labels on bars
for bar in bars1:
    h = bar.get_height()
    if h > 1:
        ax_bar.text(bar.get_x() + bar.get_width()/2, h + 0.5,
                    f'{h:.0f}%', ha='center', va='bottom', fontsize=7)
for bar in bars2:
    h = bar.get_height()
    if h > 1:
        ax_bar.text(bar.get_x() + bar.get_width()/2, h + 0.5,
                    f'{h:.0f}%', ha='center', va='bottom', fontsize=7,
                    color='#d6604d')

ax_bar.set_xticks(x_pos)
ax_bar.set_xticklabels(band_names, fontsize=7, rotation=15, ha='right')
ax_bar.set_ylabel('% of active-motor time', fontsize=9)
ax_bar.set_title('Time Spent in Each Efficiency Band\n(active-motor timesteps only)',
                 fontsize=9, fontweight='bold')
ax_bar.legend(fontsize=8)
ax_bar.grid(True, lw=0.3, axis='y')
ax_bar.set_ylim(0, max(max(can_fracs), max(new_fracs)) * 100 * 1.18)

# ── Panel 5: Summary metrics table ──────────────────────────────────────────
ax_sum.axis('off')

# Compute "effective energy efficiency": avg η weighted by P_out
weighted_eta_can = float(np.average(eta_can, weights=P_can_out)) if len(P_can_out) > 0 else 0
weighted_eta_new = float(np.average(eta_new, weights=P_new_out)) if len(P_new_out) > 0 else 0

# Compute peak-band fraction (η > 0.78)
peak_can = float(np.sum(eta_can >= 0.78)) / len(eta_can)
peak_new = float(np.sum(eta_new >= 0.78)) / len(eta_new)

# Estimate motor losses: E_loss = sum(P_elec - P_out) * DT
e_loss_can = float(np.sum(P_can_act - P_can_out)) * DT
e_loss_new = float(np.sum(P_new_act - P_new_out)) * DT
total_time_can = len(P_can) * DT
total_time_new = len(P_new) * DT

col_labels = ['Metric', 'Canonical GPS', 'Elev-aware P&G', 'Improvement']
rows = [
    ['Active-motor fraction',
     f'{frac_can*100:.0f}%',
     f'{frac_new*100:.0f}%',
     f'{(frac_new - frac_can)*100:+.0f}pp'],
    ['Mean P_elec (active)',
     f'{np.mean(P_can_act):.0f} W',
     f'{np.mean(P_new_act):.0f} W',
     f'{np.mean(P_new_act)-np.mean(P_can_act):+.0f} W'],
    ['Mean η (simple avg)',
     f'{mean_eta_can*100:.1f}%',
     f'{mean_eta_new*100:.1f}%',
     f'{(mean_eta_new-mean_eta_can)*100:+.1f}pp'],
    ['Mean η (power-weighted)',
     f'{weighted_eta_can*100:.1f}%',
     f'{weighted_eta_new*100:.1f}%',
     f'{(weighted_eta_new-weighted_eta_can)*100:+.1f}pp'],
    ['Time in Good+Peak (>78%)',
     f'{peak_can*100:.0f}%',
     f'{peak_new*100:.0f}%',
     f'{(peak_new-peak_can)*100:+.0f}pp'],
    ['Total motor losses',
     f'{e_loss_can/3600:.2f} Wh',
     f'{e_loss_new/3600:.2f} Wh',
     f'{(e_loss_new-e_loss_can)/3600:+.2f} Wh'],
]

table = ax_sum.table(
    cellText=rows,
    colLabels=col_labels,
    cellLoc='center',
    loc='center',
    bbox=[0, 0, 1, 1],
)
table.auto_set_font_size(False)
table.set_fontsize(8)
for (r, c), cell in table.get_celld().items():
    if r == 0:
        cell.set_facecolor('#333333')
        cell.set_text_props(color='white', fontweight='bold')
    elif r % 2 == 0:
        cell.set_facecolor('#f0f0f0')
    if c == 3 and r > 0:   # Improvement column
        txt = cell.get_text().get_text()
        if txt.startswith('+'):
            cell.set_facecolor('#d4edda')
        elif txt.startswith('-'):
            cell.set_facecolor('#f8d7da')
    cell.set_edgecolor('#cccccc')

ax_sum.set_title('Summary Metrics', fontsize=9, fontweight='bold', pad=12)

plt.tight_layout(rect=[0, 0, 1, 0.975])
fig.savefig(OUT_PNG, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f"\nFigure saved → {OUT_PNG}")
