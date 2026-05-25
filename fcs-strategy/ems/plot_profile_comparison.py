"""
Three standalone comparison plots — Original vs Suggested velocity profiles.

  Original  : Canonical GPS constant-speed drive cycle (canonical_with_incline.csv)
  Suggested : Elevation-aware Pulse-and-Glide profile   (sem_2026_elev_aware.csv)

Outputs (saved to results/ems/):
  plot1_velocity_time.png        Velocity vs time, all 11 laps
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
from matplotlib.patches import FancyBboxPatch
import matplotlib.ticker as mticker

from ems_core import (
    build_demand_profile, motor_eta, _M_POUT_S, _M_ETA_S,
    N_LAPS, DT, RESAMPLE_HZ, RESULTS_DIR, MATLAB_DIR,
    SC_SOC_0,
)

plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 10})

H2_DENSITY = 89.88   # g/m³ at STP
TOTAL_KM   = 14.5

C_ORIG = '#2166ac'   # blue  — Original
C_SUGG = '#d6604d'   # red   — Suggested


# ═══════════════════════════════════════════════════════════════════════════════
# Data loading
# ═══════════════════════════════════════════════════════════════════════════════

print("Loading profiles …")

# ── Original: reconstruct velocity and P_dem for all 11 laps ──────────────────
raw = pd.read_csv(os.path.join(MATLAB_DIR, 'canonical_with_incline.csv'))
lap_dur_raw = raw['elapsed_sec'].iloc[-1] / N_LAPS
raw_lap = raw[raw['elapsed_sec'] <= lap_dur_raw].reset_index(drop=True)

t_raw = raw_lap['elapsed_sec'].values
v_raw = raw_lap['Speed_kmh'].values        # km/h

smooth_pts = int(RESAMPLE_HZ * 10)        # 10-second window at 5 Hz

t_1lap = np.arange(t_raw[0], t_raw[-1], DT)
v_1lap = np.interp(t_1lap, t_raw, v_raw)
v_1lap = pd.Series(v_1lap).rolling(smooth_pts, center=True, min_periods=1).mean().values
T_lap_orig = float(t_1lap[-1] - t_1lap[0])

t_orig = np.concatenate([t_1lap + i * T_lap_orig for i in range(N_LAPS)])  # s
v_orig = np.tile(v_1lap, N_LAPS)                                             # km/h

# P_elec via ems_core (includes scale calibration to hit 14.5 km)
_t_lap, P_1lap = build_demand_profile()
P_orig = np.tile(P_1lap, N_LAPS)          # W, all 11 laps

# ── Suggested: read pre-built CSV ─────────────────────────────────────────────
df_s = pd.read_csv(os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv'))
t_sugg = df_s['time_s'].values             # s
v_sugg = df_s['velocity_kmh'].values       # km/h
P_sugg = df_s['P_elec_W'].values           # W
lap_sugg = (df_s['lap_num'].values - 1).astype(int)

# Lap boundary times (first sample of each new lap, laps 1..10)
lap_t_orig = [T_lap_orig * (i + 1) / 60.0 for i in range(N_LAPS - 1)]
lap_t_sugg = []
for li in range(1, N_LAPS):
    idx = np.argmax(lap_sugg == li)
    lap_t_sugg.append(float(t_sugg[idx]) / 60.0)


# ── Motor efficiency conversion: P_elec → P_out (iterative, 8 iters) ─────────
def _p_elec_to_p_out(p_elec):
    p_out = np.maximum(p_elec * 0.75, 0.0)
    for _ in range(8):
        eta = motor_eta(p_out)
        p_out = np.where(eta > 0.01, p_elec * eta, 0.0)
    return p_out

ACTIVE_THR = 10.0   # W — below this the motor is considered idle/glide

p_out_orig = _p_elec_to_p_out(P_orig)
p_out_sugg = _p_elec_to_p_out(P_sugg)

eta_orig_all  = motor_eta(p_out_orig)
eta_sugg_all  = motor_eta(p_out_sugg)

act_o = P_orig > ACTIVE_THR
act_s = P_sugg > ACTIVE_THR

eta_orig_act  = eta_orig_all[act_o]
eta_sugg_act  = eta_sugg_all[act_s]
P_orig_act    = P_orig[act_o]
P_sugg_act    = P_sugg[act_s]

ETA_BANDS = [
    (0.00, 0.55, '#d73027', 'Poor  < 55%'),
    (0.55, 0.70, '#fc8d59', 'Low  55–70%'),
    (0.70, 0.78, '#fee090', 'Moderate 70–78%'),
    (0.78, 0.83, '#91cf60', 'Good  78–83%'),
    (0.83, 1.00, '#1a9850', 'Peak  > 83%'),
]

def band_fracs(eta_arr):
    out = []
    for lo, hi, _, _ in ETA_BANDS:
        out.append(float(np.mean((eta_arr >= lo) & (eta_arr < hi)) * 100))
    return out

bf_orig = band_fracs(eta_orig_act)
bf_sugg = band_fracs(eta_sugg_act)


# ── Summary scalars ───────────────────────────────────────────────────────────
dur_orig_min = float(t_orig[-1]) / 60.0
dur_sugg_min = float(t_sugg[-1]) / 60.0

dist_orig_km = 14.500   # calibrated by build_demand_profile()
dist_sugg_km = 14.516   # verified in build_velocity_profile.py

mean_v_orig = dist_orig_km / (dur_orig_min / 60.0)   # km/h (overall mean)
mean_v_sugg = dist_sugg_km / (dur_sugg_min / 60.0)

max_v_orig  = float(np.max(v_orig))
max_v_sugg  = float(np.max(v_sugg))

act_frac_orig = float(np.mean(act_o)) * 100
act_frac_sugg = float(np.mean(act_s)) * 100

mean_Pdem_all_orig = float(np.mean(P_orig))
mean_Pdem_all_sugg = float(np.mean(P_sugg))
mean_Pdem_act_orig = float(np.mean(P_orig_act)) if P_orig_act.size else 0.0
mean_Pdem_act_sugg = float(np.mean(P_sugg_act)) if P_sugg_act.size else 0.0

mean_eta_orig = float(np.mean(eta_orig_act)) * 100
mean_eta_sugg = float(np.mean(eta_sugg_act)) * 100

good_peak_orig = sum(bf_orig[3:])
good_peak_sugg = sum(bf_sugg[3:])

# Strategy G results (tuned, from strategy_g_elev_compare.py run)
H2_ORIG   = 8.669   # g
H2_SUGG   = 4.281   # g
KM_M3_ORIG = TOTAL_KM / (H2_ORIG / H2_DENSITY)
KM_M3_SUGG = TOTAL_KM / (H2_SUGG / H2_DENSITY)


print(f"  Original:  {dur_orig_min:.1f} min  max {max_v_orig:.1f} km/h  "
      f"mean η={mean_eta_orig:.1f}%  active={act_frac_orig:.1f}%")
print(f"  Suggested: {dur_sugg_min:.1f} min  max {max_v_sugg:.1f} km/h  "
      f"mean η={mean_eta_sugg:.1f}%  active={act_frac_sugg:.1f}%")


# ═══════════════════════════════════════════════════════════════════════════════
# Plot 1 — Velocity vs Time
# ═══════════════════════════════════════════════════════════════════════════════
print("\nGenerating Plot 1: Velocity vs Time …")

fig1, ax = plt.subplots(figsize=(16, 5), dpi=150)

ax.plot(t_orig / 60.0, v_orig, color=C_ORIG, lw=0.7, alpha=0.85,
        label=f'Original — constant-speed GPS  (mean {mean_v_orig:.1f} km/h, max {max_v_orig:.1f} km/h)')
ax.plot(t_sugg / 60.0, v_sugg, color=C_SUGG, lw=0.7, alpha=0.85,
        label=f'Suggested — elev-aware P&G  (mean {mean_v_sugg:.1f} km/h, max {max_v_sugg:.1f} km/h)')

# Lap boundaries
for lt in lap_t_orig:
    ax.axvline(lt, color=C_ORIG, lw=0.4, ls='--', alpha=0.35)
for lt in lap_t_sugg:
    ax.axvline(lt, color=C_SUGG, lw=0.4, ls='--', alpha=0.35)

# Lap number annotations on top (use original lap boundaries)
for i, lt in enumerate(lap_t_orig):
    ax.text(lt - (T_lap_orig / 60.0) / 2, ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 34,
            f'Lap {i+1}', ha='center', va='top', fontsize=6.5, color='grey')

ax.set_xlabel('Time [min]', fontsize=11)
ax.set_ylabel('Speed [km/h]', fontsize=11)
ax.set_title('Velocity Profile — Original vs Suggested  (11 laps)', fontsize=12, fontweight='bold')
ax.legend(fontsize=9, loc='upper right')
ax.set_xlim(0, max(dur_orig_min, dur_sugg_min) * 1.01)
ax.set_ylim(bottom=0)
ax.grid(True, lw=0.25, alpha=0.6)

# Duration annotations
ax.axvline(dur_orig_min, color=C_ORIG, lw=1.0, ls=':')
ax.axvline(dur_sugg_min, color=C_SUGG, lw=1.0, ls=':')
ax.text(dur_orig_min + 0.1, 2, f'{dur_orig_min:.1f} min', color=C_ORIG, fontsize=8, va='bottom')
ax.text(dur_sugg_min + 0.1, 5, f'{dur_sugg_min:.1f} min', color=C_SUGG, fontsize=8, va='bottom')

# Add lap numbers after layout is set
fig1.tight_layout()
# Re-do lap labels now that ylim is set
ymax = ax.get_ylim()[1]
for i in range(N_LAPS):
    t_start = i * T_lap_orig / 60.0
    t_end   = (i + 1) * T_lap_orig / 60.0
    mid     = (t_start + t_end) / 2.0
    ax.text(mid, ymax * 0.97, f'L{i+1}', ha='center', va='top',
            fontsize=6.5, color=C_ORIG, alpha=0.5)

out1 = os.path.join(RESULTS_DIR, 'plot1_velocity_time.png')
fig1.savefig(out1, dpi=150, bbox_inches='tight')
plt.close(fig1)
print(f"  Saved → {out1}")


# ═══════════════════════════════════════════════════════════════════════════════
# Plot 2 — Motor Power Distribution
# ═══════════════════════════════════════════════════════════════════════════════
print("Generating Plot 2: Motor Power Distribution …")

fig2, (ax_hist, ax_band) = plt.subplots(1, 2, figsize=(16, 6), dpi=150,
                                         gridspec_kw={'width_ratios': [1.3, 1]})
fig2.suptitle('Motor Power Distribution — Original vs Suggested',
              fontsize=12, fontweight='bold')

# ── Left: P_dem histogram (ALL time, including glide zeros) ──────────────────
bins = np.concatenate([[-5], np.linspace(0, 25, 3),
                        np.linspace(25, 1300, 55)])

# Normalised to fraction of race time
hist_o, edges = np.histogram(P_orig, bins=bins, density=False)
hist_s, _     = np.histogram(P_sugg, bins=bins, density=False)
frac_o = hist_o / len(P_orig) * 100
frac_s = hist_s / len(P_sugg) * 100
width  = np.diff(edges)
centres= (edges[:-1] + edges[1:]) / 2

# Separate "glide" bin (first few bins, P<25W) from driving bins
glide_mask = centres < 25
drive_mask  = ~glide_mask

ax_hist.bar(centres[glide_mask], frac_o[glide_mask], width=width[glide_mask],
            color=C_ORIG, alpha=0.70, label='_nolegend_')
ax_hist.bar(centres[glide_mask], frac_s[glide_mask], width=width[glide_mask],
            color=C_SUGG, alpha=0.70, label='_nolegend_')

ax_hist.bar(centres[drive_mask], frac_o[drive_mask], width=width[drive_mask],
            color=C_ORIG, alpha=0.65, label=f'Original (active {act_frac_orig:.0f}%)')
ax_hist.bar(centres[drive_mask], frac_s[drive_mask], width=width[drive_mask],
            color=C_SUGG, alpha=0.65, label=f'Suggested (active {act_frac_sugg:.0f}%)')

# Shade glide zone
ax_hist.axvspan(-5, 25, color='lightgrey', alpha=0.35, zorder=0, label='Glide / zero-demand zone')
ax_hist.axvline(25, color='grey', lw=0.8, ls='--', alpha=0.6)

# Mean lines (active only)
ax_hist.axvline(mean_Pdem_act_orig, color=C_ORIG, lw=1.4, ls=':',
                label=f'Mean active: {mean_Pdem_act_orig:.0f} W')
ax_hist.axvline(mean_Pdem_act_sugg, color=C_SUGG, lw=1.4, ls=':',
                label=f'Mean active: {mean_Pdem_act_sugg:.0f} W')

# BAFANG efficiency curve on twin axis
ax_eta = ax_hist.twinx()
p_curve = np.linspace(0, 1300, 500)
# Convert P_elec to P_out for η curve (approximate: P_out ≈ P_elec × η, solve iteratively)
p_out_curve = p_curve * 0.75
for _ in range(8):
    eta_c = motor_eta(p_out_curve)
    p_out_curve = np.where(eta_c > 0.01, p_curve * eta_c, 0.0)
eta_curve = motor_eta(p_out_curve) * 100
ax_eta.plot(p_curve, eta_curve, color='black', lw=1.2, ls='-', alpha=0.5,
            label='BAFANG η')
ax_eta.set_ylabel('Motor efficiency η [%]', fontsize=9, color='black', alpha=0.6)
ax_eta.set_ylim(0, 110)
ax_eta.tick_params(axis='y', colors='grey')

# Band shading on η axis
for lo, hi, col, lbl in ETA_BANDS:
    ax_eta.axhspan(lo * 100, hi * 100, color=col, alpha=0.12, zorder=0)

ax_hist.set_xlabel('Motor electrical power demand  P_dem [W]', fontsize=10)
ax_hist.set_ylabel('Fraction of race time [%]', fontsize=10)
ax_hist.set_title('Power Demand Distribution\n(all samples, 11-lap race)', fontsize=10)
ax_hist.set_xlim(-10, 1300)
ax_hist.set_ylim(bottom=0)
ax_hist.legend(fontsize=8, loc='upper right')
ax_hist.grid(True, lw=0.25, alpha=0.5)

# ── Right: Efficiency band occupancy ────────────────────────────────────────
band_labels = [b[3] for b in ETA_BANDS]
band_colors = [b[2] for b in ETA_BANDS]
x = np.arange(len(ETA_BANDS))
bar_w = 0.35

bars_o = ax_band.bar(x - bar_w/2, bf_orig, bar_w,
                      color=band_colors, alpha=0.75, edgecolor=C_ORIG, lw=1.5,
                      label='Original')
bars_s = ax_band.bar(x + bar_w/2, bf_sugg, bar_w,
                      color=band_colors, alpha=0.95, edgecolor=C_SUGG, lw=1.5,
                      label='Suggested')

# Add value labels on bars
for bar, val in zip(bars_o, bf_orig):
    if val > 0.5:
        ax_band.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                     f'{val:.1f}%', ha='center', va='bottom', fontsize=7.5,
                     color=C_ORIG, fontweight='bold')
for bar, val in zip(bars_s, bf_sugg):
    if val > 0.5:
        ax_band.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                     f'{val:.1f}%', ha='center', va='bottom', fontsize=7.5,
                     color=C_SUGG, fontweight='bold')

# Mean η annotations
ax_band.axhline(0, color='black', lw=0.5)
ax_band.text(0.02, 0.97, f'Mean η (active)\nOriginal:   {mean_eta_orig:.1f}%\nSuggested: {mean_eta_sugg:.1f}%',
             transform=ax_band.transAxes, va='top', ha='left', fontsize=9,
             bbox=dict(boxstyle='round,pad=0.4', fc='white', ec='grey', alpha=0.8))

ax_band.set_xticks(x)
ax_band.set_xticklabels(band_labels, fontsize=8.5, rotation=15, ha='right')
ax_band.set_ylabel('% of active motor time', fontsize=10)
ax_band.set_title('Motor Efficiency Band Occupancy\n(active motor samples only)', fontsize=10)
ax_band.legend(fontsize=9, loc='upper left')
ax_band.set_ylim(0, max(max(bf_orig), max(bf_sugg)) * 1.18)
ax_band.grid(True, axis='y', lw=0.25, alpha=0.5)

# Colour the x-tick labels to match band colours
for tick, col in zip(ax_band.get_xticklabels(), band_colors):
    tick.set_color(col if col != '#fee090' else '#888800')  # avoid illegible yellow

fig2.tight_layout()
out2 = os.path.join(RESULTS_DIR, 'plot2_power_distribution.png')
fig2.savefig(out2, dpi=150, bbox_inches='tight')
plt.close(fig2)
print(f"  Saved → {out2}")


# ═══════════════════════════════════════════════════════════════════════════════
# Plot 3 — Comparison Table
# ═══════════════════════════════════════════════════════════════════════════════
print("Generating Plot 3: Comparison Table …")

rows = [
    # (Category, Metric, orig_val, sugg_val, lower_is_better)
    # ── Drive Cycle ───────────────────────────────────────────────────────────
    ('Drive Cycle', 'Total distance [km]',
     f'{dist_orig_km:.3f}', f'{dist_sugg_km:.3f}', None),
    ('Drive Cycle', 'Race duration [min]',
     f'{dur_orig_min:.1f}', f'{dur_sugg_min:.1f}', True),
    ('Drive Cycle', 'Max speed [km/h]',
     f'{max_v_orig:.1f}', f'{max_v_sugg:.1f}', None),
    ('Drive Cycle', 'Mean speed (overall) [km/h]',
     f'{mean_v_orig:.1f}', f'{mean_v_sugg:.1f}', None),
    # ── Motor Operating Points ────────────────────────────────────────────────
    ('Motor', 'Motor active fraction [%]',
     f'{act_frac_orig:.1f}', f'{act_frac_sugg:.1f}', None),
    ('Motor', 'Mean P_dem — all time [W]',
     f'{mean_Pdem_all_orig:.0f}', f'{mean_Pdem_all_sugg:.0f}', True),
    ('Motor', 'Mean P_dem — active only [W]',
     f'{mean_Pdem_act_orig:.0f}', f'{mean_Pdem_act_sugg:.0f}', None),
    ('Motor', 'Mean motor η — active [%]',
     f'{mean_eta_orig:.1f}', f'{mean_eta_sugg:.1f}', False),
    ('Motor', 'Time in Good band  (78–83%) [%]',
     f'{bf_orig[3]:.1f}', f'{bf_sugg[3]:.1f}', False),
    ('Motor', 'Time in Good+Peak  (> 78%) [%]',
     f'{good_peak_orig:.1f}', f'{good_peak_sugg:.1f}', False),
    ('Motor', 'Time in Poor band  (< 55%) [%]',
     f'{bf_orig[0]:.1f}', f'{bf_sugg[0]:.1f}', True),
    # ── Energy Management (Strategy G) ───────────────────────────────────────
    ('Strategy G', 'H₂ consumed [g]',
     f'{H2_ORIG:.3f}', f'{H2_SUGG:.3f}', True),
    ('Strategy G', 'H₂ saving vs Original',
     '—', f'{(1 - H2_SUGG/H2_ORIG)*100:.1f} %', False),
    ('Strategy G', 'km / m³ H₂',
     f'{KM_M3_ORIG:.1f}', f'{KM_M3_SUGG:.1f}', False),
    ('Strategy G', 'Efficiency gain [km/m³]',
     '—', f'+{KM_M3_SUGG - KM_M3_ORIG:.1f}', False),
    ('Strategy G', 'Final ΔSOC',
     '≈ 0.00', '≈ 0.00', None),
]

n_rows = len(rows)
fig3, ax3 = plt.subplots(figsize=(13, 0.46 * n_rows + 2.2), dpi=150)
ax3.set_xlim(0, 1)
ax3.set_ylim(0, n_rows + 1.6)
ax3.axis('off')

fig3.suptitle('Original vs Suggested — Head-to-Head Metrics',
              fontsize=14, fontweight='bold', y=0.98)

# Column layout
COL_X  = [0.00, 0.13, 0.57, 0.79]   # left edges
COL_W  = [0.13, 0.44, 0.22, 0.21]   # widths
HDRS   = ['Category', 'Metric', 'Original', 'Suggested']
ROW_H  = 0.85

# Header row
for cx, cw, hdr in zip(COL_X, COL_W, HDRS):
    ax3.add_patch(FancyBboxPatch((cx + 0.004, n_rows + 0.45), cw - 0.008, ROW_H,
                                  boxstyle='round,pad=0.02',
                                  fc='#2c3e50', ec='none',
                                  transform=ax3.transData, clip_on=False))
    ax3.text(cx + cw / 2, n_rows + 0.88, hdr, ha='center', va='center',
             fontsize=10, fontweight='bold', color='white')

# Category colour mapping
CAT_COLORS = {
    'Drive Cycle': '#dbe9f4',
    'Motor':       '#fde8e4',
    'Strategy G':  '#e8f4e8',
}

prev_cat = None
for i, (cat, metric, v_orig, v_sugg, lower_better) in enumerate(rows):
    y = n_rows - 1 - i
    row_bg = CAT_COLORS.get(cat, '#f9f9f9')

    # Alternate shade within category
    if cat != prev_cat:
        prev_cat = cat
    shade = row_bg

    # Category column
    ax3.add_patch(FancyBboxPatch((COL_X[0] + 0.004, y + 0.08), COL_W[0] - 0.008, ROW_H,
                                  boxstyle='round,pad=0.01', fc=shade, ec='none'))
    ax3.text(COL_X[0] + COL_W[0]/2, y + 0.50, cat, ha='center', va='center',
             fontsize=7.5, color='#444', style='italic')

    # Metric column
    ax3.add_patch(FancyBboxPatch((COL_X[1] + 0.004, y + 0.08), COL_W[1] - 0.008, ROW_H,
                                  boxstyle='round,pad=0.01', fc=shade, ec='none'))
    ax3.text(COL_X[1] + 0.012, y + 0.50, metric, ha='left', va='center', fontsize=9)

    # Original column
    ax3.add_patch(FancyBboxPatch((COL_X[2] + 0.004, y + 0.08), COL_W[2] - 0.008, ROW_H,
                                  boxstyle='round,pad=0.01', fc=shade, ec='none'))
    ax3.text(COL_X[2] + COL_W[2]/2, y + 0.50, v_orig, ha='center', va='center',
             fontsize=9.5, color=C_ORIG, fontweight='bold')

    # Suggested column — colour by improvement direction
    if lower_better is not None and v_sugg != '—':
        try:
            num_o = float(v_orig.replace('%', '').replace('≈', '').replace('+', '').strip())
            num_s = float(v_sugg.replace('%', '').replace('≈', '').replace('+', '').strip())
            improved = (num_s < num_o) if lower_better else (num_s > num_o)
            sugg_color = '#006400' if improved else '#8b0000'
        except Exception:
            sugg_color = C_SUGG
    elif v_sugg.startswith('+') or (not v_sugg.startswith('—') and '%' in v_sugg and
                                     not v_sugg.startswith('≈')):
        sugg_color = '#006400'
    else:
        sugg_color = C_SUGG

    ax3.add_patch(FancyBboxPatch((COL_X[3] + 0.004, y + 0.08), COL_W[3] - 0.008, ROW_H,
                                  boxstyle='round,pad=0.01', fc=shade, ec='none'))
    ax3.text(COL_X[3] + COL_W[3]/2, y + 0.50, v_sugg, ha='center', va='center',
             fontsize=9.5, color=sugg_color, fontweight='bold')

# Footer note
ax3.text(0.5, -0.35,
         'Strategy G results: tuned FF+SOC-PI (soc_glide_off = SC_SOC_0)  |  '
         'H₂ density = 89.88 g/m³ at STP  |  Total race distance = 14.5 km',
         ha='center', va='top', fontsize=7.5, color='grey',
         transform=ax3.transData)

fig3.tight_layout(rect=[0, 0.0, 1, 0.97])
out3 = os.path.join(RESULTS_DIR, 'plot3_comparison_table.png')
fig3.savefig(out3, dpi=150, bbox_inches='tight')
plt.close(fig3)
print(f"  Saved → {out3}")

print("\nAll 3 plots saved.")
