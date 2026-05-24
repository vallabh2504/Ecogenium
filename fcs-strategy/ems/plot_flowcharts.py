"""
EMS Flowchart Generator — Hydraix I / SEM 2026
Produces two figures:
  1. System architecture: Velocity → Dynamics → Motor → EMS → FC/SC → Outputs
  2. Strategy A deep dive: per-timestep algorithm + 2D table + bisection tuning
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import os

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'results', 'ems')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Colour palette ─────────────────────────────────────────────────────────────
C_INPUT   = '#1565C0'   # deep blue       — raw inputs
C_PROC    = '#2E7D32'   # deep green      — physics / processing
C_EMS     = '#BF360C'   # deep orange-red — EMS control
C_FC      = '#00695C'   # dark teal       — FC model
C_SC      = '#E65100'   # dark amber      — SC model
C_OUT     = '#4A148C'   # deep purple     — outputs
C_PARAM   = '#37474F'   # blue-grey       — parameter annotation
C_TUNE    = '#AD1457'   # dark pink       — tuning loop
C_TABLE   = '#1A237E'   # indigo          — lookup table
C_ARROW   = '#263238'   # near-black      — arrows
C_FB      = '#558B2F'   # olive green     — feedback arrows

ALPHA = 0.88


def fbox(ax, cx, cy, w, h, lines, bg, fg='white', fs=8.5, lw=1.4, style='round,pad=0.12'):
    p = FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                       boxstyle=style, linewidth=lw,
                       edgecolor='white', facecolor=bg, alpha=ALPHA, zorder=3)
    ax.add_patch(p)
    text = '\n'.join(lines)
    ax.text(cx, cy, text, ha='center', va='center', fontsize=fs, color=fg,
            fontweight='bold', zorder=4, multialignment='center',
            linespacing=1.45)


def arrow(ax, x1, y1, x2, y2, label='', col=C_ARROW, lw=1.6, rad=0.0, ls='-'):
    style = f'arc3,rad={rad}'
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=col, lw=lw,
                                connectionstyle=style,
                                linestyle=ls),
                zorder=5)
    if label:
        mx = (x1 + x2) / 2 + 0.06
        my = (y1 + y2) / 2
        ax.text(mx, my, label, ha='left', va='center',
                fontsize=7.5, color=col, style='italic', zorder=6)


# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 1 — System Architecture
# ══════════════════════════════════════════════════════════════════════════════
fig1, ax1 = plt.subplots(figsize=(13, 17))
ax1.set_xlim(0, 13)
ax1.set_ylim(0, 17)
ax1.axis('off')
fig1.patch.set_facecolor('#ECEFF1')
ax1.set_facecolor('#ECEFF1')

fig1.suptitle(
    'Hydraix I EMS — System Architecture\n'
    'Silesia Ring · Shell Eco-marathon 2026',
    fontsize=14, fontweight='bold', color='#1A237E', y=0.98)

# ── Row 1: Inputs ──────────────────────────────────────────────────────────────
fbox(ax1, 3.3, 15.6, 5.4, 1.5,
     ['GPS TELEMETRY', 'Race 4 canonical drive cycle',
      '11 laps · 14.5 km · 35 min',
      'Resampled 5 Hz + 10 s smooth'],
     C_INPUT, fs=8)

fbox(ax1, 9.7, 15.6, 5.0, 1.5,
     ['VEHICLE PARAMETERS',
      'm = 175 kg  |  Cd = 0.15  |  Af = 0.8 m²',
      'CRR = 0.006  |  ρ = 1.225 kg/m³',
      'η_drivetrain = 0.95'],
     C_PARAM, fg='#ECEFF1', fs=8)

# ── Row 2: Vehicle Dynamics Model ─────────────────────────────────────────────
fbox(ax1, 6.5, 13.5, 10.8, 1.7,
     ['VEHICLE DYNAMICS MODEL',
      'P_rolling = CRR · m · g · v(t)',
      'P_aero = ½ · Cd · Af · ρ · v³(t)',
      'P_accel = m · a(t) · v(t)     P_gradient = m · g · sin(θ) · v(t)',
      'P_wheel(t) = P_rolling + P_aero + P_accel + P_gradient'],
     C_PROC, fs=8.2)

arrow(ax1, 3.3, 14.85, 5.0, 14.38, 'v(t), θ(t)', col=C_INPUT)
arrow(ax1, 9.7, 14.85, 8.0, 14.38, 'parameters', col=C_PARAM)

# ── Row 3: Motor Model ────────────────────────────────────────────────────────
fbox(ax1, 6.5, 11.55, 10.8, 1.55,
     ['MOTOR MODEL — BAFANG RM G060.1000 6T 90A 48V',
      'P_motor = P_wheel / η_drivetrain   (when P_wheel > 0)',
      'η_motor = lookup(P_motor)   [61-point factory test table: 22% – 83.4%]',
      'P_dem(t) = P_motor / η_motor       [electrical demand at bus]'],
     C_PROC, fs=8.2)

arrow(ax1, 6.5, 12.62, 6.5, 12.33, 'P_wheel(t)', col=C_PROC)

# ── Row 4: EMS Control Block ──────────────────────────────────────────────────
fbox(ax1, 6.5, 9.35, 10.8, 1.95,
     ['EMS CONTROL BLOCK — Strategy A: Heuristic 2D Lookup Table',
      'u(t) = [P_dem(t) − P_fc_prev] / E_sc        [Normalised SC power request, 1/s]',
      'P_fc(t) = TABLE[u(t), SOC(t)]                [Bilinear interpolation, 21×18 nodes]',
      'Apply FC ramp rate limit: |ΔP_fc / Δt| ≤ 100 W/s',
      'Clamp: P_fc ∈ [100 W, 1013 W]     →     P_sc(t) = P_dem(t) − P_fc(t)'],
     C_EMS, fs=8.2)

arrow(ax1, 6.5, 10.78, 6.5, 10.33, 'P_dem(t)', col=C_PROC)

# ── Feedback: SOC → EMS ───────────────────────────────────────────────────────
arrow(ax1, 10.0, 7.25, 10.0, 8.38, 'SOC(t) feedback', col=C_FB, lw=1.4, ls='--')

# ── Row 5: FC Model (left) and SC Model (right) ───────────────────────────────
fbox(ax1, 3.3, 7.0, 5.4, 2.2,
     ['FC SYSTEM MODEL',
      'balticFuelCells LC 52.30',
      '─────────────────────────',
      'Chamberlin-Kim polarisation:',
      'V_cell = E₀ − η_act − η_ohm − η_conc',
      'BOP: blower (var-speed) + pump + ECU',
      'P_fc_net = P_gross − P_BOP(I)',
      'I_fc = P_fc_net⁻¹  [polarisation inverse]',
      'ṁ_H₂ = K_H2 · I_fc  [Faraday\'s law]'],
     C_FC, fs=7.8)

fbox(ax1, 9.7, 7.0, 5.4, 2.2,
     ['SUPERCAPACITOR MODEL',
      '100 F / 32 V  (Maxwell BMOD0058×3)',
      '─────────────────────────────────',
      'P_sc(t) = P_dem(t) − P_fc(t)',
      'SOC = [V² − V_min²] / [V_max² − V_min²]',
      'E_sc = ½C(V_max²−V_min²) = 10.67 Wh',
      'ΔSOC = −P_sc_eff · Δt / E_sc_J',
      'η_sc = 0.95 (round-trip)  |  R_esr = 15 mΩ',
      'SOC ∈ [0.15, 0.95]'],
     C_SC, fg='white', fs=7.8)

arrow(ax1, 5.55, 8.38, 3.85, 8.1, 'P_fc setpoint', col=C_EMS)
arrow(ax1, 7.45, 8.38, 9.15, 8.1, 'P_sc = P_dem − P_fc', col=C_EMS)

# ── Row 6: Outputs ────────────────────────────────────────────────────────────
fbox(ax1, 3.3, 4.6, 5.4, 1.55,
     ['FC OUTPUTS',
      'I_fc(t)  [A]',
      'ṁ_H₂(t)  [g/s]',
      'H₂ consumed = 8.204 g / race'],
     C_FC, fs=8.2)

fbox(ax1, 9.7, 4.6, 5.4, 1.55,
     ['SC OUTPUTS',
      'SOC(t)  profile',
      'P_sc(t) split  [W]',
      'SOC_start = SOC_end = 0.600'],
     C_SC, fg='white', fs=8.2)

arrow(ax1, 3.3, 5.9, 3.3, 5.38, '', col=C_FC)
arrow(ax1, 9.7, 5.9, 9.7, 5.38, '', col=C_SC)

# ── Row 7: Combined result ─────────────────────────────────────────────────────
fbox(ax1, 6.5, 2.8, 10.8, 1.55,
     ['RACE SIMULATION RESULT — 11 Laps · 14.5 km · 35 min',
      'Total H₂ consumed: 8.204 g  |  ΔSOC = +0.006  ✓ Charge sustained',
      'Mean P_fc = 267.8 W  |  Peak SC discharge = 939 W  |  SOC range: 0.600 – 0.606'],
     C_OUT, fs=8.5)

arrow(ax1, 3.3, 3.83, 4.6, 3.58, '', col=C_FC)
arrow(ax1, 9.7, 3.83, 8.4, 3.58, '', col=C_SC)

# ── Legend ────────────────────────────────────────────────────────────────────
legend_items = [
    mpatches.Patch(color=C_INPUT,  label='Raw input data'),
    mpatches.Patch(color=C_PROC,   label='Physics / processing model'),
    mpatches.Patch(color=C_EMS,    label='EMS control block'),
    mpatches.Patch(color=C_FC,     label='Fuel-cell system model'),
    mpatches.Patch(color=C_SC,     label='Supercapacitor model'),
    mpatches.Patch(color=C_OUT,    label='Simulation outputs'),
]
ax1.legend(handles=legend_items, loc='lower right', fontsize=8,
           framealpha=0.85, ncol=2, bbox_to_anchor=(1.0, 0.0))

out1 = os.path.join(OUT_DIR, 'flowchart_system_architecture.png')
fig1.savefig(out1, dpi=160, bbox_inches='tight', facecolor=fig1.get_facecolor())
print(f'Saved: {out1}')
plt.close(fig1)


# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 2 — Strategy A Deep Dive
# ══════════════════════════════════════════════════════════════════════════════
fig2, ax2 = plt.subplots(figsize=(14, 20))
ax2.set_xlim(0, 14)
ax2.set_ylim(0, 20)
ax2.axis('off')
fig2.patch.set_facecolor('#ECEFF1')
ax2.set_facecolor('#ECEFF1')

fig2.suptitle(
    'Strategy A — Heuristic 2D Lookup Table: Algorithm & Tuning\n'
    'Hydraix I EMS · Shell Eco-marathon 2026',
    fontsize=14, fontweight='bold', color='#1A237E', y=0.99)

# ── LEFT COLUMN: per-timestep algorithm (x ~ 4.5) ────────────────────────────

# Step 0: Start
fbox(ax2, 4.5, 18.8, 5.0, 0.7,
     ['START OF TIMESTEP  k  (Δt = 0.2 s, 5 Hz)'],
     '#263238', fs=8.5)

# Step 1: Read state
fbox(ax2, 4.5, 17.6, 5.6, 1.2,
     ['READ SYSTEM STATE',
      'P_dem(t)   — from vehicle dynamics+motor model',
      'SOC(t)     — from supercapacitor state',
      'P_fc_prev  — FC output at previous step'],
     C_INPUT, fs=8)

arrow(ax2, 4.5, 18.45, 4.5, 18.2, col=C_ARROW)

# Step 2: Compute u
fbox(ax2, 4.5, 16.2, 5.6, 1.1,
     ['COMPUTE NORMALISED SC POWER REQUEST',
      'u(t)  =  [P_dem(t) − P_fc_prev]  /  E_sc_J',
      '        where  E_sc_J = 38 400 J  (= 10.67 Wh)'],
     C_PARAM, fg='#ECEFF1', fs=8.2)

arrow(ax2, 4.5, 17.0, 4.5, 16.75, 'u > 0: SC must discharge\nu < 0: FC over-produces, SC charges',
      col=C_PARAM)

# Step 3: Clamp
fbox(ax2, 4.5, 14.95, 5.6, 1.0,
     ['CLAMP INPUTS TO TABLE BOUNDS',
      'u   ← clip(u,   −0.050,  +0.050)  [1/s]',
      'SOC ← clip(SOC,   0.10,    0.95)'],
     C_PARAM, fg='#ECEFF1', fs=8.2)

arrow(ax2, 4.5, 15.65, 4.5, 15.45, col=C_ARROW)

# Step 4: Lookup table
fbox(ax2, 4.5, 13.45, 5.6, 1.5,
     ['2D BILINEAR TABLE LOOKUP',
      'P_fc_new = TABLE[u, SOC]',
      'Table axes: u ∈ 21 pts · SOC ∈ 18 pts',
      'Table values built from:',
      '  P_base + ΔP_soc + ΔP_rate  (see right panel)'],
     C_TABLE, fs=8.2)

arrow(ax2, 4.5, 14.45, 4.5, 14.2, col=C_ARROW)

# Step 5: Ramp rate
fbox(ax2, 4.5, 12.1, 5.6, 1.1,
     ['APPLY FC RAMP-RATE LIMIT',
      'P_fc ← clamp(P_fc_new,',
      '             P_fc_prev − 20 W,   P_fc_prev + 20 W)',
      '       (100 W/s × 0.2 s step = 20 W/step)'],
     C_EMS, fs=8.2)

arrow(ax2, 4.5, 12.7, 4.5, 12.65, col=C_ARROW)

# Step 6: Saturate
fbox(ax2, 4.5, 10.9, 5.6, 0.95,
     ['HARD SATURATION',
      'P_fc ← clip(P_fc,  100 W,  1013 W)',
      'FC cannot go below 100 W (membrane protection)'],
     C_EMS, fs=8.2)

arrow(ax2, 4.5, 11.58, 4.5, 11.38, col=C_ARROW)

# Step 7: Power split
fbox(ax2, 4.5, 9.75, 5.6, 0.95,
     ['POWER SPLIT',
      'P_sc(t) = P_dem(t) − P_fc(t)',
      'P_sc > 0 → SC discharging   |   P_sc < 0 → SC charging'],
     C_SC, fg='white', fs=8.2)

arrow(ax2, 4.5, 10.38, 4.5, 10.23, col=C_ARROW)

# Step 7b: SC floor check
fbox(ax2, 4.5, 8.6, 5.6, 1.05,
     ['SOC FLOOR PROTECTION',
      'if P_sc > 0  AND  SOC → SC_SOC_MIN (0.15):',
      '    P_fc ← min(1013, P_dem)  [force FC to max]',
      '    P_sc ← max(0,  P_dem − P_fc)'],
     '#B71C1C', fs=8.2)

arrow(ax2, 4.5, 9.28, 4.5, 9.13, 'SOC near floor?', col=C_ARROW)

# Step 8: SOC update
fbox(ax2, 4.5, 7.3, 5.6, 1.15,
     ['UPDATE SUPERCAPACITOR SOC',
      'P_eff = P_sc/η_sc  (discharge)   or   P_sc·η_sc  (charge)',
      'SOC_new = clip( SOC − P_eff · Δt / E_sc_J,  0.15,  0.95 )',
      'V_sc = √(V_min² + SOC · (V_max² − V_min²))'],
     C_SC, fg='white', fs=8.2)

arrow(ax2, 4.5, 8.08, 4.5, 7.93, col=C_ARROW)

# Step 9: FC state
fbox(ax2, 4.5, 6.1, 5.6, 1.05,
     ['COMPUTE FC OPERATING STATE',
      'I_fc = polarisation_inverse(P_fc)    [from 8000-pt lookup]',
      'ṁ_H₂ = K_H2 · I_fc                   [Faraday\'s law]',
      'K_H2 = N·M_H2/(2F) = 5.430×10⁻⁴ g/C'],
     C_FC, fs=8.2)

arrow(ax2, 4.5, 6.83, 4.5, 6.63, col=C_ARROW)

# Step 10: Accumulate
fbox(ax2, 4.5, 5.05, 5.6, 0.85,
     ['ACCUMULATE',
      'H2_total += ṁ_H₂ · Δt     SOC ← SOC_new     k ← k + 1'],
     '#263238', fs=8.2)

arrow(ax2, 4.5, 5.63, 4.5, 5.48, col=C_ARROW)

# Step 11: End / next lap
fbox(ax2, 4.5, 4.05, 5.6, 0.8,
     ['END OF TIMESTEP  →  repeat for all 10 505 steps (11 laps × 955 pts/lap)'],
     '#263238', fs=8.2)

arrow(ax2, 4.5, 4.63, 4.5, 4.45, col=C_ARROW)

# Loop-back arrow (k → k+1)
ax2.annotate('', xy=(1.7, 17.6), xytext=(1.7, 4.05),
             arrowprops=dict(arrowstyle='->', color='#546E7A', lw=1.3,
                             connectionstyle='arc3,rad=0'))
ax2.plot([1.7, 2.2], [4.05, 4.05], color='#546E7A', lw=1.3)
ax2.plot([1.7, 2.2], [17.6, 17.6], color='#546E7A', lw=1.3)
ax2.text(1.3, 10.8, 'k → k+1\nnext\nstep', ha='center', va='center',
         fontsize=7.5, color='#546E7A', style='italic')


# ── RIGHT COLUMN: table construction + tuning loop ────────────────────────────

# Table construction panel
fbox(ax2, 10.8, 17.3, 6.0, 2.9,
     ['2D LOOKUP TABLE CONSTRUCTION',
      '───────────────────────────────────────',
      'Axes:',
      '  u-axis : 21 pts, −0.050 → +0.050 [1/s]',
      '  SOC-axis: 18 pts,   0.10 → 0.95  [−]',
      '',
      'At each node (u_i, SOC_j):',
      '',
      '  P_base      = 259 W  (race avg demand)',
      '',
      '  ΔP_soc  =  K_soc × (0.60 − SOC_j)',
      '      SOC low  → +ΔP  (FC works harder)',
      '      SOC high → −ΔP  (FC backs off)',
      '',
      '  ΔP_rate =  K_rate × (|u_i| − 0.020) × sign(u_i)',
      '             only when |u_i| > 0.020',
      '      K_rate = 500 W·s  (feedforward)',
      '',
      '  TABLE[i,j] = clip(P_base + ΔP_soc + ΔP_rate,',
      '                     100 W,  1013 W)'],
     C_TABLE, fs=7.8)

# Tuning loop panel
fbox(ax2, 10.8, 12.4, 6.0, 3.9,
     ['BISECTION TUNING LOOP  (outer loop)',
      '────────────────────────────────────────',
      'Goal: |ΔSOC| = |SOC_final − SOC_initial| < 0.01',
      '',
      'INIT:  K_soc = 200 W/SOC',
      '       lo = 50,  hi = 600',
      '',
      'EACH ITERATION:',
      '  1. Build TABLE with current K_soc',
      '  2. Simulate full 11-lap race',
      '  3. Compute ΔSOC = SOC(end) − 0.600',
      '',
      '  ΔSOC < −0.01  →  lo ← K_soc  (increase)',
      '  ΔSOC > +0.01  →  hi ← K_soc  (decrease)',
      '  K_soc ← (lo + hi) / 2',
      '',
      'Iteration trace:',
      '  1:  K=200.0   ΔSOC=+0.094  → decrease',
      '  2:  K=125.0   ΔSOC=+0.043  → decrease',
      '  3:  K= 87.5   ΔSOC=+0.017  → decrease',
      '  4:  K= 68.8   ΔSOC=+0.006  ✓ CONVERGED',
      '',
      'Tuned value: K_soc = 68.8 W per unit SOC'],
     C_TUNE, fs=7.8)

# Results annotation panel
fbox(ax2, 10.8, 7.8, 6.0, 3.6,
     ['STRATEGY A — FINAL OPERATING ENVELOPE',
      '───────────────────────────────────────',
      '',
      'SOC profile:',
      '  SOC_start = 0.600',
      '  SOC_end   = 0.606   (ΔSOC = +0.006 ✓)',
      '  SOC_min   = 0.600   (never drops below start)',
      '  SOC_max   = 0.606   (tightly regulated)',
      '  → SC oscillation ≈ 0.006 SOC units = 64 J',
      '',
      'FC power range (from lookup table):',
      '  Minimum  ≈  235 W  (SOC near max, u<0)',
      '  Nominal  ≈  259 W  (SOC = ref, u = 0)',
      '  Maximum  ≈  305 W  (SOC near min, u>0)',
      '  Floor    =  100 W  (hard clamp)',
      '  Mean     =  267.8 W (race average)',
      '',
      'SC power range:',
      '  Charging max  ≈ −305 W  (glide + FC surplus)',
      '  Discharging   ≈ +939 W  (acceleration peak)',
      '',
      'H₂ consumed: 8.204 g  |  Race: 35 min'],
     C_EMS, fs=7.8)

# Physical meaning annotation
fbox(ax2, 10.8, 4.15, 6.0, 2.75,
     ['PHYSICAL INSIGHT',
      '──────────────────────────────────',
      'K_soc = 68.8 W/SOC is very small.',
      'Max SOC correction = 68.8 × 0.45 = ±31 W.',
      '',
      'The table is essentially FLAT at 259 W.',
      'Strategy A converges to: FC = avg demand,',
      'SC buffers ALL transients above/below.',
      '',
      'This is the same solution ECMS finds',
      'analytically (s = 0.79 → P_fc* = 268 W).',
      'The heuristic table rediscovered the',
      'Pontryagin optimal through bisection.'],
     '#1B5E20', fs=7.8)

# Arrows connecting right-column panels
arrow(ax2, 10.8, 15.85, 10.8, 14.34, 'TABLE feeds into strategy_fn', col=C_TABLE, lw=1.3)
arrow(ax2, 10.8, 10.44, 10.8, 9.60, 'tuned K_soc → final table', col=C_TUNE, lw=1.3)
arrow(ax2, 10.8, 6.0,  10.8, 5.53,  'interpretation', col=C_EMS, lw=1.3)

# Connector from left column to right-column table
arrow(ax2, 7.3, 13.45, 7.8, 14.92, 'uses table', col=C_TABLE, lw=1.3, rad=-0.2)

# Connector from tuning to main flow (K_soc determines table)
arrow(ax2, 7.82, 12.4, 7.3, 14.45, 'K_soc set\nby tuning', col=C_TUNE, lw=1.3, rad=0.2)

# ── Legend ────────────────────────────────────────────────────────────────────
legend_items2 = [
    mpatches.Patch(color=C_INPUT,  label='System state inputs'),
    mpatches.Patch(color=C_TABLE,  label='Lookup table (2D bilinear)'),
    mpatches.Patch(color=C_EMS,    label='FC constraints & saturation'),
    mpatches.Patch(color=C_SC,     label='Supercapacitor update'),
    mpatches.Patch(color=C_FC,     label='FC electrochemical model'),
    mpatches.Patch(color=C_TUNE,   label='Bisection tuning loop'),
    mpatches.Patch(color='#1B5E20', label='Physical insight'),
]
ax2.legend(handles=legend_items2, loc='lower right', fontsize=7.5,
           framealpha=0.85, ncol=2, bbox_to_anchor=(1.0, 0.0))

out2 = os.path.join(OUT_DIR, 'flowchart_strategy_a_deep_dive.png')
fig2.savefig(out2, dpi=160, bbox_inches='tight', facecolor=fig2.get_facecolor())
print(f'Saved: {out2}')
plt.close(fig2)

print('\nBoth flowcharts generated successfully.')
