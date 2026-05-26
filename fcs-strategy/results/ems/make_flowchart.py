import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.patheffects as pe
import os

OUT = '/home/user/Ecogenium/fcs-strategy/results/ems/strategy_flowchart.png'

fig, ax = plt.subplots(figsize=(16, 26))
ax.set_xlim(0, 16); ax.set_ylim(0, 26)
ax.axis('off')
fig.patch.set_facecolor('#F8F9FB')

# ── Palette ────────────────────────────────────────────────────────────────────
P = dict(
    inp  = ('#DBEAFE','#1D4ED8'),  # input  — blue
    vdm  = ('#EDE9FE','#4C1D95'),  # vehicle dynamics — purple
    pd   = ('#FEF3C7','#B45309'),  # P_demand — amber
    best = ('#FFF7ED','#C2410C'),  # Strategy G — orange
    alt  = ('#F1F5F9','#64748B'),  # other strategies — slate
    fc   = ('#ECFDF5','#065F46'),  # FC model — emerald
    h2   = ('#F0FDF4','#166534'),  # H2 — green
    chk  = ('#FDF2F8','#701A75'),  # charge check — fuchsia
    out  = ('#F0FDF4','#14532D'),  # output — dark green
)
CA  = '#374151'   # arrow colour
CX  = 8.0         # main-pipeline x

# ── Drawing helpers ─────────────────────────────────────────────────────────────
def box(cx, cy, w, h, lines, pal, fs=9.5, bold_first=True, lw=2.2, zorder=3):
    bg, ed = pal
    r = FancyBboxPatch((cx-w/2, cy-h/2), w, h,
                        boxstyle="round,pad=0.18",
                        fc=bg, ec=ed, lw=lw, zorder=zorder)
    ax.add_patch(r)
    n = len(lines)
    step = h / (n+1)
    for i, txt in enumerate(lines):
        fw = 'bold' if (bold_first and i==0) else 'normal'
        fs_i = fs if i==0 else fs-1.2
        col = ed if i==0 else '#374151'
        ax.text(cx, cy + h/2 - step*(i+1), txt,
                ha='center', va='center', fontsize=fs_i,
                fontweight=fw, color=col, zorder=zorder+1)

def arr(x1,y1,x2,y2, col=CA, lw=2.0, style='->', rad=0.0):
    ax.annotate('', xy=(x2,y2), xytext=(x1,y1),
                arrowprops=dict(arrowstyle=style, color=col, lw=lw,
                                connectionstyle=f'arc3,rad={rad}'),
                zorder=6)

def hline(x1,x2,y, col=CA, lw=1.6, ls='-'):
    ax.plot([x1,x2],[y,y], color=col, lw=lw, ls=ls, zorder=5)

def vline(x,y1,y2, col=CA, lw=1.6, ls='-'):
    ax.plot([x,x],[y1,y2], color=col, lw=lw, ls=ls, zorder=5)

def dot(x,y, col=CA, r=0.12):
    c = plt.Circle((x,y), r, color=col, zorder=7)
    ax.add_patch(c)

def label(x,y,txt, fs=8.5, col='#374151', ha='center', va='center', bold=False):
    ax.text(x,y,txt, ha=ha, va=va, fontsize=fs,
            fontweight='bold' if bold else 'normal', color=col, zorder=8)

# ══════════════════════════════════════════════════════════════════════════════
# TITLE
# ══════════════════════════════════════════════════════════════════════════════
ax.text(CX, 25.3, 'Hydraix I SEM 2026 — EMS Strategy Flowchart',
        ha='center', va='center', fontsize=14, fontweight='bold', color='#1E293B')
ax.text(CX, 24.85, 'Velocity Profile → Vehicle Dynamics → EMS Strategy → FC Model → Mileage',
        ha='center', va='center', fontsize=9, color='#64748B', style='italic')

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — P&G GRID SEARCH (INPUT)
# ══════════════════════════════════════════════════════════════════════════════
BY1 = 23.8
box(CX, BY1, 8.0, 1.2,
    ['INPUT  —  P&G Velocity Profile  v(t)',
     'Grid search: V_HI ∈ {8.5, 9.0} m/s  ·  V_LO ∈ {6.0–7.0} m/s  ·  P_PU ∈ {500–600} W',
     'Physics simulation: _build_lap() via Newton 2nd law  ·  coast-to-stop @ 5 km/h'],
    P['inp'], fs=9.0)
arr(CX, BY1-0.6, CX, BY1-1.05)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — VEHICLE DYNAMICS MODEL
# ══════════════════════════════════════════════════════════════════════════════
BY2 = 22.35
box(CX, BY2, 10.5, 1.65,
    ['VEHICLE DYNAMICS MODEL  (run_power_plot.py  ≡  ecogenium_energy_analysis.m)',
     'F_drag = ½ρ·Cd·Af·v²    F_roll = Crr·M·g    F_grad = M·g·sin(θ)    F_acc = M·a',
     'P_wheel = (F_drag+F_roll+F_grad+F_acc)·v   →   P_motor = P_wheel / η_DT  (η_DT = 0.95)',
     'P_elec = P_motor / η_motor(P)   ←   BAFANG RM G060.1000  61-pt lookup  (η: 51–82%)'],
    P['vdm'], fs=8.8, lw=2.5)
arr(CX, BY2-0.83, CX, BY2-1.22)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — ELECTRICAL POWER DEMAND
# ══════════════════════════════════════════════════════════════════════════════
BY3 = 20.6
box(CX, BY3, 6.2, 0.85,
    ['ELECTRICAL POWER DEMAND  P_d(t)  [W]',
     'Motor-on: P_d = P_PU = 500 W  ·  Motor-off (glide/coast): P_d = 0 W'],
    P['pd'], fs=9.2, lw=2.2)

# Distribution to 4 strategies
dist_y = BY3 - 0.43
hline(1.8, 14.2, dist_y, lw=1.6)   # horizontal bus
dot(CX, dist_y)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — EMS STRATEGIES (4 parallel branches)
# ══════════════════════════════════════════════════════════════════════════════
#  x positions:  Const=2.1   PI=5.3   StratG=8.0(main)   Rule=11.5
SX = [2.1, 5.3, CX, 11.7]
SY = 18.15   # strategy box y-centre
SW, SH = 2.7, 2.5  # box width, height

strats = [
    # (x, title, lines, palette, result_text)
    (SX[0], 'CONSTANT FC', [
        'Fixed P_FC = 296 W',
        'No SOC feedback',
        'No LPF / ramp logic',
        '─────────────────',
        'km/m³: 379.9  ✓',
        'ΔSOC: +0.005  ✓',
        'SC swing: 0.048',
    ], P['alt']),
    (SX[1], 'PI ONLY', [
        'SOC-PI: K_p=3000',
        'No LPF feedforward',
        'Pure error feedback',
        '─────────────────',
        'km/m³: 377.1  ✓',
        'ΔSOC: −0.036  ✗',
        'CS not achieved',
    ], P['alt']),
    (SX[2], '★  STRATEGY G  ★', [
        'LPF feedforward τ=15s',
        '+ SOC-PI  K_p=372',
        '+ per-lap SOC offset',
        '─────────────────',
        'km/m³: 375.1  ✓',
        'ΔSOC: +0.014  ✓',
        'Best robust CS  ★',
    ], P['best']),
    (SX[3], 'RULE-BASED', [
        'Hysteresis P_hi=321W',
        'Binary high/low FC',
        'SOC band ±5%',
        '─────────────────',
        'km/m³: 378.7  ✓',
        'ΔSOC: −0.004  ✓',
        'SC swing: 0.082',
    ], P['alt']),
]

for sx, title, lines, pal in strats:
    # vertical drop from bus to box top
    vline(sx, dist_y, SY+SH/2+0.04, col=CA if sx==CX else '#94A3B8', lw=2.0 if sx==CX else 1.4)
    # arrowhead into box
    ax.annotate('', xy=(sx, SY+SH/2+0.04), xytext=(sx, SY+SH/2+0.22),
                arrowprops=dict(arrowstyle='->', color=CA if sx==CX else '#94A3B8',
                                lw=2.0 if sx==CX else 1.4), zorder=6)
    lw_box = 2.8 if sx==CX else 1.8
    box(sx, SY, SW, SH, [title]+lines, pal, fs=8.2, lw=lw_box)

# Dead-end markers for non-best strategies
for sx in [SX[0], SX[1], SX[3]]:
    vline(sx, SY-SH/2-0.04, SY-SH/2-0.55, col='#94A3B8', lw=1.2, ls='--')
    ax.text(sx, SY-SH/2-0.7, '✗  not selected', ha='center', va='center',
            fontsize=8, color='#94A3B8', style='italic', zorder=8)

# ══════════════════════════════════════════════════════════════════════════════
# BEST PATH label + arrow from Strategy G down
# ══════════════════════════════════════════════════════════════════════════════
best_bot = SY - SH/2
arr(CX, best_bot-0.06, CX, best_bot-1.05, col=P['best'][1], lw=2.5)
ax.text(CX+0.35, best_bot-0.55, 'BEST PATH', ha='left', va='center',
        fontsize=9, fontweight='bold', color=P['best'][1], zorder=8)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 5 — SC POWER BALANCE
# ══════════════════════════════════════════════════════════════════════════════
BY5 = 15.6
box(CX, BY5, 8.0, 1.0,
    ['SUPERCAPACITOR POWER BALANCE  (Maxwell 87F / 32V)',
     'P_SC = P_d − P_FC    SOC update: SOC[k+1] = SOC[k] − P_SC·Δt / E_SC',
     'Clamp: SOC ∈ [0.15, 0.95]  ·  SC floor protection: force P_FC↑ before SOC hits minimum'],
    P['fc'], fs=8.6)
arr(CX, BY5-0.5, CX, BY5-0.95)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 6 — FC ELECTROCHEMICAL MODEL
# ══════════════════════════════════════════════════════════════════════════════
BY6 = 14.1
box(CX, BY6, 9.0, 1.3,
    ['FC ELECTROCHEMICAL MODEL  —  balticFuelCells LC52.30',
     'Chamberline-Kim polarisation:  V_cell = E₀ − η_act(J) − η_ohm(J) − η_conc(J)',
     'P_FC → I_FC  (table inversion, 8000-pt lookup)   ·   P_net = N·V_cell·I − P_blower − P_BOP',
     'Ramp-rate limited: ΔP_FC ≤ 100 W/s  ·  P_FC ∈ [FC_MIN=100W, FC_MAX=1013W]'],
    P['fc'], fs=8.5, lw=2.5)
arr(CX, BY6-0.65, CX, BY6-1.05)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 7 — H2 CONSUMPTION RATE
# ══════════════════════════════════════════════════════════════════════════════
BY7 = 12.55
box(CX, BY7, 7.2, 1.1,
    ['H₂ CONSUMPTION RATE  (Faraday\'s Law)',
     'I_FC = fc_current(P_FC)  [A]',
     'ṁ_H₂ = K_H₂ × I_FC   [g/s]     K_H₂ = N·M_H₂ / (2·F) = 5.430×10⁻⁴ g/C'],
    P['h2'], fs=8.8)
arr(CX, BY7-0.55, CX, BY7-0.95)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 8 — H2 MASS INTEGRATION
# ══════════════════════════════════════════════════════════════════════════════
BY8 = 11.1
box(CX, BY8, 6.4, 0.95,
    ['H₂ MASS INTEGRATION  (11 laps × 35.5 min)',
     'M_H₂ = Σ ṁ_H₂ · Δt   [g]     Δt = 0.2 s  (5 Hz simulation)',
     'Best result: M_H₂ = 3.474 g  ·  H₂ density = 89.88 g/m³ (STP)'],
    P['h2'], fs=8.8)
arr(CX, BY8-0.48, CX, BY8-0.92)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 9 — CHARGE SUSTENANCE CHECK
# ══════════════════════════════════════════════════════════════════════════════
BY9 = 9.68
box(CX, BY9, 7.0, 1.05,
    ['CHARGE SUSTENANCE CHECK',
     'ΔSOC = SOC_final − SOC_initial       Criterion: |ΔSOC| ≤ 1.5%',
     'Bisect K_p until |ΔSOC| ≤ 0.015  ·  Best: ΔSOC = +0.014  ✓'],
    P['chk'], fs=8.8)
arr(CX, BY9-0.53, CX, BY9-0.97)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 10 — MILEAGE OUTPUT
# ══════════════════════════════════════════════════════════════════════════════
BY10 = 8.2
box(CX, BY10, 9.5, 1.45,
    ['OUTPUT  —  FUEL ECONOMY  [km/m³]',
     'km/m³ = Total distance / (M_H₂ / ρ_H₂)',
     '       = 14.5 km  /  (3.474 g / 89.88 g·m⁻³)',
     '       = 375.1  km/m³       (Strategy G — V_HI=32.4 km/h  P_PU=500 W  K_p=372)'],
    P['out'], fs=9.0, lw=2.8)

# ══════════════════════════════════════════════════════════════════════════════
# LEGEND
# ══════════════════════════════════════════════════════════════════════════════
leg_x, leg_y = 0.4, 8.8
lw_l = 1.6
for (bg,ed), lbl in [(P['inp'],'Input / Output'), (P['vdm'],'Calculation block'),
                      (P['best'],'Best strategy path (Strategy G)'), (P['alt'],'Alternative strategies')]:
    r = FancyBboxPatch((leg_x,leg_y-0.22), 0.55, 0.4, boxstyle='round,pad=0.05',
                        fc=bg, ec=ed, lw=lw_l, zorder=9)
    ax.add_patch(r)
    ax.text(leg_x+0.7, leg_y-0.02, lbl, ha='left', va='center',
            fontsize=8.5, color='#1E293B', zorder=10)
    leg_y -= 0.62

plt.tight_layout(pad=0.5)
fig.savefig(OUT, dpi=160, bbox_inches='tight', facecolor='#F8F9FB')
plt.close(fig)
print(f'Saved → {OUT}')
