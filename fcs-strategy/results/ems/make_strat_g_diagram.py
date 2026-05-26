import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np, os

OUT = '/home/user/Ecogenium/fcs-strategy/results/ems/strategy_g_detail.png'

fig = plt.figure(figsize=(20, 26))
fig.patch.set_facecolor('#F8FAFC')
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 20); ax.set_ylim(0, 26); ax.axis('off')

# ── Palettes ───────────────────────────────────────────────────────────────────
OR  = ('#FFF7ED','#C2410C','#C2410C')   # orange  – Strategy G highlight
BL  = ('#EFF6FF','#1D4ED8','#1D4ED8')   # blue    – inputs
GR  = ('#F0FDF4','#15803D','#15803D')   # green   – outputs / good
PU  = ('#FAF5FF','#7E22CE','#7E22CE')   # purple  – internal states
SL  = ('#F1F5F9','#475569','#475569')   # slate   – neutral
RD  = ('#FFF1F2','#BE123C','#BE123C')   # red     – constraints
YL  = ('#FEFCE8','#A16207','#A16207')   # yellow  – signals
CA  = '#334155'   # arrow

# ── Helpers ────────────────────────────────────────────────────────────────────
def box(cx,cy,w,h,lines,pal,fs=9,lw=2.2,zorder=4):
    bg,ed,tc=pal
    r=FancyBboxPatch((cx-w/2,cy-h/2),w,h,boxstyle='round,pad=0.15',
                     fc=bg,ec=ed,lw=lw,zorder=zorder)
    ax.add_patch(r)
    n=len(lines); step=h/(n+1)
    for i,txt in enumerate(lines):
        fw='bold' if i==0 else 'normal'
        fs_i=fs if i==0 else fs-1.3
        col=ed if i==0 else '#374151'
        ax.text(cx,cy+h/2-step*(i+1),txt,ha='center',va='center',
                fontsize=fs_i,fontweight=fw,color=col,zorder=zorder+1)

def sumblock(cx,cy,r_=0.38,col='#1D4ED8'):
    c=plt.Circle((cx,cy),r_,fc='white',ec=col,lw=2.2,zorder=6)
    ax.add_patch(c); ax.text(cx,cy,'Σ',ha='center',va='center',
                             fontsize=14,color=col,fontweight='bold',zorder=7)

def arr(x1,y1,x2,y2,col=CA,lw=1.8,style='->',rad=0.,ls='-'):
    ax.annotate('',xy=(x2,y2),xytext=(x1,y1),
                arrowprops=dict(arrowstyle=style,color=col,lw=lw,
                                linestyle=ls,
                                connectionstyle=f'arc3,rad={rad}'),zorder=8)

def label(x,y,t,fs=8.5,col='#334155',ha='center',va='center',bold=False,angle=0):
    ax.text(x,y,t,ha=ha,va=va,fontsize=fs,fontweight='bold' if bold else 'normal',
            color=col,zorder=9,rotation=angle)

def brace_label(x,y,t,col='#64748B',fs=8.2):
    ax.text(x,y,t,ha='center',va='top',fontsize=fs,color=col,
            style='italic',zorder=9)

# ══════════════════════════════════════════════════════════════════════════════
# TITLE SECTION
# ══════════════════════════════════════════════════════════════════════════════
ax.text(10,25.55,'Strategy G — LPF Feedforward + SOC-PI + Per-Lap Offset',
        ha='center',va='center',fontsize=15,fontweight='bold',color='#1E293B')
ax.text(10,25.1,'Internal control architecture, signal flow, and Simulink mapping',
        ha='center',va='center',fontsize=9.5,color='#64748B',style='italic')
ax.axhline(24.75,color='#CBD5E1',lw=1.2,xmin=0.02,xmax=0.98)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION A — CONTROL BLOCK DIAGRAM
# ══════════════════════════════════════════════════════════════════════════════
ax.text(0.4,24.5,'A.  CONTROL BLOCK DIAGRAM',ha='left',va='center',
        fontsize=11,fontweight='bold',color='#C2410C')

# --- Inputs (left side, y ≈ 22) ---
box(1.8,22.4,2.9,0.75,['P_d(t)  [W]','Motor electrical demand'],BL,fs=9)
box(1.8,19.0,2.9,0.75,['SOC(t)','SC state of charge'],BL,fs=9)
box(1.8,21.0,2.9,0.75,['lap index  l(t)','per-lap trigger'],SL,fs=9)

# --- Motor-off gate ---
box(5.6,22.4,2.1,0.75,['MOTOR-OFF\nGATE','Pd<5W → 0'],SL,fs=8.5)
arr(3.25,22.4,4.55,22.4)
label(3.9,22.55,'P_d',fs=8)

# --- LPF block ---
box(8.8,22.4,2.6,1.05,
    ['LPF  (τ=15 s)','α=exp(−Δt/τ)=0.987',
     'x[k]=α·x[k−1]+(1−α)·P_d'],PU,fs=8.5)
arr(6.7,22.4,7.5,22.4)
label(7.1,22.55,'P_d (gated)',fs=7.5)

# --- SOC error node ---
# SOC reference
ax.text(5.1,19.55,'SOC_ref=0.60',ha='center',va='center',fontsize=8,color='#15803D',
        fontweight='bold')
# error summer
sumblock(5.6,19.0,col='#15803D')
arr(3.25,19.0,5.22,19.0)   # SOC(t) → Σ
label(5.9,19.35,'+',fs=9,col='#15803D',bold=True)
label(5.9,18.65,'−',fs=11,col='#BE123C',bold=True)
arr(5.6,19.55,5.6,19.38)   # SOC_ref ↓
label(4.85,18.78,'e = SOC_ref − SOC',fs=8,col='#15803D')

# --- Proportional ---
box(8.8,20.0,2.3,0.75,['×  K_p  =  372','Proportional gain'],GR,fs=8.8)
arr(5.98,19.0,6.8,19.0)
# line from error to split
ax.plot([6.8,6.8],[18.0,19.0],color=CA,lw=1.8,zorder=5)
ax.plot([6.8,7.65],[19.0,19.0],color=CA,lw=1.8,zorder=5)  # to P block
ax.plot([6.8,7.65],[18.0,18.0],color=CA,lw=1.8,zorder=5)  # to I block
arr(7.65,19.0,7.65,19.0)  # dummy
ax.annotate('',xy=(7.65,20.0),xytext=(6.8,20.0),
            arrowprops=dict(arrowstyle='->',color=CA,lw=1.8),zorder=8)
ax.annotate('',xy=(7.65,18.0),xytext=(6.8,18.0),
            arrowprops=dict(arrowstyle='->',color=CA,lw=1.8),zorder=8)
ax.plot([6.8,6.8],[18.0,20.0],color=CA,lw=1.8,zorder=5)

# --- Integral ---
box(8.8,18.0,2.3,0.85,['∫ e dt  ×  K_i=2','Anti-windup clamp',
                         '|I|≤75 W'],GR,fs=8.5)

# --- Per-lap offset block ---
box(8.8,21.2,2.6,0.82,
    ['PER-LAP OFFSET  δ','δ+=0.3·e_SOC·E_SC/186',
     'clip(δ, −200, +200 W)'],YL,fs=8.4)
arr(5.6,19.38,5.6,17.0)  # SOC error line down
ax.plot([5.6,5.6],[17.0,21.2],color='#A16207',lw=1.5,ls='--',zorder=5)
ax.plot([5.6,7.5],[21.2,21.2],color='#A16207',lw=1.5,ls='--',zorder=5)
ax.annotate('',xy=(7.5,21.2),xytext=(5.6,21.2),
            arrowprops=dict(arrowstyle='->',color='#A16207',lw=1.5,linestyle='dashed'),zorder=8)
arr(3.25,21.0,5.6,21.0,col='#A16207',lw=1.4,ls='--')  # lap trigger
ax.plot([5.6,5.6],[21.0,21.2],color='#A16207',lw=1.4,ls='--',zorder=5)

# --- Summation node ---
sumblock(12.5,20.4,col='#C2410C')
# LPF → Σ
arr(10.1,22.4,12.5,22.4,col='#7E22CE',lw=1.8)
ax.plot([12.5,12.5],[22.4,20.78],color='#7E22CE',lw=1.8,zorder=5)
# Kp → Σ
arr(10.1,20.0,12.5,20.0,col='#15803D',lw=1.8)
ax.plot([12.5,12.5],[20.0,20.02],color='#15803D',lw=1.8,zorder=5)
# Ki → Σ
arr(10.1,18.0,12.5,18.0,col='#15803D',lw=1.8)
ax.plot([12.5,12.5],[18.0,20.02],color='#15803D',lw=1.8,zorder=5)
# Offset → Σ
arr(10.1,21.2,12.5,21.2,col='#A16207',lw=1.5,ls='--')
ax.plot([12.5,12.5],[21.2,20.78],color='#A16207',lw=1.5,ls='--',zorder=5)
# Labels at Σ
label(12.05,22.0,'+',fs=9,col='#7E22CE',bold=True)
label(12.05,20.3,'+',fs=9,col='#15803D',bold=True)
label(12.05,18.3,'+',fs=9,col='#15803D',bold=True)
label(13.0,21.1,'+',fs=9,col='#A16207',bold=True)

# --- Ramp-rate limiter ---
box(14.8,20.4,2.2,0.8,['FC RAMP LIMITER','±100 W/s  (±20 W/step)'],RD,fs=8.8)
arr(12.88,20.4,13.7,20.4)

# --- Saturation ---
box(17.2,20.4,2.2,0.8,['SATURATION','[0,  1013] W'],RD,fs=8.8)
arr(15.9,20.4,16.1,20.4)

# --- Output ---
box(19.0,20.4,1.6,0.75,['P_FC(t)'],GR,fs=9.5,lw=2.5)
arr(18.3,20.4,18.2,20.4)

# --- SC SOC update feedback loop ---
ax.plot([19.0,19.0],[20.02,16.8],color='#BE123C',lw=1.5,ls='-.',zorder=5)
ax.plot([1.8,19.0],[16.8,16.8],color='#BE123C',lw=1.5,ls='-.',zorder=5)
arr(1.8,16.8,1.8,18.63,col='#BE123C',lw=1.5,ls='dashed')
label(10.5,16.5,'SOC feedback  (SC model → SOC[k+1] = f(SOC[k], P_SC, Δt))',
      fs=8,col='#BE123C')
label(0.5,17.7,'SOC(t)',fs=8,col='#BE123C',bold=True,angle=90)

# Signal annotations
label(11.0,22.75,'LPF(P_d) — smoothed FC setpoint',fs=7.8,col='#7E22CE')
label(11.0,20.25,'K_p·e — proportional correction',fs=7.8,col='#15803D')
label(11.0,17.65,'K_i·∫e — integral (drift removal)',fs=7.8,col='#15803D')
label(11.0,21.5,'δ — per-lap long-range correction',fs=7.8,col='#A16207')

ax.axhline(16.2,color='#CBD5E1',lw=1.0,xmin=0.02,xmax=0.98)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION B — STEP-BY-STEP SIGNAL FLOW
# ══════════════════════════════════════════════════════════════════════════════
ax.text(0.4,16.0,'B.  STEP-BY-STEP SIGNAL FLOW  (each 0.2 s timestep)',
        ha='left',va='center',fontsize=11,fontweight='bold',color='#C2410C')

steps=[
    ('①',8.8, 15.3,'LPF UPDATE\n(feedforward)',
     ['x[k] = 0.987 · x[k−1]  +  0.013 · P_d[k]',
      'Smooths the pulse/glide square wave into a gentle',
      'rolling average → FC follows average, SC absorbs pulses'],PU),
    ('②',8.8, 13.3,'SOC ERROR\n(feedback signal)',
     ['e[k] = SOC_ref − SOC[k]  =  0.60 − SOC[k]',
      'Positive e → SC low → need more FC output',
      'Negative e → SC high → need less FC output'],GR),
    ('③',8.8, 11.3,'PI COMPUTATION\n(correction)',
     ['Proportional:  K_p · e[k]           (instant response)',
      'Integral:      K_i · Σe·Δt          (removes steady-state drift)',
      'Anti-windup:   |I| clamped at 75 W to prevent integrator blow-up'],GR),
    ('④',8.8,  9.3,'PER-LAP OFFSET\n(slow correction)',
     ['At each lap boundary: δ += 0.3 · e_SOC · E_SC / 186',
      'Slowly shifts the FC setpoint if SOC drifts lap-over-lap',
      'Clipped ±200 W   →   prevents over-correction'],YL),
    ('⑤',8.8,  7.3,'SETPOINT ASSEMBLY\n+ HARDWARE LIMITS',
     ['P_cmd = LPF(P_d) + K_p·e + K_i·∫e + δ',
      'Ramp-limit: |ΔP_FC| ≤ 20 W per step  (FC membrane protection)',
      'Saturate: P_FC ∈ [100 W, 1013 W]  (FC safe operating range)'],RD),
]

for num,cx,cy,title,desc,pal in steps:
    ax.text(0.5,cy+0.55,num,ha='center',va='center',fontsize=18,
            fontweight='bold',color=pal[1])
    box(cx,cy,17.2,1.5,[title]+desc,pal,fs=8.8,lw=2.0)
    if cy > 7.3:
        arr(cx,cy-0.75,cx,cy-1.25,col=pal[1],lw=1.8)

ax.axhline(6.2,color='#CBD5E1',lw=1.0,xmin=0.02,xmax=0.98)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION C — SIMULINK BLOCK MAP
# ══════════════════════════════════════════════════════════════════════════════
ax.text(0.4,6.0,'C.  SIMULINK IMPLEMENTATION  (direct 1:1 block mapping)',
        ha='left',va='center',fontsize=11,fontweight='bold',color='#1D4ED8')

sim_blocks=[
  ('Strategy G element','Simulink block','Block parameters','Difficulty'),
  ('LPF  (τ=15 s, α=0.987)','Discrete Transfer Fcn',
   'Num:[1−α]  Den:[1, −α]  Ts=0.2 s','Trivial'),
  ('SOC error  e=SOC_ref−SOC','Sum  (+/−)','2 inputs, signs: +−','Trivial'),
  ('Proportional  K_p·e','Gain','K_p = 372','Trivial'),
  ('Integrator  K_i·∫e','Discrete Integrator',
   'K_i=2, Ts=0.2, anti-windup clamp ±75 W','Easy'),
  ('Per-lap offset δ','Triggered Subsystem  +  Unit Delay  +  Gain  +  Saturation',
   'Trigger: rising edge lap counter','Moderate'),
  ('Summation  P_cmd','Sum','4 inputs: LPF + Kp·e + Ki·∫e + δ','Trivial'),
  ('Ramp-rate limiter','Rate Limiter','Rising/Falling: ±100 W/s','Trivial'),
  ('Saturation  [100,1013]W','Saturation','Lower=100, Upper=1013','Trivial'),
  ('Motor-off gate  Pd<5W→0','Switch  or  Dead Zone','Threshold=5 W','Trivial'),
  ('fc_min floor (100W glide)','Switch  (via coast flag)','Coast flag from drive cycle','Easy'),
]

tw=[4.5,4.5,6.0,2.0]; starts=[0.3,4.9,9.5,15.6]
row_h=0.52; head_y=5.75; cols=['#1E40AF','#1E40AF','#374151','#374151']

# Header
for j,(tw_j,sx_j,col_j,hdr) in enumerate(zip(tw,starts,cols,sim_blocks[0])):
    bg='#1E3A5F'
    r=FancyBboxPatch((sx_j,head_y-0.28),tw_j-0.05,0.44,
                      boxstyle='round,pad=0.05',fc=bg,ec=bg,lw=0,zorder=4)
    ax.add_patch(r)
    ax.text(sx_j+tw_j/2-0.025,head_y,hdr,ha='center',va='center',
            fontsize=8.5,fontweight='bold',color='white',zorder=5)

diff_cols={'Trivial':'#15803D','Easy':'#B45309','Moderate':'#BE123C'}
for i,row in enumerate(sim_blocks[1:]):
    ry=head_y-(i+1)*row_h
    bg='#F8FAFC' if i%2==0 else 'white'
    for j,(tw_j,sx_j,cell) in enumerate(zip(tw,starts,row)):
        r=FancyBboxPatch((sx_j,ry-row_h/2+0.06),tw_j-0.05,row_h-0.1,
                          boxstyle='round,pad=0.04',
                          fc=diff_cols.get(cell,bg) if j==3 else bg,
                          ec='#E2E8F0',lw=0.8,zorder=4)
        ax.add_patch(r)
        tc=('white' if (j==3 and cell in diff_cols) else
            ('#C2410C' if j==0 else '#1D4ED8' if j==1 else '#374151'))
        ax.text(sx_j+tw_j/2-0.025,ry,cell,ha='center',va='center',
                fontsize=7.6,color=tc,fontweight='bold' if j<=1 else 'normal',zorder=5)

# Difficulty legend
ax.text(0.3,0.55,'Difficulty:',ha='left',va='center',fontsize=8.5,
        color='#374151',fontweight='bold')
for txt,col,x0 in [('Trivial (<5 min)','#15803D',2.2),
                    ('Easy (10 min)','#B45309',5.8),
                    ('Moderate (30 min)','#BE123C',9.0)]:
    r=FancyBboxPatch((x0,0.28),3.2,0.44,boxstyle='round,pad=0.06',
                      fc=col,ec=col,lw=0,zorder=4)
    ax.add_patch(r)
    ax.text(x0+1.6,0.5,txt,ha='center',va='center',
            fontsize=8.2,color='white',fontweight='bold',zorder=5)

ax.text(13.0,0.5,'⚡ Total Simulink build time: ~2 hours for a complete, verified model',
        ha='left',va='center',fontsize=9,color='#C2410C',fontweight='bold')

plt.tight_layout(pad=0)
fig.savefig(OUT,dpi=155,bbox_inches='tight',facecolor='#F8FAFC')
plt.close(fig)
print(f'Saved → {OUT}')
