"""
Strategy G head-to-head: Canonical GPS vs Elevation-aware P&G profile.

Runs Feedforward + SOC-PI (Strategy G) on both velocity profiles, auto-bisects
K_p for |ΔSOC| < 0.01 on each, and produces a 5-panel comparison figure.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from ems_core import (
    build_demand_profile, fc_current, fc_h2_rate,
    sc_soc_update, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX, SC_E_J,
    FC_P_MAX, FC_RAMP, K_H2, N_LAPS, DT, RESULTS_DIR, MATLAB_DIR,
)

H2_DENSITY  = 89.88   # g/m³ at STP
TOTAL_KM    = 14.5
K_LAP       = 0.3     # lap supervisor gain
TAU         = 15.0    # LPF time constant [s]
K_I_FIXED   = 2.0     # W/(SOC·s)
TOLERANCE   = 0.01    # |ΔSOC| target


# ── Strategy factory ──────────────────────────────────────────────────────────
def make_strategy(K_p, K_i=2.0, tau=15.0, T_LAP=None):
    """
    Feedforward + SOC-PI (Strategy G).
    T_LAP: lap duration [s] used by lap supervisor (None → use canonical T_LAP).
    """
    alpha = np.exp(-DT / tau)

    # T_LAP is captured per-call so each bisection iteration gets a fresh closure.
    # We resolve T_LAP_canonical lazily inside the factory so multiple calls to
    # make_strategy() don't all re-run build_demand_profile().
    _T = T_LAP  # may be None; resolved below after canonical profile is built

    state = {
        'P_filt':       None,
        'integrator':   0.0,
        'P_lap_offset': 0.0,
        'prev_lap_idx': -1,
        'T_lap_eff':    _T,   # filled in on first lap-boundary call if None
    }

    def strategy_fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx):
        # Initialise filter on first call
        if state['P_filt'] is None:
            state['P_filt'] = P_dem

        # Resolve T_lap_eff once if not provided (use canonical duration)
        if state['T_lap_eff'] is None:
            state['T_lap_eff'] = _T_LAP_CANONICAL

        # Lap boundary: update lap supervisor
        if lap_idx != state['prev_lap_idx'] and lap_idx > 0:
            E_deficit = (SC_SOC_0 - SOC) * SC_E_J
            state['P_lap_offset'] += K_LAP * E_deficit / state['T_lap_eff']
            state['P_lap_offset'] = float(np.clip(state['P_lap_offset'], -200.0, 200.0))
        state['prev_lap_idx'] = lap_idx

        # 1st-order IIR low-pass filter (feedforward)
        state['P_filt'] = alpha * state['P_filt'] + (1.0 - alpha) * P_dem
        P_filt = state['P_filt']

        # SOC-PI controller
        soc_err = SC_SOC_0 - SOC
        state['integrator'] += soc_err * DT
        if K_i > 1e-9:
            state['integrator'] = float(np.clip(
                state['integrator'], -150.0 / K_i, +150.0 / K_i))
        P_pi = K_p * soc_err + K_i * state['integrator']

        # Combined command
        P_cmd = P_filt + P_pi + state['P_lap_offset']
        if P_dem < 25.0 and SOC >= SC_SOC_0 - 0.05:
            P_cmd = 0.0   # true glide: FC off

        P_fc = float(np.clip(P_cmd, 0.0, FC_P_MAX))
        return (P_fc, P_filt)

    return strategy_fn


# ── Simulator: canonical GPS profile ─────────────────────────────────────────
def simulate_canonical(strategy_fn, SOC_0=SC_SOC_0):
    t_lap, P_lap = build_demand_profile()
    N_pts  = len(t_lap)
    T_lap  = float(t_lap[-1])
    t_all  = np.concatenate([t_lap + i * T_lap for i in range(N_LAPS)])
    P_dem  = np.tile(P_lap, N_LAPS)
    n      = N_LAPS * N_pts

    P_fc_ar   = np.empty(n)
    P_sc_ar   = np.empty(n)
    SOC_ar    = np.empty(n + 1)
    I_fc_ar   = np.empty(n)
    m_H2_ar   = np.empty(n)
    P_filt_ar = np.empty(n)
    lap_SOC   = np.zeros(N_LAPS + 1)

    SOC_ar[0] = SOC_0
    lap_SOC[0] = SOC_0
    P_fc_prev = 0.0

    for k in range(n):
        lap_idx  = k // N_pts
        t_in_lap = float(t_all[k] - lap_idx * T_lap)
        soc_k = SOC_ar[k]
        Pd    = float(P_dem[k])

        res = strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx)
        P_fc_cmd, P_filt_val = res if isinstance(res, tuple) else (res, res)
        P_fc_cmd = float(P_fc_cmd)

        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k    = Pd - P_fc_cmd
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = min(FC_P_MAX, Pd)
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        P_filt_ar[k]  = float(P_filt_val)
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(max(P_fc_cmd, 1e-6)) if P_fc_cmd > 0 else 0.0)
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev     = P_fc_cmd

        if (k + 1) % N_pts == 0:
            lap_SOC[(k + 1) // N_pts] = SOC_ar[k + 1]

    return {
        't':          t_all,
        'P_dem':      P_dem,
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'P_filt':     P_filt_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'SOC_final':  float(SOC_ar[n]),
        'lap_SOC':    lap_SOC,
        'N_pts_lap':  N_pts,
        'T_lap':      T_lap,
    }


# ── Simulator: custom (elevation-aware) profile ───────────────────────────────
def simulate_custom(strategy_fn, t_all, P_dem, lap_idx_arr, SOC_0=SC_SOC_0):
    """Strategy G on a pre-built demand array with variable lap lengths."""
    n = len(P_dem)

    P_fc_ar   = np.empty(n)
    P_sc_ar   = np.empty(n)
    SOC_ar    = np.empty(n + 1)
    I_fc_ar   = np.empty(n)
    m_H2_ar   = np.empty(n)
    P_filt_ar = np.empty(n)
    lap_SOC   = np.zeros(N_LAPS + 1)

    SOC_ar[0]  = SOC_0
    lap_SOC[0] = SOC_0
    P_fc_prev  = 0.0

    # Time offset at which each lap starts
    lap_start_t = {}
    for k in range(n):
        li = int(lap_idx_arr[k])
        if li not in lap_start_t:
            lap_start_t[li] = float(t_all[k])

    for k in range(n):
        lap_idx  = int(lap_idx_arr[k])
        t_in_lap = float(t_all[k]) - lap_start_t[lap_idx]
        soc_k = SOC_ar[k]
        Pd    = float(P_dem[k])

        res = strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx)
        P_fc_cmd, P_filt_val = res if isinstance(res, tuple) else (res, res)
        P_fc_cmd = float(P_fc_cmd)

        if P_fc_cmd > P_fc_prev:
            P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        P_fc_cmd = float(np.clip(P_fc_cmd, 0.0, FC_P_MAX))

        P_sc_k    = Pd - P_fc_cmd
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd  = min(FC_P_MAX, Pd)
            P_sc_k    = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]    = P_fc_cmd
        P_sc_ar[k]    = P_sc_k
        P_filt_ar[k]  = float(P_filt_val)
        SOC_ar[k + 1] = trial_soc
        I_fc_ar[k]    = float(fc_current(max(P_fc_cmd, 1e-6)) if P_fc_cmd > 0 else 0.0)
        m_H2_ar[k]    = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev     = P_fc_cmd

        # Record SOC at each lap boundary
        nxt_lap = int(lap_idx_arr[k + 1]) if k + 1 < n else lap_idx + 1
        if nxt_lap != lap_idx:
            lap_SOC[lap_idx + 1] = SOC_ar[k + 1]

    lap_SOC[N_LAPS] = SOC_ar[n]   # ensure final entry

    return {
        't':          t_all,
        'P_dem':      P_dem,
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'P_filt':     P_filt_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'SOC_final':  float(SOC_ar[n]),
        'lap_SOC':    lap_SOC,
    }


# ── Bisect K_p for charge sustenance ─────────────────────────────────────────
def bisect_kp(sim_fn, label, T_LAP_override=None):
    print(f"\n── Bisecting K_p for '{label}' (|ΔSOC| < {TOLERANCE}) ──────────")
    lo, hi = 0.0, 3000.0
    K_p    = 800.0
    best   = None

    for i in range(25):
        strat  = make_strategy(K_p=K_p, K_i=K_I_FIXED, tau=TAU, T_LAP=T_LAP_override)
        result = sim_fn(strat)
        d      = result['delta_SOC']
        print(f"  iter {i+1:2d}  K_p={K_p:7.1f}  ΔSOC={d:+.4f}  H2={result['m_H2_total']:.3f} g")
        best = result
        if abs(d) <= TOLERANCE:
            break
        if d > TOLERANCE:
            lo = K_p   # SOC drifting high → raise K_p (reduces net FC output)
        else:
            hi = K_p   # SOC drifting low  → lower K_p
        K_p = (lo + hi) / 2.0

    return K_p, best


# ── Load elevation-aware profile ──────────────────────────────────────────────
print("Loading elevation-aware profile …")
_csv = os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv')
_df  = pd.read_csv(_csv)
t_elev   = _df['time_s'].values
P_elev   = _df['P_elec_W'].values
lap_elev = (_df['lap_num'].values - 1).astype(int)   # 0-indexed

# Mean lap duration (for lap supervisor T_LAP)
_dur = []
for _li in range(N_LAPS):
    _mask = lap_elev == _li
    if _mask.sum() > 0:
        _dur.append(float(t_elev[_mask][-1] - t_elev[_mask][0]))
T_LAP_ELEV = float(np.mean(_dur))

print(f"  {len(_df)} samples  |  mean T_lap = {T_LAP_ELEV:.1f} s  "
      f"|  mean P_dem = {np.mean(P_elev):.1f} W")

# ── Resolve canonical T_LAP (used by strategy closures when T_LAP=None) ──────
_t_ref, _ = build_demand_profile()
_T_LAP_CANONICAL = float(_t_ref[-1])   # module-level for make_strategy closures

# Expose to the strategy closures via module-level name
import builtins
# Use a simple global so make_strategy() can see it
globals()['_T_LAP_CANONICAL'] = _T_LAP_CANONICAL

print(f"  Canonical T_lap = {_T_LAP_CANONICAL:.1f} s")

# ── Run: Canonical ────────────────────────────────────────────────────────────
kp_can, res_can = bisect_kp(
    lambda strat: simulate_canonical(strat),
    'Canonical GPS',
    T_LAP_override=None,
)

# ── Run: Elevation-aware ──────────────────────────────────────────────────────
kp_elev, res_elev = bisect_kp(
    lambda strat: simulate_custom(strat, t_elev, P_elev, lap_elev),
    'Elev-aware P&G',
    T_LAP_override=T_LAP_ELEV,
)

# ── Derived metrics ───────────────────────────────────────────────────────────
def metrics(res):
    m  = res['m_H2_total']
    ds = res['delta_SOC']
    return {
        'H2_g':      m,
        'km_m3':     TOTAL_KM / (m / H2_DENSITY),
        'delta_SOC': ds,
        'mean_Pdem': float(np.mean(res['P_dem'])),
        'mean_Pfc':  float(np.mean(res['P_fc'])),
        'mean_Ifc':  float(np.mean(res['I_fc'])),
        'duration':  float(res['t'][-1]) / 60.0,
        'cum_H2':    np.cumsum(K_H2 * res['I_fc'] * DT),
        't_min':     res['t'] / 60.0,
    }

mc = metrics(res_can)
me = metrics(res_elev)

print("\n── Strategy G results ────────────────────────────────────────────────")
print(f"{'Metric':<28} {'Canonical GPS':>15} {'Elev-aware P&G':>16}")
print(f"{'H2 consumed [g]':<28} {mc['H2_g']:>15.3f} {me['H2_g']:>16.3f}")
print(f"{'km/m³':<28} {mc['km_m3']:>15.1f} {me['km_m3']:>16.1f}")
print(f"{'ΔSOC':<28} {mc['delta_SOC']:>+15.4f} {me['delta_SOC']:>+16.4f}")
print(f"{'Mean P_dem [W]':<28} {mc['mean_Pdem']:>15.1f} {me['mean_Pdem']:>16.1f}")
print(f"{'Mean P_fc [W]':<28} {mc['mean_Pfc']:>15.1f} {me['mean_Pfc']:>16.1f}")
print(f"{'Race duration [min]':<28} {mc['duration']:>15.1f} {me['duration']:>16.1f}")
print(f"{'Tuned K_p [W/SOC]':<28} {kp_can:>15.1f} {kp_elev:>16.1f}")

# ── Lap-boundary times ────────────────────────────────────────────────────────
lap_t_can  = [res_can['T_lap'] * (i + 1) / 60.0 for i in range(N_LAPS - 1)]

# For elev profile: find time of first sample of each lap (laps 1..10 boundaries)
lap_t_elev = []
for li in range(1, N_LAPS):
    idx = np.argmax(lap_elev == li)
    lap_t_elev.append(float(t_elev[idx]) / 60.0)

# ── Figure ────────────────────────────────────────────────────────────────────
C_CAN  = '#2166ac'    # blue  — canonical
C_ELEV = '#d6604d'    # red   — elevation-aware

fig = plt.figure(figsize=(15, 18), dpi=150)
gs  = GridSpec(6, 1, figure=fig, hspace=0.42, top=0.94, bottom=0.04)
axes = [fig.add_subplot(gs[i]) for i in range(5)]
ax_tab = fig.add_subplot(gs[5])

fig.suptitle(
    "Strategy G — Feedforward + SOC-PI\n"
    "Canonical GPS profile  vs  Elevation-aware Pulse-and-Glide profile",
    fontsize=13, fontweight='bold',
)

def _lap_vlines(ax, lap_times, color='grey'):
    for t in lap_times:
        ax.axvline(x=t, color=color, lw=0.4, alpha=0.4, ls='--')

# ── Panel 1: Demand (P_dem) ───────────────────────────────────────────────────
ax = axes[0]
ax.plot(mc['t_min'], res_can['P_dem'],  color=C_CAN,  lw=0.5, alpha=0.8,
        label=f"Canonical  (mean {mc['mean_Pdem']:.0f} W)")
ax.plot(me['t_min'], res_elev['P_dem'], color=C_ELEV, lw=0.5, alpha=0.8,
        label=f"Elev-aware (mean {me['mean_Pdem']:.0f} W)")
_lap_vlines(ax, lap_t_can, C_CAN)
_lap_vlines(ax, lap_t_elev, C_ELEV)
ax.set_ylabel('P_dem [W]')
ax.set_title('Motor Power Demand  (input to Strategy G)')
ax.legend(fontsize=8, loc='upper right')
ax.grid(True, lw=0.25)

# ── Panel 2: FC power (P_fc) ──────────────────────────────────────────────────
ax = axes[1]
ax.plot(mc['t_min'], res_can['P_fc'],   color=C_CAN,  lw=0.6, alpha=0.85,
        label=f"Canonical  (mean {mc['mean_Pfc']:.0f} W)")
ax.plot(me['t_min'], res_elev['P_fc'],  color=C_ELEV, lw=0.6, alpha=0.85,
        label=f"Elev-aware (mean {me['mean_Pfc']:.0f} W)")
_lap_vlines(ax, lap_t_can, C_CAN)
_lap_vlines(ax, lap_t_elev, C_ELEV)
ax.set_ylabel('P_fc [W]')
ax.set_title('FC Net Power Output  (Strategy G decision)')
ax.legend(fontsize=8, loc='upper right')
ax.grid(True, lw=0.25)

# ── Panel 3: SC State of Charge ───────────────────────────────────────────────
ax = axes[2]
ax.plot(mc['t_min'], res_can['SOC'],    color=C_CAN,  lw=0.8, label='Canonical')
ax.plot(me['t_min'], res_elev['SOC'],   color=C_ELEV, lw=0.8, label='Elev-aware')
ax.axhline(SC_SOC_0,   color='black',  lw=0.8, ls='--', label=f'SOC_ref={SC_SOC_0:.2f}')
ax.axhline(SC_SOC_MIN, color='red',    lw=0.6, ls=':',  label=f'SOC_min={SC_SOC_MIN:.2f}')
ax.axhline(SC_SOC_MAX, color='purple', lw=0.6, ls=':',  label=f'SOC_max={SC_SOC_MAX:.2f}')
_lap_vlines(ax, lap_t_can, C_CAN)
_lap_vlines(ax, lap_t_elev, C_ELEV)
ax.set_ylabel('SC SOC [—]')
ax.set_ylim(0.0, 1.0)
ax.set_title('Supercapacitor State of Charge  (dashed lines = lap boundaries)')
ax.legend(fontsize=8, loc='upper right', ncol=2)
ax.grid(True, lw=0.25)

# ── Panel 4: FC stack current ─────────────────────────────────────────────────
ax = axes[3]
ax.plot(mc['t_min'], res_can['I_fc'],   color=C_CAN,  lw=0.6, alpha=0.85,
        label=f"Canonical  (mean {mc['mean_Ifc']:.2f} A)")
ax.plot(me['t_min'], res_elev['I_fc'],  color=C_ELEV, lw=0.6, alpha=0.85,
        label=f"Elev-aware (mean {me['mean_Ifc']:.2f} A)")
_lap_vlines(ax, lap_t_can, C_CAN)
_lap_vlines(ax, lap_t_elev, C_ELEV)
ax.set_ylabel('I_fc [A]')
ax.set_title('FC Stack Current')
ax.legend(fontsize=8, loc='upper right')
ax.grid(True, lw=0.25)

# ── Panel 5: Cumulative H₂ ────────────────────────────────────────────────────
ax = axes[4]
ax.plot(mc['t_min'], mc['cum_H2'],  color=C_CAN,  lw=1.0,
        label=f"Canonical  — total {mc['H2_g']:.3f} g  ({mc['km_m3']:.1f} km/m³)")
ax.plot(me['t_min'], me['cum_H2'],  color=C_ELEV, lw=1.0,
        label=f"Elev-aware — total {me['H2_g']:.3f} g  ({me['km_m3']:.1f} km/m³)")
_lap_vlines(ax, lap_t_can, C_CAN)
_lap_vlines(ax, lap_t_elev, C_ELEV)
ax.set_ylabel('Cumulative H₂ [g]')
ax.set_xlabel('Time [min]')
ax.set_title('Cumulative Hydrogen Consumed')
ax.legend(fontsize=8, loc='upper left')
ax.grid(True, lw=0.25)

# ── Panel 6: Summary table ────────────────────────────────────────────────────
ax_tab.axis('off')
rows = [
    ['Metric',                    'Canonical GPS',         'Elev-aware P&G',        'Delta (Elev − Can)'],
    ['H₂ consumed [g]',           f"{mc['H2_g']:.3f}",    f"{me['H2_g']:.3f}",     f"{me['H2_g']-mc['H2_g']:+.3f}"],
    ['km / m³',                   f"{mc['km_m3']:.1f}",   f"{me['km_m3']:.1f}",    f"{me['km_m3']-mc['km_m3']:+.1f}"],
    ['ΔSOC (end − start)',        f"{mc['delta_SOC']:+.4f}",f"{me['delta_SOC']:+.4f}", '—'],
    ['Race duration [min]',       f"{mc['duration']:.1f}", f"{me['duration']:.1f}", f"{me['duration']-mc['duration']:+.1f}"],
    ['Mean P_dem [W]',            f"{mc['mean_Pdem']:.1f}",f"{me['mean_Pdem']:.1f}", f"{me['mean_Pdem']-mc['mean_Pdem']:+.1f}"],
    ['Mean P_fc [W]',             f"{mc['mean_Pfc']:.1f}", f"{me['mean_Pfc']:.1f}", f"{me['mean_Pfc']-mc['mean_Pfc']:+.1f}"],
    ['Mean I_fc [A]',             f"{mc['mean_Ifc']:.2f}", f"{me['mean_Ifc']:.2f}", f"{me['mean_Ifc']-mc['mean_Ifc']:+.2f}"],
    ['Tuned K_p [W/SOC]',        f"{kp_can:.1f}",         f"{kp_elev:.1f}",        '—'],
    ['H₂ saving vs canonical',   '—',                     f"{(1-me['H2_g']/mc['H2_g'])*100:.1f} %", '—'],
]

col_w   = [0.31, 0.23, 0.23, 0.23]
col_x   = [0.0, 0.31, 0.54, 0.77]
row_h   = 1.0 / len(rows)

header_bg = '#d0dce8'
row_bg    = ['#f7f7f7', '#ffffff']

for r_idx, row in enumerate(rows):
    y = 1.0 - (r_idx + 1) * row_h
    for c_idx, (cell, cx, cw) in enumerate(zip(row, col_x, col_w)):
        is_header = (r_idx == 0)
        fc_color  = header_bg if is_header else row_bg[r_idx % 2]
        weight    = 'bold' if is_header else 'normal'
        # colour positive delta green, negative red for H2 (lower is better)
        if c_idx == 3 and r_idx > 0 and cell not in ('—',):
            try:
                val = float(cell.strip('%'))
                text_color = '#006400' if val < 0 else '#8b0000'
            except ValueError:
                text_color = 'black'
        else:
            text_color = 'black'

        ax_tab.add_patch(plt.Rectangle(
            (cx, y), cw, row_h,
            transform=ax_tab.transAxes, clip_on=False,
            facecolor=fc_color, edgecolor='#aaaaaa', lw=0.5,
        ))
        ax_tab.text(
            cx + cw / 2, y + row_h / 2,
            cell,
            transform=ax_tab.transAxes, clip_on=False,
            ha='center', va='center',
            fontsize=8.5, fontweight=weight, color=text_color,
        )

out_path = os.path.join(RESULTS_DIR, 'strategy_g_elev_comparison.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close()
print(f"\nFigure saved → {out_path}")
