"""
build_velocity_profile.py — Elevation-aware velocity-vs-time drive cycle for
Hydraix I SEM 2026 at Silesia Ring.

Generates a physically-realistic pulse-and-glide profile that accounts for
route elevation changes, targeting 11 laps × 1319.6 m = 14.52 km in 34-35 min.
"""

import matplotlib
matplotlib.use('Agg')

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

# ── Paths ─────────────────────────────────────────────────────────────────────
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
ROUTE_CSV   = os.path.join(_THIS_DIR, '..', 'datasheets', 'sem_2025_eu.csv')
MATLAB_DIR  = os.path.join(_THIS_DIR, '..', 'matlab')
RESULTS_DIR = os.path.join(_THIS_DIR, '..', 'results', 'ems')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(MATLAB_DIR, exist_ok=True)

OUTPUT_CSV = os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv')
OUTPUT_PNG = os.path.join(RESULTS_DIR, 'velocity_profile_elev_aware.png')

# ── Vehicle constants ──────────────────────────────────────────────────────────
MASS    = 175.0     # kg
G       = 9.81      # m/s²
CD      = 0.15
AF      = 0.8       # m²
CRR     = 0.006
RHO     = 1.225     # kg/m³
ETA_DT  = 0.95
DT      = 0.2       # s  (5 Hz)

FC_P_MAX_W = 1013.0   # W  FC max power
MOT_MAX_W  = 1000.0   # W  motor rated electrical input (BAFANG RM G060.1000)

# ── Race parameters ────────────────────────────────────────────────────────────
N_LAPS  = 11

# ── Pulse-and-glide tuning parameters ─────────────────────────────────────────
V_HIGH  = 8.33    # m/s  (30 km/h) — glide trigger (flat), tuned to 8.0 below
V_LOW   = 5.83    # m/s  (21 km/h) — pulse trigger (flat)
V_MAX   = 10.0    # m/s  (36 km/h) — absolute speed cap
V_MIN   = 5.0     # m/s  (18 km/h) — minimum acceptable speed
P_PULSE = 1000.0  # W   motor electrical power during flat pulse (rated input)
P_BOOST = 1000.0  # W   motor electrical power on uphill (rated input)
A_STOP  = 0.5     # m/s²  gentle coast-to-stop (rolling + aero + light retardation)
GRADE_THRESH = 0.006  # 0.6%  — only real hills trigger special modes
P_RAMP_MAX   = 200.0  # W/step — motor power ramp limit per DT

# ── Tuned operating point ──────────────────────────────────────────────────────
# With A_STOP=0.5 m/s² the lap-end stop takes ~12-15 s (vs 3 s before),
# adding ~1.5-2 min total.  V_HIGH raised to 8.5 m/s to compensate.
V_HIGH = 8.2      # effective glide-trigger (29.5 km/h)

N_STOP_STEPS = round(4.0 / DT)   # 4-second stop at lap line = 20 steps

# ── Load and process route CSV ─────────────────────────────────────────────────

def _load_route():
    """Read route CSV, smooth elevation with Savitzky-Golay, build interpolators."""
    df = pd.read_csv(ROUTE_CSV)
    # Rename columns robustly
    df.columns = [c.strip() for c in df.columns]
    dist_col = [c for c in df.columns if 'Distance' in c][0]
    elev_col = [c for c in df.columns if 'Elevation' in c][0]

    s_raw  = df[dist_col].values
    e_raw  = df[elev_col].values

    D_LAP = float(s_raw[-1])   # 1319.627 m

    # Smooth elevation (window 51, poly 3)
    e_smooth = savgol_filter(e_raw, window_length=51, polyorder=3)

    # Compute gradient dElev/ds on the smoothed elevation
    grad_raw = np.gradient(e_smooth, s_raw)

    # Smooth gradient again (window 51, poly 3)
    grad_smooth = savgol_filter(grad_raw, window_length=51, polyorder=3)

    # Build interpolators (extrapolate=True so wrap-around queries at s~D_LAP work)
    elev_fn  = interp1d(s_raw, e_smooth,    kind='linear', fill_value='extrapolate')
    grade_fn = interp1d(s_raw, grad_smooth, kind='linear', fill_value='extrapolate')

    return D_LAP, s_raw, e_smooth, grad_smooth, elev_fn, grade_fn


D_LAP, _s_raw, _e_smooth, _grad_smooth, _elev_fn, _grade_fn = _load_route()


# ── Physics helpers ────────────────────────────────────────────────────────────

def _aero_drag(v):
    """Aerodynamic drag force [N]."""
    return 0.5 * CD * AF * RHO * v**2


def _rolling_resist(v):
    """Rolling resistance force [N] (velocity-independent for v>0)."""
    return CRR * MASS * G


def _gravity_force(grade):
    """Gravitational force component along slope [N]."""
    return MASS * G * grade   # positive = uphill resistance


def _net_force(v, P_elec, grade):
    """
    Net force on vehicle [N] given electrical motor power and grade.

    F_motor = P_elec * ETA_DT / max(v, 0.3) clamped to MASS * 2.0 m/s²
    """
    if P_elec > 0:
        F_motor = P_elec * ETA_DT / max(v, 0.3)
        F_motor = min(F_motor, MASS * 2.0)
    else:
        F_motor = 0.0

    F_drag    = _aero_drag(v)
    F_roll    = _rolling_resist(v)
    F_gravity = _gravity_force(grade)

    F_net = F_motor - F_drag - F_roll - F_gravity
    return F_net


# ── One-lap builder ────────────────────────────────────────────────────────────

def _build_one_lap():
    """
    Forward-Euler simulation of one lap starting from v=0.

    Returns
    -------
    t_lst    : list of floats  — time [s]
    v_lst    : list of floats  — velocity [m/s]
    P_lst    : list of floats  — motor electrical power [W]
    s_lst    : list of floats  — position in lap [m]
    elev_lst : list of floats  — elevation [m]
    grade_lst: list of floats  — grade at position
    """
    t_lst    = []
    v_lst    = []
    P_lst    = []
    s_lst    = []
    elev_lst = []
    grade_lst= []

    t = 0.0
    v = 0.0
    s = 0.0
    mode = 'PULSE'   # start in pulse to accelerate from stop
    first_step = True  # skip recording the initial v=0 state (overlaps prev lap's stop)
    prev_P_elec = 0.0  # for power ramping

    MAX_LAP_TIME = 600.0  # safety cap

    while True:
        # Wrap position to [0, D_LAP]
        s_wrap = s % D_LAP

        grade = float(_grade_fn(s_wrap))
        elev  = float(_elev_fn(s_wrap))

        # ── Brake check: do we need to start braking to stop at lap end? ──────
        remaining = D_LAP - s_wrap
        brake_dist = v**2 / (2.0 * A_STOP) + v * DT

        if v > 0 and remaining <= brake_dist and remaining > 0 and mode != 'BRAKE':
            mode = 'BRAKE'

        # ── Determine P_elec for THIS step based on mode and grade ────────────
        if mode == 'BRAKE':
            P_elec = 0.0
        elif grade < -GRADE_THRESH:
            # Downhill: motor OFF, let gravity assist; cap at V_MAX
            if v < V_LOW and grade > -0.005:
                # Flattening after downhill, below V_LOW — resume pulse
                P_elec = P_PULSE
                mode = 'PULSE'
            else:
                P_elec = 0.0
        elif grade > +GRADE_THRESH:
            # Uphill: always boost until V_HIGH, then glide if > V_HIGH
            # (entering uphill mid-glide: force pulse to maintain speed on hill)
            if v >= V_HIGH:
                mode = 'GLIDE'
                P_elec = 0.0
            else:
                mode = 'PULSE'
                P_elec = P_BOOST
        else:
            # Flat: standard pulse-and-glide
            if v <= V_LOW:
                mode = 'PULSE'
                P_elec = P_PULSE
            elif v >= V_HIGH:
                mode = 'GLIDE'
                P_elec = 0.0
            else:
                # Maintain current mode
                if mode == 'PULSE':
                    P_elec = P_PULSE
                elif mode == 'GLIDE':
                    P_elec = 0.0
                else:
                    P_elec = P_PULSE

        # Ramp power: limit step change to P_RAMP_MAX W per timestep
        P_elec = float(np.clip(P_elec,
                                prev_P_elec - P_RAMP_MAX,
                                prev_P_elec + P_RAMP_MAX))
        prev_P_elec = P_elec

        # Record state for this timestep BEFORE updating velocity
        # (skip the very first step: initial v=0 would extend the prev lap's stop)
        if not first_step:
            t_lst.append(t)
            v_lst.append(v)
            P_lst.append(P_elec)
            s_lst.append(s_wrap)
            elev_lst.append(elev)
            grade_lst.append(grade)
        first_step = False

        # ── Advance state ─────────────────────────────────────────────────────
        if mode == 'BRAKE':
            # Gentle coast-to-stop (forward-Euler: s advances with current v)
            P_elec = 0.0
            v_new = max(0.0, v - A_STOP * DT)
            s_new = s + v * DT
        else:
            # Physics-based forward Euler
            F_net = _net_force(v, P_elec, grade)
            a = F_net / MASS
            v_new = max(0.0, v + a * DT)

            # Speed cap
            if grade < -GRADE_THRESH:
                v_upper = min(V_MAX, V_HIGH + 3.0)
                v_new = min(v_new, v_upper)
            else:
                v_new = min(v_new, V_MAX)

            s_new = s + v * DT

        t += DT
        v  = v_new
        s  = s_new

        # ── Check if we've reached end of lap ─────────────────────────────────
        # Exit when car has passed the lap line, or stops at the lap line
        if s >= D_LAP or (mode == 'BRAKE' and v == 0.0) or t > MAX_LAP_TIME:
            break

    # ── Append the 4-second stop (N_STOP_STEPS steps of v=0, P=0) ─────────────
    s_end    = float(s_lst[-1])   # keep position at lap line
    elev_end = float(elev_lst[-1])
    grade_end= float(grade_lst[-1])

    for _ in range(N_STOP_STEPS):
        t_lst.append(t)
        v_lst.append(0.0)
        P_lst.append(0.0)
        s_lst.append(s_end)
        elev_lst.append(elev_end)
        grade_lst.append(grade_end)
        t += DT

    return t_lst, v_lst, P_lst, s_lst, elev_lst, grade_lst


# ── Full 11-lap profile ────────────────────────────────────────────────────────

def build_full_profile():
    """
    Build the full 11-lap race profile by concatenating individual laps.

    Returns
    -------
    t_arr       : ndarray  — continuous time [s]
    v_arr       : ndarray  — velocity [m/s]
    P_arr       : ndarray  — motor electrical power [W]
    s_arr       : ndarray  — position within current lap [m]
    lap_num_arr : ndarray  — lap number (1-indexed)
    elev_arr    : ndarray  — elevation [m]
    grade_arr   : ndarray  — grade at position
    """
    all_t     = []
    all_v     = []
    all_P     = []
    all_s     = []
    all_lap   = []
    all_elev  = []
    all_grade = []

    t_offset = 0.0

    for lap in range(1, N_LAPS + 1):
        t_l, v_l, P_l, s_l, e_l, g_l = _build_one_lap()

        t_arr_lap = np.array(t_l) + t_offset

        all_t.append(t_arr_lap)
        all_v.append(np.array(v_l))
        all_P.append(np.array(P_l))
        all_s.append(np.array(s_l))
        all_lap.append(np.full(len(t_l), lap, dtype=int))
        all_elev.append(np.array(e_l))
        all_grade.append(np.array(g_l))

        # Offset: end of this lap + one DT gap before next lap starts
        t_offset = float(t_arr_lap[-1]) + DT

    t_arr       = np.concatenate(all_t)
    v_arr       = np.concatenate(all_v)
    P_arr       = np.concatenate(all_P)
    s_arr       = np.concatenate(all_s)
    lap_num_arr = np.concatenate(all_lap)
    elev_arr    = np.concatenate(all_elev)
    grade_arr   = np.concatenate(all_grade)

    # Smooth velocity while preserving v=0 stop segments
    from scipy.signal import savgol_filter as _sgf
    v_smooth = v_arr.copy()
    # Find non-zero segments and smooth them individually
    in_motion = (v_arr > 0.0)
    # Find start/end indices of each contiguous motion segment
    motion_starts = np.where(np.diff(np.concatenate([[0], in_motion.astype(int)])) == 1)[0]
    motion_ends   = np.where(np.diff(np.concatenate([in_motion.astype(int), [0]])) == -1)[0] + 1
    for ms, me in zip(motion_starts, motion_ends):
        seg = v_arr[ms:me]
        if len(seg) > 25:   # only smooth if segment is long enough
            v_smooth[ms:me] = _sgf(seg, window_length=21, polyorder=2)
        v_smooth[ms:me] = np.clip(v_smooth[ms:me], 0.0, V_MAX)
    v_arr = v_smooth

    return t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr


# ── Verification ───────────────────────────────────────────────────────────────

def _verify(t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr):
    total_dist_m = float(np.trapezoid(v_arr, t_arr))   # ∫v dt
    total_dist_km = total_dist_m / 1000.0
    duration_s = float(t_arr[-1])
    duration_min = duration_s / 60.0
    avg_speed_ms = total_dist_m / duration_s
    avg_speed_kmh = avg_speed_ms * 3.6

    # Count lap-end stops: zero-velocity segments where lap number changes
    # A stop is a block of v==0 steps; count distinct stop blocks
    v_zero = (v_arr == 0.0)
    # find transitions from non-zero to zero
    stop_starts = np.where(np.diff(v_zero.astype(int)) == 1)[0] + 1
    n_stops = len(stop_starts)

    # Stop duration: each stop block length × DT
    # Check first stop block duration
    if n_stops > 0:
        first_stop = stop_starts[0]
        # find end of first stop block
        k = first_stop
        while k < len(v_arr) and v_arr[k] == 0.0:
            k += 1
        first_stop_dur = (k - first_stop) * DT
    else:
        first_stop_dur = 0.0

    print()
    print("── Verification ──────────────────────────────────")
    print(f"  Lap distance (CSV)  : {D_LAP:.3f} m")
    print(f"  N laps              : {N_LAPS}")
    print(f"  Total distance      : {total_dist_km:.3f} km  (need > 14.5 km)")
    print(f"  Duration            : {duration_min:.1f} min   (need 34–35 min)")
    print(f"  Avg speed           : {avg_speed_kmh:.2f} km/h (target ≈ 25.8 km/h)")
    print(f"  Min velocity        : {float(np.min(v_arr))*3.6:.2f} km/h  ✓")
    print(f"  Lap-end stops       : {n_stops}  (need 11)")
    print(f"  Stop duration each  : {first_stop_dur:.1f} s  ✓")

    ok_dist = total_dist_km > 14.5
    ok_time = 34.0 <= duration_min <= 35.0
    ok_stop = n_stops == 11

    verdict_ok = ok_dist and ok_time and ok_stop
    verdict = "✓ All checks passed" if verdict_ok else (
        "✗ Issues: " +
        ("dist<14.5km " if not ok_dist else "") +
        ("time out of 34-35min " if not ok_time else "") +
        ("stops≠11 " if not ok_stop else "")
    )
    print(f"VERDICT: {verdict}")
    print()

    return total_dist_km, duration_min, avg_speed_kmh


# ── Plotting ───────────────────────────────────────────────────────────────────

def _get_lap1_mask(lap_num_arr):
    return lap_num_arr == 1


def _color_by_grade(grade, v_or_empty=None, mode_arr=None):
    """Return colour string based on grade."""
    if grade < -GRADE_THRESH:
        return 'blue'
    elif grade > +GRADE_THRESH:
        return 'red'
    else:
        return 'grey'


def _make_figure(t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr):
    fig = plt.figure(figsize=(16, 14))
    fig.suptitle(
        "Hydraix I — Elevation-Aware Velocity Profile | Silesia Ring SEM 2026 | "
        f"11 laps × 1319.6 m = 14.52 km",
        fontsize=13, fontweight='bold', y=0.98
    )

    # ── Grid layout: 4 rows
    # Row 1 (top): panel 1 — full width elevation profile
    # Row 2: panel 2 (left) + panel 3 (right)
    # Row 3: panel 4 — full width velocity vs time (all laps)
    # Row 4: panel 5 (left) + panel 6 (right)
    gs = fig.add_gridspec(4, 2, hspace=0.45, wspace=0.3,
                          height_ratios=[1, 1.1, 1.2, 1.1])

    ax1  = fig.add_subplot(gs[0, :])    # Panel 1 — full width
    ax2  = fig.add_subplot(gs[1, 0])    # Panel 2 — left
    ax3  = fig.add_subplot(gs[1, 1])    # Panel 3 — right
    ax4  = fig.add_subplot(gs[2, :])    # Panel 4 — full width
    ax5  = fig.add_subplot(gs[3, 0])    # Panel 5 — left
    ax6  = fig.add_subplot(gs[3, 1])    # Panel 6 — right

    t_min = t_arr / 60.0

    # ══════════════════════════════════════════════════════════════════════════
    # Panel 1: Elevation profile of one lap, coloured by grade
    # ══════════════════════════════════════════════════════════════════════════
    s_route = _s_raw
    e_route = _e_smooth
    g_route = _grad_smooth

    # Colour segments
    for i in range(len(s_route) - 1):
        g = g_route[i]
        if g < -GRADE_THRESH:
            c = 'steelblue'
        elif g > +GRADE_THRESH:
            c = 'tomato'
        else:
            c = 'lightgrey'
        ax1.fill_between(s_route[i:i+2], e_route[i:i+2],
                         min(e_route) - 0.05,
                         color=c, alpha=0.5, linewidth=0)
        ax1.plot(s_route[i:i+2], e_route[i:i+2], color=c, linewidth=1.5)

    ax1.set_xlabel('Distance along lap [m]', fontsize=9)
    ax1.set_ylabel('Elevation [m]', fontsize=9)
    ax1.set_title(
        'Silesia Ring Route — Elevation Profile (1 lap = 1319.6 m)',
        fontsize=10, fontweight='bold'
    )
    ax1.set_xlim(0, D_LAP)

    # Secondary twin x-axis (cumulative distance, same as primary here)
    ax1_top = ax1.twiny()
    ax1_top.set_xlim(0, D_LAP)
    ax1_top.set_xlabel('Cumulative distance [m]', fontsize=8)

    # Legend
    patch_up   = mpatches.Patch(color='tomato',    alpha=0.7, label='Uphill')
    patch_dn   = mpatches.Patch(color='steelblue', alpha=0.7, label='Downhill')
    patch_flat = mpatches.Patch(color='lightgrey', alpha=0.7, label='Flat')
    ax1.legend(handles=[patch_up, patch_dn, patch_flat],
               loc='upper right', fontsize=8)

    # ══════════════════════════════════════════════════════════════════════════
    # Panel 2: Velocity vs distance — Lap 1 only
    # ══════════════════════════════════════════════════════════════════════════
    mask1 = _get_lap1_mask(lap_num_arr)
    s1    = s_arr[mask1]
    v1    = v_arr[mask1] * 3.6   # km/h
    g1    = grade_arr[mask1]
    P1    = P_arr[mask1]

    # Determine segment colour: uphill=red, downhill=blue, brake=black,
    # flat-pulse=orange, flat-glide=green
    def _seg_color_v_dist(i):
        if g1[i] < -GRADE_THRESH:
            return 'steelblue'
        elif g1[i] > +GRADE_THRESH:
            return 'tomato'
        elif v1[i] == 0.0:
            return 'black'
        elif P1[i] > 0:
            return 'darkorange'
        else:
            return 'mediumseagreen'

    for i in range(len(s1) - 1):
        c = _seg_color_v_dist(i)
        ax2.plot(s1[i:i+2], v1[i:i+2], color=c, linewidth=0.8)

    ax2.axhline(V_HIGH * 3.6, color='gray', linestyle='--', linewidth=0.8, label=f'V_HIGH={V_HIGH*3.6:.0f} km/h')
    ax2.axhline(V_LOW  * 3.6, color='gray', linestyle=':',  linewidth=0.8, label=f'V_LOW={V_LOW*3.6:.0f} km/h')
    ax2.set_xlabel('Distance in lap [m]', fontsize=9)
    ax2.set_ylabel('Velocity [km/h]', fontsize=9)
    ax2.set_title('Velocity vs Distance — Lap 1 (elevation-aware P&G)', fontsize=9, fontweight='bold')
    ax2.set_xlim(0, D_LAP)

    legend_handles2 = [
        mpatches.Patch(color='tomato',        label='Uphill boost'),
        mpatches.Patch(color='steelblue',     label='Downhill coast'),
        mpatches.Patch(color='darkorange',    label='Flat pulse'),
        mpatches.Patch(color='mediumseagreen',label='Flat glide'),
        mpatches.Patch(color='black',         label='Brake'),
    ]
    ax2.legend(handles=legend_handles2, fontsize=7, loc='upper right')

    # ══════════════════════════════════════════════════════════════════════════
    # Panel 3: Grade vs distance — one lap
    # ══════════════════════════════════════════════════════════════════════════
    ax3.plot(_s_raw, _grad_smooth * 100.0, color='dimgray', linewidth=1.0,
             label='Grade (%)')
    ax3.axhline(0, color='black', linewidth=0.5)
    ax3.fill_between(_s_raw,
                     np.where(_grad_smooth > GRADE_THRESH, _grad_smooth * 100.0, 0.0),
                     0.0,
                     color='tomato', alpha=0.4, label=f'Uphill (grade > {GRADE_THRESH*100:.1f}%)')
    ax3.fill_between(_s_raw,
                     np.where(_grad_smooth < -GRADE_THRESH, _grad_smooth * 100.0, 0.0),
                     0.0,
                     color='steelblue', alpha=0.4, label=f'Downhill (grade < -{GRADE_THRESH*100:.1f}%)')
    ax3.set_xlabel('Distance in lap [m]', fontsize=9)
    ax3.set_ylabel('Grade [%]', fontsize=9)
    ax3.set_title('Grade Profile — Silesia Ring (1 lap)', fontsize=9, fontweight='bold')
    ax3.set_xlim(0, D_LAP)
    ax3.legend(fontsize=7, loc='upper right')

    # ══════════════════════════════════════════════════════════════════════════
    # Panel 4: Velocity vs time — all 11 laps
    # ══════════════════════════════════════════════════════════════════════════
    ax4.plot(t_min, v_arr * 3.6, color='royalblue', linewidth=0.6, alpha=0.85,
             label='Velocity')

    # Mark lap-end stops with grey vertical bands
    v_zero = (v_arr == 0.0)
    in_stop = False
    stop_start_t = None
    for i, vz in enumerate(v_zero):
        if vz and not in_stop:
            in_stop = True
            stop_start_t = t_min[i]
        elif not vz and in_stop:
            in_stop = False
            ax4.axvspan(stop_start_t, t_min[i-1], color='lightgrey',
                        alpha=0.6, zorder=0)
    if in_stop:
        ax4.axvspan(stop_start_t, t_min[-1], color='lightgrey', alpha=0.6, zorder=0)

    ax4.axhline(V_HIGH * 3.6, color='darkorange', linestyle='--', linewidth=0.8,
                label=f'V_HIGH = {V_HIGH*3.6:.0f} km/h')
    ax4.axhline(V_LOW  * 3.6, color='green',      linestyle='--', linewidth=0.8,
                label=f'V_LOW = {V_LOW*3.6:.0f} km/h')
    ax4.set_xlabel('Time [min]', fontsize=9)
    ax4.set_ylabel('Velocity [km/h]', fontsize=9)
    ax4.set_title(
        'Velocity vs Time — Full 11-Lap Race (with 4 s lap-end stops)',
        fontsize=9, fontweight='bold'
    )
    ax4.legend(fontsize=8, loc='upper right')
    ax4.set_xlim(0, t_min[-1])

    # ══════════════════════════════════════════════════════════════════════════
    # Panel 5: Cumulative distance vs time
    # ══════════════════════════════════════════════════════════════════════════
    cum_dist_km = np.cumsum(v_arr * DT) / 1000.0
    ax5.plot(t_min, cum_dist_km, color='royalblue', linewidth=1.2)
    ax5.axhline(14.5, color='red', linestyle='--', linewidth=1.0,
                label='14.5 km target')
    ax5.set_xlabel('Time [min]', fontsize=9)
    ax5.set_ylabel('Cumulative Distance [km]', fontsize=9)
    ax5.set_title('Cumulative Distance vs Time', fontsize=9, fontweight='bold')
    ax5.legend(fontsize=8)
    ax5.set_xlim(0, t_min[-1])

    # ══════════════════════════════════════════════════════════════════════════
    # Panel 6: Motor power — first 4 minutes
    # ══════════════════════════════════════════════════════════════════════════
    first4_mask = t_min <= 4.0
    ax6.plot(t_min[first4_mask], P_arr[first4_mask],
             color='darkorange', linewidth=0.8)
    ax6.axhline(P_PULSE, color='green',  linestyle='--', linewidth=0.8,
                label=f'P_PULSE = {P_PULSE:.0f} W')
    ax6.axhline(P_BOOST, color='tomato', linestyle='--', linewidth=0.8,
                label=f'P_BOOST = {P_BOOST:.0f} W')
    ax6.set_xlabel('Time [min]', fontsize=9)
    ax6.set_ylabel('Motor P_elec [W]', fontsize=9)
    ax6.set_title('Motor Power Demand — first 4 min', fontsize=9, fontweight='bold')
    ax6.legend(fontsize=8)
    ax6.set_xlim(0, 4.0)
    ax6.set_ylim(-50, 1300)

    fig.savefig(OUTPUT_PNG, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Figure saved → {OUTPUT_PNG}")


# ── Save CSV ───────────────────────────────────────────────────────────────────

def _save_csv(t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr):
    df = pd.DataFrame({
        'time_s':       t_arr,
        'velocity_ms':  v_arr,
        'velocity_kmh': v_arr * 3.6,
        'dist_in_lap_m':s_arr,
        'lap_num':      lap_num_arr,
        'elevation_m':  elev_arr,
        'grade':        grade_arr,
        'P_elec_W':     P_arr,
    })
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"  CSV saved      → {OUTPUT_CSV}")
    print(f"  CSV rows       : {len(df)}")


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("Building elevation-aware velocity profile ...")
    print(f"  Route: D_LAP = {D_LAP:.3f} m")

    t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr = build_full_profile()

    total_km, dur_min, avg_kmh = _verify(
        t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr
    )

    print("Saving outputs ...")
    _save_csv(t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr)
    _make_figure(t_arr, v_arr, P_arr, s_arr, lap_num_arr, elev_arr, grade_arr)

    print()
    print("Done.")
