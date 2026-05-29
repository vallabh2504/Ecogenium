"""
profile_comparison.py — Compare three driving profiles with Strategy G EMS.

Profiles:
  A: Python P&G (sem_combined_best.csv)
  B: MATLAB elevation-aware P&G (sem_2026_elev_aware.csv)
  C: Canonical real telemetry (canonical_with_incline.csv), scaled to 14.5 km

Each profile is simulated with Strategy G (K_i=2.0, K_p bisected for charge
sustenance) and the resulting km/m³, H2, dSOC, K_p and avg motor efficiency
are printed in a comparison table.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as mgridspec

from ems_core import (
    FC_P_MIN, FC_P_MAX, FC_RAMP, LHV_H2, ETA_DCDC,
    SC_C, SC_V_MAX, SC_V_MIN, SC_E_J, SC_ESR, SC_ETA, SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX,
    K_H2, fc_current, sc_soc_update, sc_voltage, sc_terminal_voltage,
    motor_lookup_2d, R_WHEEL,
    N_LAPS, DT, MATLAB_DIR, RESULTS_DIR,
)

# ── Vehicle constants (must match combined_best_profile.py) ────────────────────
MASS    = 180.
G       = 9.81
CRR     = 0.006
CD      = 0.15
AF      = 1.35
RHO     = 1.225
H2_DENSITY = 0.0899       # kg/m³ at STP
TOTAL_KM   = 14.5         # total race distance [km]
G_FC_MIN   = 150.         # Strategy G FC floor [W]


# ── Step 1 — Strategy G ───────────────────────────────────────────────────────
def make_strat_g(K_p, K_i=2., tau=15.):
    alpha = float(np.exp(-DT / tau))
    st = {'lpf': None, 'integ': 0., 'prev_lap': -1, 'offset': 0.}

    def fn(I_motor, U_sc, Pp, til, li):
        P_motor = I_motor * U_sc
        SOC = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)
        if st['lpf'] is None:
            st['lpf'] = float(P_motor)
        if li != st['prev_lap'] and li > 0:
            st['offset'] += 0.3 * (SC_SOC_0 - SOC) * SC_E_J / 186.
            st['offset'] = float(np.clip(st['offset'], -400., 400.))
        st['prev_lap'] = li
        st['integ'] += (SC_SOC_0 - SOC) * DT
        if K_i > 1e-9:
            st['integ'] = float(np.clip(st['integ'], -400. / K_i, 400. / K_i))
        if I_motor < 0.1:
            return G_FC_MIN
        st['lpf'] = alpha * st['lpf'] + (1 - alpha) * float(P_motor)
        Pc = st['lpf'] + st['offset'] + K_p * (SC_SOC_0 - SOC) + K_i * st['integ']
        return float(np.clip(Pc, G_FC_MIN, FC_P_MAX))

    return fn


# ── Step 2 — Motor signals and simulation ─────────────────────────────────────
def compute_motor_signals(va, ga):
    """Vehicle dynamics → electrical demand via 2D BAFANG LUT."""
    accel       = np.gradient(va, DT)
    P_wheel     = (CRR * MASS * G * va
                   + 0.5 * CD * AF * RHO * va**3
                   + MASS * accel * va
                   + MASS * G * ga * va)
    P_wheel_pos = np.maximum(P_wheel, 0.)
    v_safe      = np.maximum(va, 0.3)
    Torque_Nm   = (P_wheel_pos / v_safe) * R_WHEEL
    Speed_kmh   = va * 3.6
    I_dc, V_eff, _ = motor_lookup_2d(Speed_kmh, Torque_Nm)
    P_elec      = I_dc * V_eff
    return P_elec, V_eff, Torque_Nm


def simulate_from_arrays(fn, va, ga, la, ca):
    """
    Run EMS simulation on pre-built velocity/grade/lap/coast arrays.

    Parameters
    ----------
    fn  : strategy function (I_motor, U_sc, Pp, til, li) -> P_fc [W]
    va  : velocity [m/s], shape (N,)
    ga  : grade [-] (sin of incline angle), shape (N,)
    la  : lap index array (1-based integers), shape (N,)
    ca  : coast-to-stop boolean array, shape (N,)

    Returns
    -------
    dict with m_H2, dSOC, SOC, Pfc, Psc, eta_motor, total_km
    """
    P_elec, V_eff, Torque_Nm = compute_motor_signals(va, ga)

    n = len(P_elec)
    Pfc_arr  = np.empty(n)
    Psc_arr  = np.empty(n)
    SOCa     = np.empty(n + 1)
    mH2_arr  = np.empty(n)
    eta_arr  = np.zeros(n)   # motor efficiency at each step

    laps_u   = np.unique(la)
    lt0      = {int(l): np.where(la == l)[0][0] for l in laps_u}

    SOCa[0] = SC_SOC_0
    pp = FC_P_MIN
    v_min = float(SC_V_MIN)

    for k in range(n):
        s       = SOCa[k]
        P_elec_k = float(P_elec[k])
        li      = int(la[k]) - 1          # 0-based lap index for strategy fn
        til     = float((k - lt0[li + 1]) * DT)

        # 2-step ESR correction
        U_oc  = float(sc_voltage(s))
        I_m   = P_elec_k / max(U_oc, v_min)
        U_sc  = sc_terminal_voltage(s, I_m)
        I_m   = P_elec_k / max(U_sc, v_min)
        P_demand = P_elec_k

        # EMS decision
        Pc = float(fn(I_m, U_sc, pp, til, li))
        # Ramp-rate limit
        Pc = float(np.clip(Pc, pp - FC_RAMP * DT, pp + FC_RAMP * DT))

        is_coast = bool(ca[k])
        fc_min   = 0. if is_coast else FC_P_MIN
        Pc = float(np.clip(Pc, fc_min, FC_P_MAX))

        # FC delivers Pc × ETA_DCDC to the bus (boost-converter loss)
        Psk = P_demand - Pc * ETA_DCDC
        ts  = sc_soc_update(s, Psk, DT)

        # SC floor protection
        if Psk > 0 and ts <= SC_SOC_MIN:
            Pc  = float(np.clip(P_demand / ETA_DCDC, FC_P_MIN, FC_P_MAX))
            Psk = max(0., P_demand - Pc * ETA_DCDC)
            ts  = sc_soc_update(s, Psk, DT)

        Pfc_arr[k] = Pc
        Psc_arr[k] = Psk
        SOCa[k + 1] = ts

        I_fc_k = float(fc_current(Pc)) if Pc >= FC_P_MIN else 0.
        mH2_arr[k] = K_H2 * I_fc_k * DT
        pp = Pc

        # Motor efficiency (LUT-based) — only when motor is on
        if P_elec_k > 1. and Torque_Nm[k] > 0.05:
            _, _, eta_k = motor_lookup_2d(va[k] * 3.6, Torque_Nm[k])
            eta_arr[k] = float(np.atleast_1d(eta_k)[0])

    total_km = float(np.sum(va * DT)) / 1000.
    m_H2 = float(np.sum(mH2_arr)); dSOC = float(SOCa[n] - SC_SOC_0)
    E_fc_J = float(np.sum(Pfc_arr) * DT)
    # Charge-sustaining-normalised H2 (dSOC=0 basis) — see combined_best_profile
    eta_sc_ow = SC_ETA ** 0.5
    m_H2_norm = m_H2 - (dSOC * SC_E_J / (eta_sc_ow * ETA_DCDC) / E_fc_J) * m_H2 if E_fc_J > 0 else m_H2
    return {
        'm_H2'     : m_H2,
        'm_H2_norm': m_H2_norm,
        'dSOC'     : dSOC,
        'SOC'      : SOCa[:-1],
        'Pfc'      : Pfc_arr,
        'Psc'      : Psc_arr,
        'eta_motor': eta_arr,
        'total_km' : total_km,
    }


# ── Step 3 — Load profiles ────────────────────────────────────────────────────

def load_profile_a():
    """Profile A: Python P&G from sem_combined_best.csv."""
    path = os.path.join(MATLAB_DIR, 'sem_combined_best.csv')
    df = pd.read_csv(path)
    va = df['velocity_ms'].values.astype(float)
    # grade column is already sin(angle) in combined_best
    ga = df['grade'].values.astype(float) if 'grade' in df.columns else np.zeros(len(va))
    la = df['lap_num'].values.astype(int)
    # Coast detection: velocity < 0.5 m/s and decelerating from >1 m/s, or v=0 for >1s
    ca = _detect_coast(va)
    print(f"  Profile A loaded: {len(va)} steps, {len(va)*DT:.0f}s, "
          f"laps {la.min()}-{la.max()}, coast steps={int(ca.sum())}")
    return va, ga, la, ca


def load_profile_b():
    """Profile B: MATLAB elevation-aware P&G from sem_2026_elev_aware.csv."""
    path = os.path.join(MATLAB_DIR, 'sem_2026_elev_aware.csv')
    df = pd.read_csv(path)
    va = df['velocity_ms'].values.astype(float)
    ga = df['grade'].values.astype(float) if 'grade' in df.columns else np.zeros(len(va))
    la = df['lap_num'].values.astype(int)
    ca = _detect_coast(va)
    print(f"  Profile B loaded: {len(va)} steps, {len(va)*DT:.0f}s, "
          f"laps {la.min()}-{la.max()}, coast steps={int(ca.sum())}")
    return va, ga, la, ca


def load_profile_c():
    """Profile C: canonical real telemetry, scaled to 14.5 km, split into 11 laps."""
    path = os.path.join(MATLAB_DIR, 'canonical_with_incline.csv')
    df = pd.read_csv(path)

    t_raw   = df['elapsed_sec'].values.astype(float)
    v_raw   = df['Speed_kmh'].values.astype(float) / 3.6      # m/s
    inc_raw = df['inc_angle_deg'].values.astype(float)

    # Bisect speed scale so total distance = 14.5 km
    def _total_dist_km(scale):
        t_u = np.arange(t_raw[0], t_raw[-1], DT)
        v_u = np.interp(t_u, t_raw, v_raw * scale)
        return float(np.sum(v_u * DT)) / 1000.

    lo, hi = 0.5, 2.0
    for _ in range(20):
        mid = (lo + hi) / 2
        if _total_dist_km(mid) < TOTAL_KM:
            lo = mid
        else:
            hi = mid
    scale = (lo + hi) / 2
    print(f"  Profile C: speed scale={scale:.4f}  (total dist={_total_dist_km(scale):.3f} km)")

    # Resample to 5 Hz
    t_u   = np.arange(t_raw[0], t_raw[-1], DT)
    va    = np.interp(t_u, t_raw, v_raw * scale)
    inc_u = np.interp(t_u, t_raw, inc_raw)
    ga    = np.sin(np.deg2rad(inc_u))

    # Split into 11 equal laps
    n   = len(va)
    lap_size = n // N_LAPS
    la  = np.zeros(n, dtype=int)
    for i in range(N_LAPS):
        start = i * lap_size
        end   = (i + 1) * lap_size if i < N_LAPS - 1 else n
        la[start:end] = i + 1

    # Coast: steps where va < 0.3 m/s
    ca = va < 0.3

    print(f"  Profile C loaded: {n} steps, {n*DT:.0f}s, "
          f"laps 1-{la.max()}, coast steps={int(ca.sum())}")
    return va, ga, la, ca


def _detect_coast(va):
    """
    Detect coast-to-stop phases:
      - velocity < 0.5 m/s AND was decelerating from >1 m/s in last 5 steps, OR
      - velocity = 0 for more than 1 s (5 steps at 5 Hz)
    """
    n = len(va)
    ca = np.zeros(n, dtype=bool)
    WINDOW = 5
    for k in range(n):
        if va[k] < 0.5:
            # Check if previously above 1 m/s in the last WINDOW steps
            lo = max(0, k - WINDOW)
            if np.any(va[lo:k] > 1.0):
                ca[k] = True
            elif va[k] == 0.:
                # Check if zero for more than 1s (5 steps)
                lo2 = max(0, k - WINDOW)
                if np.all(va[lo2:k + 1] == 0.):
                    ca[k] = True
    return ca


# ── Step 4 — Bisect K_p for charge sustenance ─────────────────────────────────
def bisect_kp(va, ga, la, ca, label=''):
    """Bisect K_p in [100, 8000] with K_i=2.0 fixed."""
    print(f"\n  Bisecting K_p for Strategy G on '{label}'")
    lo, hi = 100., 8000.
    best_result = None
    best_kp     = (lo + hi) / 2.
    for it in range(20):
        kp = (lo + hi) / 2.
        fn = make_strat_g(kp, K_i=2.0)
        r  = simulate_from_arrays(fn, va, ga, la, ca)
        ds = r['dSOC']
        print(f"    iter{it+1:2d}  K_p={kp:7.1f}  dSOC={ds:+.4f}  H2={r['m_H2']:.3f}g  "
              f"km/m³={r['total_km'] / (r['m_H2'] / 1000. / H2_DENSITY):.1f}")
        best_result = r
        best_kp     = kp
        if abs(ds) <= 0.015:
            print(f"    => Converged: K_p={kp:.1f}  dSOC={ds:+.4f}")
            break
        # dSOC < -tol means SC is depleting → need higher K_p to charge more
        if ds < -0.015:
            lo = kp
        else:
            hi = kp
        if hi - lo < 0.5:
            break
    return best_kp, best_result


# ── Step 5 — km/m³ calculation ────────────────────────────────────────────────
def compute_km_per_m3(result):
    """Compute km/m³ from the charge-sustaining-normalised H2 (dSOC=0 basis)."""
    m_H2_kg = result['m_H2_norm'] / 1000.      # g → kg
    vol_m3   = m_H2_kg / H2_DENSITY            # kg / (kg/m³) = m³
    return result['total_km'] / vol_m3


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=== Loading profiles ===")
    print("Profile A — Python P&G")
    va_a, ga_a, la_a, ca_a = load_profile_a()

    print("\nProfile B — MATLAB elevation-aware P&G")
    va_b, ga_b, la_b, ca_b = load_profile_b()

    print("\nProfile C — Canonical real telemetry")
    va_c, ga_c, la_c, ca_c = load_profile_c()

    print("\n=== Bisecting K_p (Strategy G, K_i=2.0) ===")

    kp_a, r_a = bisect_kp(va_a, ga_a, la_a, ca_a, 'A: Python P&G (VH=9, VL=7)')
    kp_b, r_b = bisect_kp(va_b, ga_b, la_b, ca_b, 'B: MATLAB elev-aware P&G')
    kp_c, r_c = bisect_kp(va_c, ga_c, la_c, ca_c, 'C: Canonical telemetry')

    # Compute km/m³ and average motor efficiency
    def avg_eta(result, va):
        motor_on = (result['eta_motor'] > 0.)
        if motor_on.sum() == 0:
            return 0.
        return float(np.mean(result['eta_motor'][motor_on]))

    km3_a = compute_km_per_m3(r_a)
    km3_b = compute_km_per_m3(r_b)
    km3_c = compute_km_per_m3(r_c)

    eta_a = avg_eta(r_a, va_a)
    eta_b = avg_eta(r_b, va_b)
    eta_c = avg_eta(r_c, va_c)

    # ── Step 6 — Print comparison table ──────────────────────────────────────
    print("\n")
    print("=== Profile Comparison (Strategy G, K_i=2.0) ===")
    hdr = f"{'Profile':<26}| {'km/m³':>6} | {'H2[g]':>6} | {'dSOC':>6} | {'K_p':>6} | {'Avg v [m/s]':>11} | {'Motor eta':>9}"
    sep = "-" * len(hdr)
    print(hdr)
    print(sep)

    def fmt_row(name, km3, result, kp, va):
        avg_v = float(np.mean(va))
        eta_m = avg_eta(result, va)
        return (f"{name:<26}| {km3:>6.1f} | {result['m_H2']:>6.3f} | "
                f"{result['dSOC']:>+6.4f} | {kp:>6.0f} | {avg_v:>11.3f} | {eta_m*100:>8.1f}%")

    print(fmt_row("A: Python P&G (VH=9,VL=7)", km3_a, r_a, kp_a, va_a))
    print(fmt_row("B: MATLAB elev-aware P&G",  km3_b, r_b, kp_b, va_b))
    print(fmt_row("C: Canonical telemetry",     km3_c, r_c, kp_c, va_c))
    print(sep)

    print("\nAverage motor efficiency (mean η_LUT over motor-on steps):")
    print(f"  A: {eta_a*100:.2f}%")
    print(f"  B: {eta_b*100:.2f}%")
    print(f"  C: {eta_c*100:.2f}%")

    print(f"\nBest km/m³: {max(km3_a, km3_b, km3_c):.1f}")

    # ── Figure: 3 subplots ────────────────────────────────────────────────────
    profiles = [
        ('A: Python P&G (VH=9,VL=7)', va_a, la_a, r_a, km3_a, kp_a),
        ('B: MATLAB elev-aware P&G',   va_b, la_b, r_b, km3_b, kp_b),
        ('C: Canonical telemetry',      va_c, la_c, r_c, km3_c, kp_c),
    ]
    colors = ['#D55E00', '#0072B2', '#009E73']

    fig, axes = plt.subplots(3, 3, figsize=(18, 13), dpi=120)
    fig.suptitle('Profile Comparison — Strategy G EMS (K_i=2.0)\n'
                 f'A: {km3_a:.1f} km/m³  |  B: {km3_b:.1f} km/m³  |  C: {km3_c:.1f} km/m³',
                 fontsize=13, fontweight='bold', y=1.01)

    for col, (name, va, la, result, km3, kp) in enumerate(profiles):
        t_arr  = np.arange(len(va)) * DT / 60.    # time in minutes
        ax_v   = axes[0][col]
        ax_soc = axes[1][col]
        ax_fc  = axes[2][col]

        c = colors[col]

        # Row 0: velocity
        ax_v.plot(t_arr, va * 3.6, color=c, lw=0.8, alpha=0.8)
        ax_v.set_title(f'{name}\n{km3:.1f} km/m³  K_p={kp:.0f}  '
                       f'H2={result["m_H2"]:.3f}g  dSOC={result["dSOC"]:+.4f}',
                       fontsize=8.5, fontweight='bold')
        ax_v.set_ylabel('Velocity [km/h]', fontsize=8)
        ax_v.set_xlabel('Time [min]', fontsize=8)
        ax_v.tick_params(labelsize=7)
        ax_v.spines['top'].set_visible(False)
        ax_v.spines['right'].set_visible(False)

        # Row 1: SOC
        ax_soc.plot(t_arr, result['SOC'], color=c, lw=1.2)
        ax_soc.axhline(SC_SOC_0, color='grey', lw=0.8, ls='--', alpha=0.6, label=f'SOC_0={SC_SOC_0}')
        ax_soc.axhline(SC_SOC_MIN, color='red', lw=0.7, ls=':', alpha=0.5, label='SOC_min')
        ax_soc.set_ylabel('SC SOC [—]', fontsize=8)
        ax_soc.set_xlabel('Time [min]', fontsize=8)
        ax_soc.legend(fontsize=7, loc='upper right')
        ax_soc.set_ylim(0., 1.)
        ax_soc.tick_params(labelsize=7)
        ax_soc.spines['top'].set_visible(False)
        ax_soc.spines['right'].set_visible(False)

        # Row 2: FC power
        ax_fc.plot(t_arr, result['Pfc'], color=c, lw=0.7, alpha=0.85)
        ax_fc.axhline(FC_P_MIN, color='grey', lw=0.8, ls='--', alpha=0.6, label=f'FC_MIN={FC_P_MIN:.0f}W')
        ax_fc.axhline(G_FC_MIN, color='orange', lw=0.8, ls=':', alpha=0.6, label=f'G_FC_MIN={G_FC_MIN:.0f}W')
        ax_fc.set_ylabel('FC Power [W]', fontsize=8)
        ax_fc.set_xlabel('Time [min]', fontsize=8)
        ax_fc.legend(fontsize=7, loc='upper right')
        ax_fc.set_ylim(0., FC_P_MAX + 50.)
        ax_fc.tick_params(labelsize=7)
        ax_fc.spines['top'].set_visible(False)
        ax_fc.spines['right'].set_visible(False)

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, 'profile_comparison.png')
    fig.savefig(out_path, dpi=120, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"\nFigure saved → {out_path}")


if __name__ == '__main__':
    main()
