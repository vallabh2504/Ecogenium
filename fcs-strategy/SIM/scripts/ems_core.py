"""
EMS Core — Shared infrastructure for Hydraix I energy management simulation.

System:  balticFuelCells LC 52.30 FC + HyCap 20P×16S supercap bank + BAFANG motor
Race:    Silesia Ring SEM 2026, 11 laps × 14.5 km, ~35 min (3.18 min/lap)

Supercapacitor module: HyCap 3.8V / 250F LIC cells, 20 parallel × 16 series
  V_max = 60.8 V,  V_min = 40.0 V (cell min 2.5 V × 16S),  C_bank = 312.5 F
  E_sc ≈ 91.0 Wh,  ESR ≈ 80 mΩ  (100 mΩ/cell × 16S / 20P).
  (mini-caps 2.7V / 50F not modelled — role in circuit TBD)

FC model: Chamberline-Kim polarization, calibrated to LC 52.30 nameplate.
  P_net_max = 1013 W @ 37.5 A (net, after BOP).
  P_net_min = 100 W (membrane-safe idle floor).

Public API
──────────
  build_demand_profile()    → (t_s, P_elec_W)  one-lap arrays at 5 Hz
  motor_eta(p_out_W)        → η  (0–1)
  motor_lookup_2d(speed_kmh, torque_nm) → (I_dc_A, V_eff_V, eta)
  fc_current(P_net_W)       → I_fc [A]
  fc_h2_rate(I_fc)          → ṁ_H2 [g/s]
  sc_voltage(soc)           → V_sc [V]
  sc_terminal_voltage(soc, I_sc_A) → V_sc_loaded [V]
  sc_soc_update(soc, P_sc_W, dt) → new_soc
  simulate_race(strategy_fn, SOC_0, verbose, fc_can_off) → results dict

Constants exported
──────────────────
  FC_P_MIN, FC_P_MAX, FC_RAMP
  SC_C, SC_V_MAX, SC_V_MIN, SC_ESR, SC_ETA, SC_E_J
  SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX
  K_H2, LHV_H2, ETA_FC_REF, ECMS_DENOM
  N_LAPS, DT, RESAMPLE_HZ
  RESULTS_DIR
  R_WHEEL
"""

import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator
import os

# ── Paths ─────────────────────────────────────────────────────────────────────
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
MATLAB_DIR  = os.path.join(_THIS_DIR, '..', 'data')
RESULTS_DIR = os.path.join(_THIS_DIR, '..', 'plots')
os.makedirs(RESULTS_DIR, exist_ok=True)

# ── Simulation constants ───────────────────────────────────────────────────────
N_LAPS      = 11
RESAMPLE_HZ = 5
DT          = 1.0 / RESAMPLE_HZ   # 0.2 s

# ── Vehicle / drivetrain ───────────────────────────────────────────────────────
_MASS   = 175.0
_G      = 9.81
_CD     = 0.15
_AF     = 0.8      # m²  frontal area
_CRR    = 0.006
_RHO    = 1.225    # kg/m³ air density
_ETA_DT = 0.95     # drivetrain

# ── FC operating envelope ──────────────────────────────────────────────────────
FC_P_MAX  = 1013.0   # W net  (37.5 A)
FC_P_MIN  = 100.0    # W net  (idle floor — never drop below to protect membrane)
FC_RAMP   = 100.0    # W/s    max ramp rate

# ── BAFANG RM G060.1000 6T 90A 48V — 61-point efficiency lookup ───────────────
_M_POUT = np.array([
     24.19,  24.32,  24.25,  28.44,  28.30,  32.42,  36.48,  40.42,
     44.70,  52.41,  56.89,  64.51,  72.95,  76.80,  84.89,  92.97,
     96.76, 113.48, 116.92, 129.35, 137.79, 144.77, 173.82, 177.86,
    206.16, 218.28, 229.22, 258.04, 258.71, 290.29, 310.45, 320.87,
    353.88, 357.90, 390.07, 414.20, 421.14, 461.25, 477.30, 498.74,
    541.47, 550.61, 590.51, 628.07, 636.72, 676.51, 716.31, 728.37,
    771.91, 815.46, 830.83, 874.14, 921.39, 947.83, 972.64,1030.08,
   1070.29,1091.75,1127.55,1170.18,1213.38
])
_M_ETA = np.array([
    0.220, 0.218, 0.220, 0.251, 0.249, 0.279, 0.298, 0.314,
    0.335, 0.383, 0.395, 0.423, 0.453, 0.452, 0.481, 0.492,
    0.500, 0.565, 0.531, 0.571, 0.569, 0.582, 0.634, 0.620,
    0.666, 0.652, 0.669, 0.673, 0.706, 0.713, 0.719, 0.713,
    0.747, 0.728, 0.757, 0.764, 0.745, 0.772, 0.767, 0.765,
    0.792, 0.776, 0.791, 0.809, 0.786, 0.799, 0.811, 0.793,
    0.806, 0.816, 0.803, 0.813, 0.824, 0.817, 0.811, 0.826,
    0.828, 0.821, 0.819, 0.824, 0.834
])
_m_idx     = np.argsort(_M_POUT)
_M_POUT_S  = _M_POUT[_m_idx]
_M_ETA_S   = _M_ETA[_m_idx]

def motor_eta(p_out_W):
    """BAFANG RM G060.1000 efficiency (0–1) from shaft output power [W]."""
    return np.interp(np.maximum(p_out_W, 0.0), _M_POUT_S, _M_ETA_S,
                     left=_M_ETA_S[0], right=_M_ETA_S[-1])


R_WHEEL = 0.295   # m — wheel radius verified against LUT (P_out = Torque × v / R)

_LUT_PATH = os.path.join(_THIS_DIR, '..', 'datasheets', 'motor_lookup_table.xlsx')

def _load_motor_lut():
    df = pd.read_excel(_LUT_PATH)
    speeds  = np.array(sorted(df['Speed_kmh'].unique()))
    torques = np.array(sorted(df['Torque_Nm'].unique()))
    def _interp(col):
        mat = df.pivot(index='Torque_Nm', columns='Speed_kmh', values=col).values
        return RegularGridInterpolator((torques, speeds), mat,
                                       method='linear', bounds_error=False,
                                       fill_value=None)
    return {'eta': _interp('eta_v1_pct'),
            'I_dc': _interp('I_dc_A'),
            'V_eff': _interp('V_eff_V')}

_MOTOR_LUT = _load_motor_lut()

def motor_lookup_2d(speed_kmh, torque_nm):
    """2D BAFANG LUT: (Speed_kmh, Torque_Nm) → (I_dc_A, V_eff_V, eta).
    Returns zeros for zero-torque / zero-speed points (coast, stop)."""
    spd = np.atleast_1d(np.asarray(speed_kmh, float))
    trq = np.atleast_1d(np.asarray(torque_nm, float))
    moving = (trq > 0.05) & (spd > 0.5)
    pts = np.column_stack([np.clip(trq, 1., 35.), np.clip(spd, 5., 40.)])
    I   = np.where(moving, _MOTOR_LUT['I_dc'](pts),       0.)
    V   = np.where(moving, _MOTOR_LUT['V_eff'](pts),      0.)
    eta = np.where(moving, _MOTOR_LUT['eta'](pts) / 100., 0.)
    return I, V, eta


# ── Drive-cycle demand builder ─────────────────────────────────────────────────
def _movmean(x, w):
    return pd.Series(x).rolling(w, center=True, min_periods=1).mean().values

def build_demand_profile():
    """
    Build one-lap electrical demand at 5 Hz, GPS-calibrated to 14.5/11 km/lap.

    Returns
    -------
    t_lap  : ndarray, shape (N,)  — time within lap [s]
    P_elec : ndarray, shape (N,)  — electrical demand [W] (≥0)
    """
    raw = pd.read_csv(os.path.join(MATLAB_DIR, 'canonical_with_incline.csv'))
    lap_end = raw['elapsed_sec'].iloc[-1] / N_LAPS
    raw = raw[raw['elapsed_sec'] <= lap_end].reset_index(drop=True)

    t_raw   = raw['elapsed_sec'].values
    v_raw   = raw['Speed_kmh'].values / 3.6   # m/s
    inc_raw = raw['inc_angle_deg'].values

    smooth_pts = RESAMPLE_HZ * 10             # 10-second window

    def _lap_dist_km(scale):
        t_u = np.arange(t_raw[0], t_raw[-1], DT)
        v_u = np.interp(t_u, t_raw, v_raw * scale)
        v_s = _movmean(v_u, smooth_pts)
        return float(np.trapezoid(v_s, t_u) / 1000.0)

    target = 14.5 / N_LAPS
    lo, hi = 0.80, 1.05
    for _ in range(60):
        mid = (lo + hi) / 2
        if _lap_dist_km(mid) < target:
            lo = mid
        else:
            hi = mid
    scale = (lo + hi) / 2

    t_u   = np.arange(t_raw[0], t_raw[-1], DT)
    v_u   = np.interp(t_u, t_raw, v_raw * scale)
    inc_u = np.interp(t_u, t_raw, inc_raw)
    v_s   = _movmean(v_u, smooth_pts)
    accel = np.gradient(v_s, DT)

    P_wheel = (_CRR * _MASS * _G * v_s
               + 0.5 * _CD * _AF * _RHO * v_s**3
               + _MASS * accel * v_s
               + _MASS * _G * np.sin(np.deg2rad(inc_u)) * v_s)
    P_motor = np.where(P_wheel > 0, P_wheel / _ETA_DT, 0.0)
    eta_m   = motor_eta(P_motor)
    P_elec  = np.where(P_motor > 0, P_motor / eta_m, 0.0)
    P_elec  = np.clip(np.where(np.isfinite(P_elec), P_elec, 0.0), 0.0, None)

    # Reset time to zero for clean lap indexing
    t_lap = t_u - t_u[0]
    return t_lap, P_elec


# ── FC electrochemical model (Chamberline-Kim, LC 52.30) ──────────────────────
_N  = 52;    _Ac = 30.0;   _Tk = 338.15
_Fc = 96485.0; _Rg = 8.314;  _ne = 2
_E0 = 0.955;  _b  = 0.058;  _J0 = 0.010;  _ASR = 0.095;  _JL = 1.65
_la = 2.0;    _xO2= 0.2095; _ra = 1.20;   _Ma  = 28.97e-3
_PP = 3.2;    _PE = 5.0;    _BI = 3.0;    _BE  = 0.30

# Blower system-resistance constant (5000 Pa at nominal flow)
_Q_nom = (_la * _N * 37.5 * _Ma) / (_ne * 2 * _Fc * _xO2 * _ra)
_K_SYS = 5000.0 / _Q_nom**2

def _vcell(I):
    J   = I / _Ac
    ea  = np.where(J > _J0, _b * np.log(J / _J0), 0.0)
    eo  = _ASR * J
    ec  = -(_Rg * _Tk / (_ne * _Fc)) * np.log(1.0 - np.minimum(J, _JL * 0.999) / _JL)
    return np.clip(_E0 - ea - eo - ec, 0.0, _E0)

def _p_blower(I):
    Q = (_la * _N * I * _Ma) / (_ne * 2 * _Fc * _xO2 * _ra)
    return np.maximum(_BI, _K_SYS * Q**3 / _BE)

def _pnet(I):
    return np.maximum(0.0, _N * _vcell(I) * I - _p_blower(I) - _PP - _PE)

# Pre-build P_net inversion table (monotonic over operating range)
_I_TAB = np.linspace(0.1, 40.0, 8000)
_P_TAB = _pnet(_I_TAB)

def fc_current(P_net_W):
    """Invert polarization curve: P_net [W] → I_fc [A]. Vectorized."""
    P_c = np.clip(np.asarray(P_net_W, dtype=float), FC_P_MIN, FC_P_MAX)
    return np.interp(P_c, _P_TAB, _I_TAB)

# Faraday factor [g H2 per C = g/(A·s)]
K_H2 = _N * 2.016 / (2.0 * _Fc)   # ≈ 5.430e-4 g/C

def fc_h2_rate(I_fc):
    """H2 mass consumption rate [g/s] from stack current [A]. Vectorized."""
    return K_H2 * np.asarray(I_fc, dtype=float)

# FC reference efficiency (LHV basis at nominal operating point)
LHV_H2     = 120_000.0                                    # J/g
_I_ref     = float(fc_current(np.array([FC_P_MAX]))[0])
_mh2_ref   = float(fc_h2_rate(np.array([_I_ref]))[0])     # g/s
ETA_FC_REF = FC_P_MAX / (_mh2_ref * LHV_H2)               # ≈ 0.415
ECMS_DENOM = ETA_FC_REF * LHV_H2                          # ≈ 50 000 W/(g/s)


# ── Supercapacitor model ───────────────────────────────────────────────────────
# HyCap 3.8V / 250F Lithium-Ion Capacitor cells — 20P × 16S bank
#   V_max  = 16 × 3.8  = 60.8 V
#   V_min  = 16 × 2.5  = 40.0 V  (cell datasheet minimum: 2.5 V)
#   C_bank = 20 × 250F / 16 = 312.5 F
#   ESR    ≈ (16 × 3 mΩ_cell) / 20 ≈ 2.4 mΩ  (estimated; confirm from datasheet)
SC_C       = 312.5    # F
SC_V_MAX   =  60.8    # V
SC_V_MIN   =  40.0    # V   (cell minimum 2.5 V × 16S)
SC_ESR     = 0.080    # Ω  — VINATech DC ESR: 100 mΩ/cell × 16S / 20P
SC_ETA     = 0.97     # round-trip efficiency
SC_E_J     = 0.5 * SC_C * (SC_V_MAX**2 - SC_V_MIN**2)   # ≈ 327 600 J = 91.0 Wh

SC_SOC_0   = 0.60     # initial & reference SOC
SC_SOC_MIN = 0.15     # hard floor
SC_SOC_MAX = 0.95     # hard ceiling

def sc_voltage(soc):
    """SC terminal voltage [V] from V²-based SOC definition."""
    soc_c = np.clip(np.asarray(soc, dtype=float), 0.0, 1.0)
    return np.sqrt(SC_V_MIN**2 + soc_c * (SC_V_MAX**2 - SC_V_MIN**2))

def sc_terminal_voltage(soc, I_sc_A):
    """SC bus voltage under load: OCV(SOC) - I * SC_ESR, clamped to [V_min, V_max]."""
    return float(np.clip(sc_voltage(soc) - float(I_sc_A) * SC_ESR,
                         SC_V_MIN, SC_V_MAX))

def sc_soc_update(soc, P_sc_W, dt):
    """
    Scalar SOC update given supercap power [W] and timestep [s].

    Sign convention:  P_sc > 0  → SC discharging (supplying load)
                      P_sc < 0  → SC charging (absorbing FC surplus)

    Converter efficiency η_sc is applied asymmetrically:
      discharge: SC must provide more energy → P_effective = P_sc / η_sc
      charge:    SC receives less energy     → P_effective = P_sc * η_sc
    """
    P_eff = P_sc_W / SC_ETA if P_sc_W > 0 else P_sc_W * SC_ETA
    return float(np.clip(soc - P_eff * dt / SC_E_J, SC_SOC_MIN, SC_SOC_MAX))


# ── Race simulation engine ─────────────────────────────────────────────────────
def simulate_race(strategy_fn, SOC_0=SC_SOC_0, verbose=False, fc_can_off=False):
    """
    Run the full 11-lap race simulation with the given energy management strategy.

    Parameters
    ----------
    strategy_fn : callable
        Signature: fn(P_dem, SOC, P_fc_prev, t_in_lap, lap_idx) → P_fc_setpoint [W]
        Called once per timestep. P_fc_prev is the FC output at the previous step.
        t_in_lap is time since the start of the current lap [s].
        lap_idx is 0-based (0 … 10).

    SOC_0 : float
        Initial and target-reference SC state of charge (default 0.60).

    verbose : bool
        Print per-lap SOC and H2 summary if True.

    fc_can_off : bool
        If True: FC can be fully shut down (P_fc = 0 allowed). Upward ramp is
        limited by FC_RAMP; instantaneous shutdown is permitted. Used by
        strategies with glide phases (B, C). I_fc is set to 0 when
        P_fc_cmd < FC_P_MIN to prevent fc_current() clip artefacts.
        If False (default): FC is always held in [FC_P_MIN, FC_P_MAX] and
        both up- and down-ramps are rate-limited.

    Returns
    -------
    dict with fields:
        t           ndarray (N,)     time [s]
        P_dem       ndarray (N,)     electrical demand [W]
        P_fc        ndarray (N,)     FC net power [W]
        P_sc        ndarray (N,)     SC power [W] (+ve = discharge)
        SOC         ndarray (N,)     SC state of charge at start of each step
        I_fc        ndarray (N,)     FC stack current [A]
        m_H2_total  float            total H2 consumed [g]
        delta_SOC   float            SOC(end) − SOC(0)
        SOC_final   float            SOC at end of race
        lap_SOC     ndarray (12,)    SOC at end of each lap (index 0 = initial)
        N_pts_lap   int              number of 5 Hz samples per lap
        T_lap       float            lap duration [s]
    """
    t_lap, P_lap = build_demand_profile()
    N_pts = len(t_lap)
    T_lap = float(t_lap[-1])

    # Tile demand and time for all laps
    t_all  = np.concatenate([t_lap + i * T_lap for i in range(N_LAPS)])
    P_dem  = np.tile(P_lap, N_LAPS)
    n      = N_LAPS * N_pts

    P_fc_ar  = np.empty(n)
    P_sc_ar  = np.empty(n)
    SOC_ar   = np.empty(n + 1)
    I_fc_ar  = np.empty(n)
    m_H2_ar  = np.empty(n)
    lap_SOC  = np.empty(N_LAPS + 1)

    SOC_ar[0]  = SOC_0
    lap_SOC[0] = SOC_0
    # FC starts off when glide is allowed; otherwise held at idle floor
    P_fc_prev  = 0.0 if fc_can_off else FC_P_MIN
    P_min      = 0.0 if fc_can_off else FC_P_MIN

    for k in range(n):
        lap_idx  = k // N_pts
        t_in_lap = float(t_all[k] - lap_idx * T_lap)
        soc_k    = SOC_ar[k]
        Pd       = float(P_dem[k])

        # ── Strategy decision ──────────────────────────────────────────────
        P_fc_cmd = float(strategy_fn(Pd, soc_k, P_fc_prev, t_in_lap, lap_idx))

        # ── FC ramp-rate and saturation constraints ─────────────────────────
        if fc_can_off:
            # Allow instant shutdown; only limit upward ramp
            if P_fc_cmd > P_fc_prev:
                P_fc_cmd = min(P_fc_cmd, P_fc_prev + FC_RAMP * DT)
        else:
            # Symmetric ramp limiting in both directions
            P_fc_cmd = float(np.clip(P_fc_cmd,
                                     P_fc_prev - FC_RAMP * DT,
                                     P_fc_prev + FC_RAMP * DT))
        P_fc_cmd = float(np.clip(P_fc_cmd, P_min, FC_P_MAX))

        P_sc_k = Pd - P_fc_cmd

        # ── SC floor protection: force FC max before SC hits hard floor ─────
        trial_soc = sc_soc_update(soc_k, P_sc_k, DT)
        if P_sc_k > 0 and trial_soc <= SC_SOC_MIN:
            P_fc_cmd = float(np.clip(Pd, P_min, FC_P_MAX))
            P_sc_k   = max(0.0, Pd - P_fc_cmd)
            trial_soc = sc_soc_update(soc_k, P_sc_k, DT)

        P_fc_ar[k]   = P_fc_cmd
        P_sc_ar[k]   = P_sc_k
        SOC_ar[k + 1] = trial_soc
        # Guard: fc_current() clips to FC_P_MIN, so set I=0 when FC is truly off
        I_fc_ar[k]   = float(fc_current(P_fc_cmd)) if P_fc_cmd >= FC_P_MIN else 0.0
        m_H2_ar[k]   = fc_h2_rate(I_fc_ar[k]) * DT
        P_fc_prev    = P_fc_cmd

        if (k + 1) % N_pts == 0:
            lap = (k + 1) // N_pts
            lap_SOC[lap] = SOC_ar[k + 1]
            if verbose:
                lap_h2 = float(np.sum(m_H2_ar[k + 1 - N_pts: k + 1]))
                print(f"  Lap {lap:2d}  SOC={lap_SOC[lap]:.3f}  H2={lap_h2:.3f} g")

    return {
        't':          t_all,
        'P_dem':      P_dem,
        'P_fc':       P_fc_ar,
        'P_sc':       P_sc_ar,
        'SOC':        SOC_ar[:-1],
        'I_fc':       I_fc_ar,
        'm_H2_total': float(np.sum(m_H2_ar)),
        'delta_SOC':  float(SOC_ar[n] - SOC_0),
        'SOC_final':  float(SOC_ar[n]),
        'lap_SOC':    lap_SOC,
        'N_pts_lap':  N_pts,
        'T_lap':      T_lap,
    }
