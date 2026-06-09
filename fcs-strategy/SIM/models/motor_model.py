"""
Motor Model — BAFANG RM G060.1000 hub motor, measured RPM×Torque efficiency map.

Data source: ../datasheets/motor_eff_rpm_torque.xlsx ('Efficiency Lookup')
  The team's AUTHORITATIVE measured map (output/wheel-referenced), replacing the
  archived motor_lookup_table.xlsx. η ≈ 0.17–0.83 (peak 83.4 %).

Public API (interface preserved for back-compat)
──────────
  motor_lookup_2d(speed_kmh, torque_nm) → (I_dc_A, V_eff_V, eta)
  motor_eta(p_out_W)                    → η  (0–1)  [1D fallback]
  R_WHEEL                               = 0.295 m
"""

import os
import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MOTOR_XLSX = os.path.join(_THIS_DIR, '..', 'datasheets', 'motor_eff_rpm_torque.xlsx')

# ── Wheel geometry ─────────────────────────────────────────────────────────────
R_WHEEL = 0.295   # m — verified: P_out = Torque × v / R
MOTOR_V = 48.0    # motor is always fed 48 V (I_dc = P_elec / 48)
RPM_PER_MS = 60.0 / (2 * np.pi) / R_WHEEL

# ── Measured efficiency map loader ─────────────────────────────────────────────
def _load_motor_eta():
    df = pd.read_excel(_MOTOR_XLSX, sheet_name='Efficiency Lookup')
    tq  = np.array([float(c) for c in df.columns[1:]])
    rp  = df.iloc[:, 0].values.astype(float)
    eta = df.iloc[:, 1:].values.astype(float) / 100.0
    itp = RegularGridInterpolator((rp, tq), eta, method='linear',
                                  bounds_error=False, fill_value=None)
    return itp, (float(rp.min()), float(rp.max())), (float(tq.min()), float(tq.max()))

_MOTOR_ETA, _RPM_RNG, _TQ_RNG = _load_motor_eta()


def motor_lookup_2d(speed_kmh, torque_nm):
    """
    (Speed_kmh, Torque_Nm) → (I_dc_A, V_eff_V, eta) on the MEASURED map.

    V_eff = 48 V; I_dc = P_elec/48 where P_elec = P_mech/η = P_wheel/η. Zeros for
    coast/stop points. Inputs may be scalars or arrays — always returns arrays.
    """
    spd = np.atleast_1d(np.asarray(speed_kmh, float))
    trq = np.atleast_1d(np.asarray(torque_nm, float))
    moving = (trq > 0.05) & (spd > 0.5)
    v   = spd / 3.6
    rp  = np.clip(v * RPM_PER_MS, _RPM_RNG[0], _RPM_RNG[1])
    tq  = np.clip(trq,            _TQ_RNG[0],  _TQ_RNG[1])
    eta = np.where(moving, np.clip(_MOTOR_ETA(np.column_stack([rp, tq])), 0.05, 0.999), 0.)
    P_mech = trq * (np.maximum(v, 0.3) / R_WHEEL)
    P_elec = np.where(moving, P_mech / np.where(eta > 0, eta, 1.), 0.)
    I = P_elec / MOTOR_V
    V = np.where(moving, MOTOR_V, 0.)
    return I, V, eta


# ── 1D fallback (61-point shaft-power curve, legacy use) ──────────────────────
_M_POUT = np.array([
     24.19,  24.32,  24.25,  28.44,  28.30,  32.42,  36.48,  40.42,
     44.70,  52.41,  56.89,  64.51,  72.95,  76.80,  84.89,  92.97,
     96.76, 113.48, 116.92, 129.35, 137.79, 144.77, 173.82, 177.86,
    206.16, 218.28, 229.22, 258.04, 258.71, 290.29, 310.45, 320.87,
    353.88, 357.90, 390.07, 414.20, 421.14, 461.25, 477.30, 498.74,
    541.47, 550.61, 590.51, 628.07, 636.72, 676.51, 716.31, 728.37,
    771.91, 815.46, 830.83, 874.14, 921.39, 947.83, 972.64, 1030.08,
   1070.29, 1091.75, 1127.55, 1170.18, 1213.38
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
_idx      = np.argsort(_M_POUT)
_M_POUT_S = _M_POUT[_idx]
_M_ETA_S  = _M_ETA[_idx]

def motor_eta(p_out_W):
    """1D BAFANG efficiency from shaft output power [W]. Legacy — prefer motor_lookup_2d."""
    return np.interp(np.maximum(p_out_W, 0.0), _M_POUT_S, _M_ETA_S,
                     left=_M_ETA_S[0], right=_M_ETA_S[-1])
