"""
Motor Model — BAFANG RM G060.1000 hub motor, 2D lookup table.

Data source: ../datasheets/motor_lookup_table.xlsx
  Grid: 36 speed points (5–40 km/h) × 35 torque points (1–35 Nm)
  Outputs: I_dc_A, V_eff_V, eta_v1_pct

Public API
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
_LUT_PATH = os.path.join(_THIS_DIR, '..', 'datasheets', 'motor_lookup_table.xlsx')

# ── Wheel geometry ─────────────────────────────────────────────────────────────
R_WHEEL = 0.295   # m — verified: P_out = Torque × v / R at every LUT row

# ── 2D LUT loader ──────────────────────────────────────────────────────────────
def _load_motor_lut():
    df = pd.read_excel(_LUT_PATH)
    speeds  = np.array(sorted(df['Speed_kmh'].unique()))   # 5–40 km/h, 36 values
    torques = np.array(sorted(df['Torque_Nm'].unique()))   # 1–35 Nm,   35 values
    def _interp(col):
        mat = df.pivot(index='Torque_Nm', columns='Speed_kmh', values=col).values
        return RegularGridInterpolator((torques, speeds), mat,
                                       method='linear', bounds_error=False,
                                       fill_value=None)
    return {'eta':  _interp('eta_v1_pct'),
            'I_dc': _interp('I_dc_A'),
            'V_eff': _interp('V_eff_V')}

_MOTOR_LUT = _load_motor_lut()


def motor_lookup_2d(speed_kmh, torque_nm):
    """
    2D BAFANG LUT: (Speed_kmh, Torque_Nm) → (I_dc_A, V_eff_V, eta).

    Returns zeros for zero-torque / zero-speed points (coast, stop).
    Inputs may be scalars or arrays — always returns arrays.
    """
    spd = np.atleast_1d(np.asarray(speed_kmh, float))
    trq = np.atleast_1d(np.asarray(torque_nm, float))
    moving = (trq > 0.05) & (spd > 0.5)
    pts = np.column_stack([np.clip(trq, 1., 35.), np.clip(spd, 5., 40.)])
    I   = np.where(moving, _MOTOR_LUT['I_dc'](pts),       0.)
    V   = np.where(moving, _MOTOR_LUT['V_eff'](pts),      0.)
    eta = np.where(moving, _MOTOR_LUT['eta'](pts) / 100., 0.)
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
