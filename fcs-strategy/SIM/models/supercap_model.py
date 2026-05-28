"""
Supercapacitor Model — VINATech VEL13353R8257G Lithium-Ion Capacitor bank.

Configuration: 20 parallel × 16 series (SEM 2026)
  V_max  = 16 × 3.8 V  = 60.8 V
  V_min  = 16 × 2.5 V  = 40.0 V   ← cell minimum from datasheet (hard limit)
  C_bank = 250 F/cell × 20P / 16S  = 312.5 F
  ESR    = 100 mΩ/cell × 16S / 20P = 80 mΩ
  E_usable = ½ C (V_max² − V_min²) = 327,600 J = 91.0 Wh

SOC definition (energy-based, not charge-based):
  SOC = (V² − V_min²) / (V_max² − V_min²)
  This maps usable energy linearly to [0, 1].

Public API
──────────
  sc_voltage(soc)                  → OCV [V]
  sc_terminal_voltage(soc, I_A)    → loaded bus voltage [V]
  sc_soc_update(soc, P_sc_W, dt)   → new SOC (scalar)

Constants
─────────
  SC_C, SC_V_MAX, SC_V_MIN, SC_ESR, SC_ETA
  SC_E_J
  SC_SOC_0, SC_SOC_MIN, SC_SOC_MAX
"""

import numpy as np

# ── Bank constants ─────────────────────────────────────────────────────────────
SC_C       = 312.5    # F      bank capacitance
SC_V_MAX   =  60.8    # V      max bus voltage  (16 × 3.8 V)
SC_V_MIN   =  40.0    # V      min bus voltage  (16 × 2.5 V — datasheet hard limit)
SC_ESR     =   0.080  # Ω      DC ESR: 100 mΩ/cell × 16S / 20P
SC_ETA     =   0.97   # —      round-trip charge/discharge efficiency
SC_E_J     = 0.5 * SC_C * (SC_V_MAX**2 - SC_V_MIN**2)  # ≈ 327,600 J = 91.0 Wh

# ── SOC reference points ───────────────────────────────────────────────────────
SC_SOC_0   = 0.60     # initial & reference SOC
SC_SOC_MIN = 0.15     # hard floor  (hard-clips in sc_soc_update)
SC_SOC_MAX = 0.95     # hard ceiling


def sc_voltage(soc):
    """
    SC open-circuit voltage [V] from energy-based SOC.

    V(SOC) = sqrt(V_min² + SOC × (V_max² − V_min²))

    This is exact for a capacitor: E = ½CV² → SOC = (V²-V_min²)/(V_max²-V_min²).
    Scalar or array input.
    """
    soc_c = np.clip(np.asarray(soc, dtype=float), 0.0, 1.0)
    return np.sqrt(SC_V_MIN**2 + soc_c * (SC_V_MAX**2 - SC_V_MIN**2))


def sc_terminal_voltage(soc, I_sc_A):
    """
    SC bus voltage under load [V]:  OCV(SOC) − I × ESR, clamped to [V_min, V_max].

    Positive I_sc_A means current flowing OUT of the SC (discharging → voltage sag).
    Returns a float scalar.
    """
    return float(np.clip(sc_voltage(soc) - float(I_sc_A) * SC_ESR,
                         SC_V_MIN, SC_V_MAX))


def sc_soc_update(soc, P_sc_W, dt):
    """
    Update SC state of charge for one time step.

    Parameters
    ----------
    soc    : float   current SOC (0–1)
    P_sc_W : float   SC power [W]
                     > 0 → SC discharging (supplying load)
                     < 0 → SC charging (absorbing surplus from FC)
    dt     : float   time step [s]

    Returns
    -------
    float  new SOC, clamped to [SC_SOC_MIN, SC_SOC_MAX]

    Efficiency is applied asymmetrically:
      discharge: P_effective = P_sc / η   (SC provides more than load sees)
      charge:    P_effective = P_sc × η   (SC receives less than FC provides)
    """
    P_eff = P_sc_W / SC_ETA if P_sc_W > 0 else P_sc_W * SC_ETA
    return float(np.clip(soc - P_eff * dt / SC_E_J, SC_SOC_MIN, SC_SOC_MAX))
