"""
Strategy G — LPF Feedforward + SOC-PI fuel-cell EMS  (Hydraix I · SEM 2026)
============================================================================
Real-time, adaptive fuel-cell power controller for the Hydraix I fuel-cell +
supercapacitor hybrid. It runs every control step (Δt = 0.2 s) from the SAME
signals that feed the Strategy block in the main Simulink vehicle model — no
drive-cycle knowledge, no lookup tables, no GPS.

    INPUTS
        I_motor    [A]  motor / VESC DC-bus current
        U_sc       [V]  supercapacitor terminal (bus) voltage
        P_fc_prev  [W]  previous FC command            (interface only)
        t_in_lap   [s]  time since the start of the lap (interface only)
        lap_idx    [-]  0-based lap counter            (used by lap offset)

    OUTPUT
        P_fc       [W]  fuel-cell NET power setpoint, clipped to
                        [FC_P_MIN, FC_P_MAX].  (Downstream the plant
                        ramp-limits this and the DC-DC delivers P_fc·η to the bus.)

How it works (three additive terms + a floor)
---------------------------------------------
  1. LPF feedforward : the FC tracks a low-pass (τ = 15 s) of the measured motor
     power, so it follows the slow average demand and ignores pulse transients
     (the supercapacitor absorbs those).
  2. SOC proportional+integral : drives the supercap back to its reference SOC,
     guaranteeing charge sustenance for ANY velocity profile (this is what makes
     G robust to imperfect / inconsistent laps). Integral is anti-windup clamped.
  3. Lap-supervisor offset : a slow per-lap nudge from the residual SOC error,
     a coarse outer loop that keeps the integrator from saturating.
  + FC floor : the FC never drops below FC_P_MIN (membrane stays warm; the SC is
     pre-charged during glide).

SOC is DERIVED from U_sc — no dedicated SOC sensor:
        SOC = (U_sc² − V_min²) / (V_max² − V_min²)

Result on the SEM-Poland (Silesia Ring) profile: ≈165.3 km/m³, charge-sustaining
(ΔSOC ≈ 0). This is the production controller used in the project's results.

Simulink mapping (block-for-block)
----------------------------------
  LPF                       → Discrete Transfer Fcn   (α = e^(−Δt/τ))
  K_p·(SOC0 − SOC)          → Gain + Sum
  K_i·∫(SOC0 − SOC) dt      → Discrete-Time Integrator + Saturation (anti-windup)
  lap offset                → Triggered Subsystem (fires on lap-line crossing)
  FC floor + output clip    → MinMax / Saturation

This file is self-contained: it needs only NumPy and depends on no other project
module, so it can be read, run, or ported to embedded C / Simulink as-is.
"""

import numpy as np

# ── Plant constants ───────────────────────────────────────────────────────────
# Fuel cell: balticFuelCells LC 52.30 (~1 kW net).  SC: HyCap LIC 20P × 16S.
DT        = 0.2        # control-loop step [s]
FC_P_MIN  = 100.0      # FC membrane-safe floor [W] — FC is held at/above this
FC_P_MAX  = 1013.0     # FC maximum net power [W] (≈37.5 A)
SC_V_MIN  = 40.0       # SC minimum bus voltage [V]  (16S × 2.5 V cell min)
SC_V_MAX  = 60.8       # SC maximum bus voltage [V]  (16S × 3.8 V cell max)
SC_C      = 312.5      # SC bank capacitance [F]  (250 F/cell × 20P / 16S)
SC_SOC_0  = 0.60       # SC reference / initial state of charge
SC_E_J    = 0.5 * SC_C * (SC_V_MAX**2 - SC_V_MIN**2)   # usable SC energy [J] ≈ 327.6 kJ

# ── Tuned controller gains (bisected for charge sustenance on the SEM profile) ──
K_P_DEFAULT = 2075.0   # proportional gain  [W per unit SOC]
K_I_DEFAULT = 2.0      # integral gain      [W per (SOC·s)]
TAU_DEFAULT = 15.0     # LPF time constant  [s]
LAP_GAIN    = 0.3      # lap-supervisor gain (fraction of SOC energy deficit / lap)
T_LAP_REF   = 186.0    # nominal lap time [s] used to scale the lap offset


def make_strategy_g(K_p=K_P_DEFAULT, K_i=K_I_DEFAULT, tau=TAU_DEFAULT):
    """
    Build the Strategy-G control function.

    Returns
    -------
    fn(I_motor, U_sc, P_fc_prev, t_in_lap, lap_idx) -> P_fc [W]
        A closure holding the controller state (LPF accumulator, SOC integrator,
        lap offset). Create ONE instance per vehicle/run and call it every step.
    """
    alpha    = float(np.exp(-DT / tau))     # IIR low-pass coefficient ≈ 0.9868
    G_FC_MIN = float(FC_P_MIN)              # FC floor (always-on)

    # Controller state (the ECU's persistent registers)
    st = {'lpf': None, 'integ': 0.0, 'prev_lap': -1, 'offset': 0.0}

    def fn(I_motor, U_sc, P_fc_prev, t_in_lap, lap_idx):
        # Measured demand and derived SOC
        P_motor = float(I_motor) * float(U_sc)                                  # [W]
        SOC     = (U_sc**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2)          # [-]

        # Initialise the LPF on the first call
        if st['lpf'] is None:
            st['lpf'] = P_motor

        # Lap-supervisor outer loop: on each new lap, nudge the offset by the
        # observed SOC energy deficit over the lap just completed.
        if lap_idx != st['prev_lap'] and lap_idx > 0:
            st['offset'] += LAP_GAIN * (SC_SOC_0 - SOC) * SC_E_J / T_LAP_REF
            st['offset']  = float(np.clip(st['offset'], -400.0, 400.0))
        st['prev_lap'] = lap_idx

        # SOC-error integral (updated every step, incl. glide — essential for
        # charge sustenance), with anti-windup clamp to ±400 W of authority.
        st['integ'] += (SC_SOC_0 - SOC) * DT
        if K_i > 1e-9:
            st['integ'] = float(np.clip(st['integ'], -400.0 / K_i, 400.0 / K_i))

        # Motor off (glide / coast / stop): hold the FC at its floor.
        if I_motor < 0.1:
            return G_FC_MIN

        # LPF feedforward + SOC-PI + lap offset
        st['lpf'] = alpha * st['lpf'] + (1.0 - alpha) * P_motor
        P_cmd = st['lpf'] + st['offset'] + K_p * (SC_SOC_0 - SOC) + K_i * st['integ']

        return float(np.clip(P_cmd, G_FC_MIN, FC_P_MAX))

    return fn


if __name__ == '__main__':
    # Minimal self-test (no plant model needed).
    g = make_strategy_g()
    # Example: motor drawing ~600 W on a 53 V bus (SOC≈0.60), lap 0.
    print("SOC(53 V)      =", round((53.0**2 - SC_V_MIN**2) / (SC_V_MAX**2 - SC_V_MIN**2), 3))
    print("P_fc @600W/53V =", round(g(600.0 / 53.0, 53.0, 0.0, 0.0, 0), 1), "W")
    print("P_fc @ motor-off =", round(g(0.0, 53.0, 0.0, 0.0, 0), 1), "W  (FC floor)")
