"""
FC Stack LC 52.30 — System-Level Simulation  v3
================================================
balticFuelCells GmbH | N=52 cells | A=30 cm² | P_nom=1040 W @ 37.5 A

CHANGES FROM v2
───────────────
BOP now uses real component specifications:
  • Air blower   : Boreasa C65B2-24PR-0B miniature centrifugal blower
                   (replaces generic isentropic blower model)
  • Coolant pump : DC-LT 2 micropump — fixed 3.2 W rated consumption
                   (replaces physics-based pump sizing)
  • Electronics  : Temperature sensors + ECU = 5 W fixed (user-measured)

Electrochemical model (Chamberline-Kim, expert-verified) is unchanged from v2.

═══════════════════════════════════════════════════════════════════════
ELECTROCHEMICAL MODEL                                          [Ref 1,6]
═══════════════════════════════════════════════════════════════════════
  V_cell = E₀ − η_act − η_ohm − η_conc

  η_act  = b · ln(J/J₀)                   Tafel activation loss
  η_ohm  = ASR · J                         Area-specific ohmic loss
  η_conc = −(RT/2F) · ln(1 − J/J_L)       Concentration (mass-transport) loss

  Calibrated to LC 52.30 nameplate:
    P_gross = 1040 W @ I = 37.5 A  → V_cell(J=1.25 A/cm²) = 0.535 V/cell ✓
    V_stack ≥ 26 V for all I ≤ 40 A                                       ✓
    H₂ supply ≈ 19 SLPM @ 37.5 A (λ_H2 = 1.40)                           ✓

═══════════════════════════════════════════════════════════════════════
HYDROGEN FLOW RATE  (Faraday's law)                           [Ref 2,5]
═══════════════════════════════════════════════════════════════════════
  ṅ_H2,consumed = N·I / (2·F)            [mol/s]
  Q_H2,supply   = λ_H2 · ṅ_H2 · 22.414 · 60   [SLPM]
  λ_H2 = 1.40  (dead-end + purge — confirms 19 SLPM spec)

═══════════════════════════════════════════════════════════════════════
BOP MODEL — REAL COMPONENT SPECIFICATIONS
═══════════════════════════════════════════════════════════════════════

1. AIR BLOWER — Boreasa C65B2-24PR-0B               [Ref 8]
   Miniature centrifugal blower, 24 VDC, brushless.
   Spec working point @ nominal 24 V:
       Q_wp  = 372 lpm,  ΔP_wp = 8 850 Pa,  P_N = 105.12 W
       η_wp  = Q_wp·ΔP_wp / P_N = (6.2×10⁻³ m³/s × 8 850 Pa) / 105.12 = 52.2 %
   Zero-flow stall: ΔP = 15 270 Pa,  P_stall = 24 V × 1.64 A = 39.4 W
   Free-flow:       Q  = 516 lpm,    P_free  = 24 V × 5.06 A = 121.4 W

   For the FC system the required air flow is only 70 lpm at I_nom (18.8 % of Q_wp).
   The blower operates at REDUCED SPEED via PWM driver to deliver exactly λ_air × stoichiometric.

   Power model: variable-speed centrifugal blower against a quadratic system resistance.
     (a) Required air flow: Q_air = λ_air · ṁ_O2,consumed · M_air / x_O2 / ρ_air   [m³/s]
                            Q_air(I) ∝ I
     (b) Cathode pressure drop (quadratic system resistance):
            ΔP_cathode(I) = k_sys · Q_air(I)²
            k_sys calibrated: ΔP = 5 000 Pa at Q_air(I_nom)  [centre of 20–100 mbar manual range]
     (c) Hydraulic power:  P_hyd = Q_air · ΔP_cathode = k_sys · Q_air³  ∝ I³
            [affinity law: for n ∝ Q against ΔP ∝ Q², shaft power ∝ Q³]             [Ref 3,4]
     (d) Electrical power: P_blower = max(P_bl_idle, P_hyd / η_eff)
            η_eff = 0.30  (off-design, low-flow region; η_wp = 52.2 % at design point)
            P_bl_idle = 3.0 W  (PWM driver + motor minimum current at low speed)

   Validation: P_blower(37.5 A) = 19.4 W  (1.9 % of P_gross — consistent with
   liquid-cooled near-atmospheric small stack literature [Ref 5,6])

2. COOLANT PUMP — DC-LT 2 micropump                 [Ref 9]
   Fixed-speed DC pump:  3.2 W  (rated electrical consumption from datasheet)
   Max flow: 72 L/h | Max head: 1 m | Medium: water / glycol | T_max: 65 °C
   Modelled as constant P_pump = 3.2 W (fixed-speed, single operating point)

3. ELECTRONICS — Temperature sensors + ECU          [user-measured]
   PT1000 temperature sensors, signal conditioning, microcontroller: 5.0 W  (fixed)

  P_BOP(I) = P_blower(I) + 3.2 + 5.0

═══════════════════════════════════════════════════════════════════════
SYSTEM EFFICIENCY  (LHV basis)                               [Ref 2,3]
═══════════════════════════════════════════════════════════════════════
  η_stack = V_cell / V_th,LHV           V_th,LHV = ΔH_LHV/(n·F) = 1.253 V
  η_sys   = P_net / (ṅ_H2,consumed · ΔH_LHV)    (dead-end: utilisation ≈ 99 %)

═══════════════════════════════════════════════════════════════════════
REFERENCES
═══════════════════════════════════════════════════════════════════════
[1] Kim J. et al. (1995) J. Electrochem. Soc. 142(8):2670–2674
[2] Kabza A. "Fuel Cell Formulary" v2.4 (2020). pemfc.de/FCF_A4.pdf
[3] Larminie J. & Dicks A. (2003) Fuel Cell Systems Explained, 2nd ed. Wiley
[4] Barbir F. (2012) PEM Fuel Cells: Theory and Practice, 2nd ed. Academic Press
[5] Scribner Associates FAQ. scribner.com — gas constants, BOP data
[6] Hao L. et al. (2016) Int. J. Chemical Engineering 2016:4109204
[7] balticFuelCells GmbH. User Manual FC Stack LCxx.30, Issue 05/2024
[8] Boreasa. Spec Sheet C65B2-24PR-0B Miniature Centrifugal Blower, Rev. X/0, Dec. 2024
[9] DC-LT 2 Micropump Technical Datasheet (3.2 W, 72 L/h, 1 m head, 65 °C max)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

# ── OUTPUT FOLDER ──────────────────────────────────────────────────────────────
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# ── PHYSICAL CONSTANTS ─────────────────────────────────────────────────────────
F          = 96_485.0      # Faraday constant                  [C/mol]
R_gas      = 8.314         # Universal gas constant            [J/mol/K]
M_H2       = 2.016e-3      # Molar mass H₂                    [kg/mol]
M_air      = 28.97e-3      # Molar mass dry air                [kg/mol]
x_O2       = 0.2095        # O₂ mole fraction in air
LHV_H2     = 241_820.0     # Lower heating value H₂           [J/mol]
V_mol_stp  = 22.414        # Molar volume at STP (0°C, 1 atm) [L/mol]
rho_air    = 1.20          # Air density at ~20°C, 1 atm      [kg/m³]
n_e        = 2             # Electrons per H₂ molecule

# ── STACK PARAMETERS (balticFuelCells FC Stack LC 52.30) [Ref 7] ───────────────
N          = 52            # Number of cells
A          = 30.0          # Active cell area                  [cm²]
T_K        = 338.15        # Operating temperature             [K] = 65°C
I_nom      = 37.5          # Nominal current (extended system) [A]
I_max      = 40.0          # Maximum rated current             [A]
V_min      = 26.0          # Minimum stack voltage             [V]
P_nom_spec = 1040.0        # Nameplate gross power             [W]

# ── POLARIZATION PARAMETERS (Chamberline-Kim, calibrated to LC 52.30) [Ref 1,6,7] ─
E0         = 0.955         # Practical OCV per cell            [V]
b          = 0.058         # Tafel slope = RT/(α·F), α≈0.5   [V]
J0         = 0.010         # Apparent exchange current density [A/cm²] — empirical fit;
                           #   not the intrinsic ORR value; absorbs reference-potential offset
ASR        = 0.095         # Area-specific resistance          [Ω·cm²]
J_L        = 1.65          # Limiting current density          [A/cm²]

# ── OPERATING CONDITIONS ───────────────────────────────────────────────────────
lam_H2     = 1.40          # H₂ stoichiometry (dead-end + purge)   [Ref 7]
lam_air    = 2.0           # Cathode air stoichiometry (≥ 2 required) [Ref 7]

# ── BOP: AIR BLOWER — Boreasa C65B2-24PR-0B [Ref 8] ─────────────────────────
# Working-point data (nominal 24 V):
BLOWER_Q_WP      = 372.0 / 60_000.0    # 372 lpm → m³/s
BLOWER_DP_WP     = 8_850.0              # Pa
BLOWER_P_WP      = 105.12              # W  (= 24 V × 4.38 A)
BLOWER_ETA_WP    = (BLOWER_Q_WP * BLOWER_DP_WP) / BLOWER_P_WP  # ≈ 0.522 (52.2 %)

# Off-design efficiency for our low-flow operating region (18–20 % of Q_wp)
# Centrifugal fans lose efficiency away from design point; conservative 30 % used
BLOWER_ETA_EFF   = 0.30

# Minimum idle power of PWM driver + motor at low speed  [W]
BLOWER_P_IDLE    = 3.0

# Cathode system-resistance constant, calibrated: ΔP = dP_cat_nom at Q_air(I_nom)
# ΔP_cathode(I) = k_sys · Q_air(I)²    (quadratic — fixed geometry)
_dP_cat_nom     = 5_000.0               # 50 mbar — centre of 20–100 mbar manual range [Ref 7]
_Q_air_nom       = (lam_air * N * I_nom * M_air
                    / (n_e * 2 * F * x_O2 * rho_air))             # m³/s at I_nom
K_SYS            = _dP_cat_nom / (_Q_air_nom ** 2)                 # Pa / (m³/s)²

# ── BOP: COOLANT PUMP — DC-LT 2 [Ref 9] ─────────────────────────────────────
P_PUMP_FIXED     = 3.2                  # W  (fixed rated consumption from datasheet)

# ── BOP: ELECTRONICS (temperature sensors + ECU) [user-measured] ─────────────
P_ELECTRONICS    = 5.0                  # W  (fixed)


# ══════════════════════════════════════════════════════════════════════════════
# ELECTROCHEMICAL MODEL
# ══════════════════════════════════════════════════════════════════════════════

def cell_voltage(I: np.ndarray) -> np.ndarray:
    """
    Chamberline-Kim polarization model [Ref 1].
    Three-term overpotential: activation (Tafel) + ohmic (ASR) + concentration (log form).
    """
    J        = I / A
    eta_act  = np.where(J > J0, b * np.log(J / J0), 0.0)
    eta_ohm  = ASR * J
    safe_J   = np.minimum(J, J_L * 0.999)
    eta_conc = -(R_gas * T_K / (n_e * F)) * np.log(1.0 - safe_J / J_L)
    return np.clip(E0 - eta_act - eta_ohm - eta_conc, 0.0, E0)


def stack_voltage(I: np.ndarray) -> np.ndarray:
    return N * cell_voltage(I)


def P_stack_gross(I: np.ndarray) -> np.ndarray:
    return stack_voltage(I) * I


# ══════════════════════════════════════════════════════════════════════════════
# HYDROGEN FLOW RATE
# ══════════════════════════════════════════════════════════════════════════════

def H2_molar_consumed(I: np.ndarray) -> np.ndarray:
    """Faraday's law: ṅ = N·I / (2·F)  [mol/s, stoichiometric]  [Ref 2]"""
    return N * I / (n_e * F)


def H2_mass_flow_gs(I: np.ndarray, lam: float = 1.0) -> np.ndarray:
    """H₂ mass flow [g/s].  lam=1 → consumed; lam=lam_H2 → supplied."""
    return lam * H2_molar_consumed(I) * M_H2 * 1000.0


def H2_SLPM(I: np.ndarray, lam: float = 1.0) -> np.ndarray:
    """H₂ volumetric flow [SLPM at 0°C, 1 atm].  [Ref 2]"""
    return lam * H2_molar_consumed(I) * V_mol_stp * 60.0


# ══════════════════════════════════════════════════════════════════════════════
# BALANCE OF PLANT
# ══════════════════════════════════════════════════════════════════════════════

def P_blower(I: np.ndarray) -> np.ndarray:
    """
    Boreasa C65B2-24PR-0B variable-speed centrifugal blower  [Ref 8].

    Physics: blower speed is modulated to deliver exactly λ_air × stoichiometric flow.
    Against a quadratic cathode resistance, power scales as Q³ (affinity law) [Ref 4]:

       P_elec = max(P_idle, Q_air³ · K_SYS / η_eff)

    where:
      Q_air(I)  = λ_air · N·I · M_air / (4·F · x_O2 · ρ_air)  [m³/s]
      ΔP(I)     = K_SYS · Q_air²                                [Pa]
      P_hyd(I)  = Q_air · ΔP  =  K_SYS · Q_air³                [W hydraulic]
      η_eff     = 0.30  (off-design; η_WP = 52.2 % at 372 lpm, 8 850 Pa)
      P_idle    = 3.0 W (PWM driver + motor at minimum speed)

    Spec reference: WP @ 24 V nominal: 372 lpm, 8 850 Pa, 105.12 W.
    """
    # O₂ consumption rate: ṅ_O₂ = N·I/(4·F)  (4 electrons per O₂)
    m_dot_air = lam_air * N * I * M_air / (n_e * 2 * F * x_O2)   # kg/s
    Q_air     = m_dot_air / rho_air                                # m³/s
    dP        = K_SYS * Q_air ** 2                                 # Pa  (quadratic resistance)
    P_hyd     = Q_air * dP                                         # W   (hydraulic power)
    return np.maximum(BLOWER_P_IDLE, P_hyd / BLOWER_ETA_EFF)      # W   (electrical)


def P_BOP(I: np.ndarray) -> np.ndarray:
    """Total BOP parasitic: blower + pump (3.2 W) + electronics (5 W)."""
    return P_blower(I) + P_PUMP_FIXED + P_ELECTRONICS


def P_net(I: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, P_stack_gross(I) - P_BOP(I))


# ══════════════════════════════════════════════════════════════════════════════
# EFFICIENCY
# ══════════════════════════════════════════════════════════════════════════════

def eta_stack_gross(I: np.ndarray) -> np.ndarray:
    """Stack gross efficiency: V_cell / V_th,LHV  (LHV basis)  [Ref 2]"""
    V_th_LHV = LHV_H2 / (n_e * F)                  # = 1.253 V
    return np.clip(cell_voltage(I) / V_th_LHV, 0.0, 1.0) * 100.0


def eta_system_net(I: np.ndarray) -> np.ndarray:
    """
    Net system efficiency on LHV basis  [Ref 2,3]:
        η_sys = P_net / (ṅ_H2,consumed · ΔH_LHV)
    Uses consumed H₂ (dead-end anode: utilisation ≈ 99 %).
    lam_H2 = 1.40 governs supply flow ONLY — it does NOT reduce efficiency
    because purge losses are negligible (<1 %).  [Ref 7]
    """
    P_chem = H2_molar_consumed(I) * LHV_H2
    return np.where(P_chem > 0,
                    np.clip(P_net(I) / P_chem, 0.0, 1.0) * 100.0,
                    0.0)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    I = np.linspace(0.5, I_max, 2000)

    Vcell      = cell_voltage(I)
    Vstack     = stack_voltage(I)
    Pgross     = P_stack_gross(I)
    Pbl        = P_blower(I)
    Ppump      = np.full_like(I, P_PUMP_FIXED)
    Pelec      = np.full_like(I, P_ELECTRONICS)
    Pbop       = P_BOP(I)
    Pnet       = P_net(I)
    H_cons_gs  = H2_mass_flow_gs(I, lam=1.0)
    H_supp_gs  = H2_mass_flow_gs(I, lam=lam_H2)
    H_cons_slpm= H2_SLPM(I, lam=1.0)
    H_supp_slpm= H2_SLPM(I, lam=lam_H2)
    Eff_g      = eta_stack_gross(I)
    Eff_s      = eta_system_net(I)

    # ── Summary table ─────────────────────────────────────────────────────────
    print("=" * 82)
    print(f"  FC Stack LC 52.30 — v3 Simulation  (N={N}, A={A} cm², T={T_K-273.15:.0f}°C)")
    print(f"  BOP: Boreasa C65B2-24PR-0B blower | DC-LT 2 pump (3.2 W) | Electronics (5 W)")
    print("=" * 82)
    print(f"  {'I [A]':>7}  {'V [V]':>8}  {'P_gross [W]':>12}  {'P_bl [W]':>9}  "
          f"{'P_bop [W]':>9}  {'P_net [W]':>10}  {'η_sys [%]':>10}  {'H2 [SLPM]':>10}")
    print("  " + "-" * 80)
    for I_c in [1.0, 5.0, 10.0, 20.0, 30.0, 37.5, 40.0]:
        idx = np.argmin(np.abs(I - I_c))
        print(f"  {I_c:7.1f}  {Vstack[idx]:8.2f}  {Pgross[idx]:12.1f}  {Pbl[idx]:9.2f}  "
              f"{Pbop[idx]:9.2f}  {Pnet[idx]:10.1f}  {Eff_s[idx]:10.1f}  "
              f"{H_supp_slpm[idx]:10.2f}")
    print("=" * 82)

    # Calibration checks
    idx_nom = np.argmin(np.abs(I - I_nom))
    idx_max = np.argmin(np.abs(I - I_max))
    print(f"\n  CALIBRATION CHECKS (electrochemical model unchanged from v2):")
    print(f"  P_gross @ {I_nom} A  = {Pgross[idx_nom]:.1f} W   spec: {P_nom_spec} W  "
          f"{'✓' if abs(Pgross[idx_nom]-P_nom_spec)<30 else '✗'}")
    print(f"  V_stack @ {I_max} A  = {Vstack[idx_max]:.2f} V  spec: ≥{V_min} V  "
          f"{'✓' if Vstack[idx_max]>=V_min else '✗'}")
    print(f"  H₂ supply @ {I_nom} A = {H_supp_slpm[idx_nom]:.2f} SLPM  spec: ≈19 SLPM  "
          f"{'✓' if abs(H_supp_slpm[idx_nom]-19.0)<0.5 else '✗'}")

    # Blower working-point validation
    Q_wp_check = BLOWER_Q_WP
    dP_wp_check= K_SYS * Q_wp_check ** 2
    P_hyd_wp   = Q_wp_check * dP_wp_check
    print(f"\n  BLOWER SPEC VALIDATION (Boreasa C65B2-24PR-0B):")
    print(f"  Working point: Q={BLOWER_Q_WP*60000:.0f} lpm, ΔP_wp_spec={BLOWER_DP_WP:.0f} Pa, "
          f"P_N_spec={BLOWER_P_WP:.1f} W")
    print(f"  η_wp (measured from spec) = {BLOWER_ETA_WP*100:.1f}%   "
          f"(used as reference for off-design η_eff = {BLOWER_ETA_EFF*100:.0f}%)")
    print(f"  P_blower @ I_nom = {Pbl[idx_nom]:.1f} W  ({Pbl[idx_nom]/Pgross[idx_nom]*100:.1f}% of P_gross)")

    # >55% efficiency range
    above55 = Eff_s >= 55.0
    print(f"\n  η_sys > 55%:  ", end="")
    if above55.any():
        I_lo = I[above55].min();  I_hi = I[above55].max()
        P_lo = Pnet[above55].min(); P_hi = Pnet[above55].max()
        print(f"I = {I_lo:.1f} – {I_hi:.1f} A  |  P_net = {P_lo:.0f} – {P_hi:.0f} W")
    else:
        print("threshold not reached")

    pk = np.argmax(Eff_s)
    print(f"  Peak η_sys :   {Eff_s[pk]:.1f}%  @ I={I[pk]:.1f} A, P_net={Pnet[pk]:.0f} W\n")

    # ── Plotting ──────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 13))
    fig.suptitle(
        "FC Stack LC 52.30 (balticFuelCells GmbH) — System Simulation  v3\n"
        "BOP: Boreasa C65B2-24PR-0B blower · DC-LT 2 pump (3.2 W) · Electronics (5 W)\n"
        "Chamberline–Kim polarization model [Kim 1995] | Parameters calibrated to nameplate",
        fontsize=11, fontweight="bold", y=0.99
    )
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.44, wspace=0.36)

    g_nom = dict(color="grey",   ls=":", lw=1.2, alpha=0.8)
    g_max = dict(color="maroon", ls=":", lw=1.2, alpha=0.8)

    # ── Plot 1: V-I and P-I ───────────────────────────────────────────────────
    ax1  = fig.add_subplot(gs[0, 0])
    ax1r = ax1.twinx()

    ax1.plot(I, Vstack,  color="royalblue",  lw=2.5, label=r"$V_{stack}$")
    ax1.plot(I, Vcell * 10, color="steelblue", lw=1.5, ls="--",
             label=r"$V_{cell} \times 10$")
    ax1r.plot(I, Pgross, color="firebrick",  lw=2,   label=r"$P_{gross}$")
    ax1r.plot(I, Pnet,   color="darkorange", lw=2.5, label=r"$P_{net}$")
    ax1r.plot(I, Pbop,   color="dimgrey",    lw=1.5, ls="--",
              label=r"$P_{BOP}$")

    ax1.axhline(V_min, color="royalblue", ls="--", lw=1,
                label=f"$V_{{min}}$ = {V_min} V")
    ax1.axvline(I_nom, **g_nom, label=f"$I_{{nom}}$ = {I_nom} A")
    ax1.axvline(I_max, **g_max, label=f"$I_{{max}}$ = {I_max} A")
    ax1r.axhline(P_nom_spec, color="firebrick", ls=":", lw=1,
                 label=f"$P_{{nom}}$ = {P_nom_spec:.0f} W")

    ax1.set_xlabel("Stack Current  $I$  [A]")
    ax1.set_ylabel("Voltage [V]", color="royalblue")
    ax1r.set_ylabel("Power [W]",  color="firebrick")
    ax1.set_title("Polarization (V–I) and Power", fontweight="bold")
    ax1.set_ylim(0, 58); ax1r.set_ylim(0, 1250)
    l1, lb1 = ax1.get_legend_handles_labels()
    l2, lb2 = ax1r.get_legend_handles_labels()
    ax1.legend(l1+l2, lb1+lb2, fontsize=7, loc="center left")
    ax1.grid(True, ls="--", alpha=0.45)

    # ── Plot 2: System Efficiency ─────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])

    ax2.plot(I, Eff_g, color="steelblue", lw=2,   ls="--",
             label=r"Gross stack $\eta_{stack}$ (LHV)")
    ax2.plot(I, Eff_s, color="seagreen",  lw=2.5,
             label=r"Net system $\eta_{sys}$ (LHV)")
    ax2.axhline(55, color="darkorange", ls="--", lw=1.8, label="55% threshold")
    ax2.axvline(I_nom, **g_nom, label=f"$I_{{nom}}$ = {I_nom} A")
    ax2.axvline(I_max, **g_max, label=f"$I_{{max}}$ = {I_max} A")

    if above55.any():
        ax2.axvspan(I_lo, I_hi, alpha=0.12, color="seagreen",
                    label=f"$\\eta_{{sys}}$ > 55 %:  {P_lo:.0f}–{P_hi:.0f} W")

    ax2.annotate(
        f"Peak {Eff_s[pk]:.1f}%\n@ {I[pk]:.1f} A / {Pnet[pk]:.0f} W",
        xy=(I[pk], Eff_s[pk]),
        xytext=(I[pk]+2.5, Eff_s[pk]-5),
        fontsize=8, color="seagreen",
        arrowprops=dict(arrowstyle="->", color="seagreen", lw=1.2)
    )

    ax2.set_xlabel("Stack Current  $I$  [A]")
    ax2.set_ylabel("Efficiency [%]")
    ax2.set_title("System Efficiency vs Current (LHV basis)", fontweight="bold")
    ax2.set_ylim(0, 80)
    ax2.legend(fontsize=7, loc="upper right")
    ax2.grid(True, ls="--", alpha=0.45)

    # ── Plot 3: H₂ Flow Rate ──────────────────────────────────────────────────
    ax3  = fig.add_subplot(gs[1, 0])
    ax3r = ax3.twinx()

    ax3.plot(I, H_supp_gs,  color="mediumpurple", lw=2.5,
             label=f"Supply (λ={lam_H2}) [g/s]")
    ax3.plot(I, H_cons_gs,  color="mediumpurple", lw=1.5, ls="--",
             label="Consumed (λ=1) [g/s]")
    ax3r.plot(I, H_supp_slpm, color="darkorange", lw=2.5,
              label=f"Supply (λ={lam_H2}) [SLPM]")
    ax3r.plot(I, H_cons_slpm, color="darkorange", lw=1.5, ls="--",
              label="Consumed (λ=1) [SLPM]")

    ax3r.axhline(19.0, color="darkorange", ls=":", lw=1.5,
                 label="Spec: 19 SLPM @ 37.5 A")
    ax3.axvline(I_nom, **g_nom)
    ax3.axvline(I_max, **g_max)

    ax3.set_xlabel("Stack Current  $I$  [A]")
    ax3.set_ylabel("H₂ Mass Flow [g/s]",    color="mediumpurple")
    ax3r.set_ylabel("H₂ Volume Flow [SLPM]", color="darkorange")
    ax3.set_title("Hydrogen Mass-Flow Rate vs Current", fontweight="bold")
    l3, lb3 = ax3.get_legend_handles_labels()
    l4, lb4 = ax3r.get_legend_handles_labels()
    ax3.legend(l3+l4, lb3+lb4, fontsize=7, loc="upper left")
    ax3.grid(True, ls="--", alpha=0.45)

    # ── Plot 4: BOP Breakdown ─────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])

    ax4.stackplot(I, Pbl, Ppump, Pelec,
                  labels=[f"Boreasa C65B2 blower (var-speed, η={BLOWER_ETA_EFF:.0%})",
                          "DC-LT 2 pump (fixed 3.2 W)",
                          "Electronics / sensors (fixed 5 W)"],
                  colors=["#e07b54", "#5b8cda", "#6abf69"], alpha=0.85)
    ax4.plot(I, Pbop,  "k-",  lw=2.2, label=r"Total $P_{BOP}$")
    ax4.plot(I, Pgross * 0.05, "r--", lw=1.2, label="5 % of $P_{gross}$")

    # Annotate blower working-point on P_BOP axis
    ax4.annotate(
        f"Blower WP ref:\n{BLOWER_P_WP:.0f} W @ 372 lpm\n(not reached in this system)",
        xy=(I_max, Pbl[-1]), xytext=(25, BLOWER_P_WP * 0.35),
        fontsize=7, color="#e07b54",
        arrowprops=dict(arrowstyle="->", color="#e07b54", lw=1)
    )

    ax4.axvline(I_nom, **g_nom, label=f"$I_{{nom}}$ = {I_nom} A")
    ax4.axvline(I_max, **g_max, label=f"$I_{{max}}$ = {I_max} A")
    ax4.set_xlabel("Stack Current  $I$  [A]")
    ax4.set_ylabel("Parasitic Power [W]")
    ax4.set_title("BOP Parasitic Load Breakdown\n(real component specifications)", fontweight="bold")
    ax4.legend(fontsize=7, loc="upper left")
    ax4.grid(True, ls="--", alpha=0.45)

    # ── Save ──────────────────────────────────────────────────────────────────
    out = os.path.join(RESULTS_DIR, "FC_LC52_30_system_curves_v3.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Plot saved → {out}\n")


if __name__ == "__main__":
    main()
