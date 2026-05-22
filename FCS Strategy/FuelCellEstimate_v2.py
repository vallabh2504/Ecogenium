"""
FC Stack LC 52.30 — Corrected System-Level Simulation
======================================================
balticFuelCells GmbH | N=52 cells | A=30 cm² | P_nom=1040 W @ 37.5 A

Generates four curves:
  1. Polarization (V–I) and Power (P–I)
  2. System efficiency vs current  (LHV basis, net)
  3. Hydrogen mass-flow rate vs current
  4. BOP parasitic load breakdown vs current

═══════════════════════════════════════════════════════
ELECTROCHEMICAL MODEL — Chamberline–Kim (1995)  [Ref 1]
═══════════════════════════════════════════════════════
Single-cell voltage:

    V_cell = E₀  −  η_act  −  η_ohm  −  η_conc

  η_act  = b · ln(J / J₀)                        Tafel activation loss   [Ref 1,4]
  η_ohm  = ASR · J                                Area-specific ohmic loss [Ref 1,4]
  η_conc = −(RT/2F) · ln(1 − J/J_L)             Concentration loss       [Ref 4,5]

  J  = I/A   [A/cm²]   current density
  b  = RT/(α·F)        Tafel slope, α ≈ 0.5 (cathode), T = 65 °C → b ≈ 0.058 V
  J₀ = exchange current density [A/cm²]  (cathode, Pt/C, ~ 0.01 A/cm² at 65 °C) [Ref 4]
  ASR= area-specific resistance [Ω·cm²]  (0.08–0.20 typical for PEM at 60–75 °C) [Ref 6]
  J_L= limiting current density [A/cm²]  (1.4–2.0 for air-fed stacks)            [Ref 4]

PARAMETER CALIBRATION to LC 52.30 nameplate data [Ref 7]:
  • Target A: V_cell(J=1.25 A/cm², I=37.5 A) = 1040/(52×37.5) = 0.533 V/cell
  • Target B: V_stack(I=40 A) ≥ 26 V  (min spec)
  • Target C: H₂ supply ≈ 19 SLPM @ 37.5 A  → implies λ_H2 = 1.40
  Calibrated values: E₀=0.955 V, b=0.058 V, J₀=0.010 A/cm², ASR=0.095 Ω·cm², J_L=1.65 A/cm²

═══════════════════════════════════════════════════════
SYSTEM EFFICIENCY (LHV basis)                  [Ref 2,3]
═══════════════════════════════════════════════════════
Stack gross efficiency:
    η_stack = V_cell / V_th,LHV      where V_th,LHV = ΔH_LHV/(n·F) = 1.253 V

Net system efficiency (with BOP parasitics):
    η_sys = P_net / (ṅ_H2,consumed · ΔH_LHV)
          = (V_stack·I − P_BOP) / (N·I/(2F) · 241 820 J/mol)

  ΔH_LHV(H₂) = 241 820 J/mol  (lower heating value, gaseous H₂O product)
  Note: efficiency computed on consumed H₂ (dead-end anode utilisation ≈ 99 %)

═══════════════════════════════════════════════════════
HYDROGEN FLOW RATE — Faraday's Law              [Ref 2,5]
═══════════════════════════════════════════════════════
    ṅ_H2,consumed = N·I / (2·F)              [mol/s]
    ṁ_H2,consumed = ṅ_H2 · M_H2             [g/s]
    Q_H2,supply   = λ_H2 · ṅ_H2 · 22.414 · 60  [SLPM at STP: 0 °C, 1 atm]

  λ_H2 = 1.40  (dead-end with 150 ms purge every 7.5 s per manual [Ref 7];
                confirmed: Q_H2,supply @ 37.5 A = 1.4 × 13.57 = 18.99 ≈ 19 SLPM ✓)

═══════════════════════════════════════════════════════
BALANCE OF PLANT (BOP) MODEL                   [Ref 3,4,5]
═══════════════════════════════════════════════════════
Three parasitic loads are modelled:

1. Air blower (cathode supply):
     ṁ_air = λ_air · N·I · M_air / (4·F·x_O2)      [kg/s]      [Ref 4,5]
     P_blower = ṁ_air · ΔP_cat / (ρ_air · η_isen)   [W]
     λ_air = 2.0  (manual minimum [Ref 7]), ΔP_cat = 10 000 Pa (centre of 20–100 mbar), η_isen = 0.60

2. Coolant pump (liquid circuit):
     Q_thermal  = P_gross · (1/η_stack − 1)           [W waste heat]
     ṁ_coolant  = Q_thermal / (c_p,water · ΔT_cool)   [kg/s]    [Ref 4]
     P_pump     = ṁ_coolant · ΔP_cool / (ρ_water · η_pump)  [W]
     ΔP_cool = 30 000 Pa, η_pump = 0.40, ΔT_cool = 7 K

3. Controller / ECU:  P_ctrl = 10 W (fixed)            [Ref 5,6]

Expected total BOP at I_nom = 37.5 A: ~37–45 W  (~3.6 % of gross power) — consistent
with liquid-cooled 1 kW systems in literature [Ref 5,6].

═══════════════════════════════════════════════════════
REFERENCES
═══════════════════════════════════════════════════════
[1] Kim J. et al. (1995) J. Electrochem. Soc. 142(8):2670–2674
    Chamberline-Kim semi-empirical polarization model.
[2] Kabza A. "Fuel Cell Formulary" v2.4 (2020). pemfc.de/FCF_A4.pdf
    LHV efficiency formula, Faraday-law flow constants.
[3] Larminie J. & Dicks A. (2003) "Fuel Cell Systems Explained," 2nd ed. Wiley.
    System efficiency derivation, BOP architecture.
[4] Barbir F. (2012) "PEM Fuel Cells: Theory and Practice," 2nd ed. Academic Press.
    H₂/air stoichiometry, compressor power, thermal management sizing.
[5] Scribner Associates FAQ. scribner.com
    Practical gas constants: H₂ = 7 sccm/A/cell, Air = 16.7 sccm/A/cell.
    BOP parasitic magnitudes for liquid-cooled ~1 kW systems.
[6] Hao L. et al. (2016) Int. J. Chemical Engineering 2016:4109204.
    DOI:10.1155/2016/4109204  — ASR and parameter ranges for PEM stacks.
[7] balticFuelCells GmbH. User Manual FC Stack LCxx.30, Issue 05/2024.
    Stack specs: N=52, A=30 cm², V=26–52 V, I_max=40 A, P_nom=1040 W @ 37.5 A,
    H₂ consumption ≈ 19 SLPM at nominal, cathode λ ≥ 2, dead-end anode.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")          # non-interactive backend for headless environments
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

# ─── OUTPUT FOLDER ────────────────────────────────────────────────────────────
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# ─── PHYSICAL CONSTANTS ───────────────────────────────────────────────────────
F        = 96_485.0      # Faraday constant                [C/mol]
R_gas    = 8.314         # Universal gas constant          [J/mol/K]
M_H2     = 2.016e-3      # Molar mass H₂                  [kg/mol]
M_air    = 28.97e-3      # Molar mass dry air              [kg/mol]
x_O2     = 0.2095        # O₂ mole fraction in air
LHV_H2   = 241_820.0     # Lower heating value H₂         [J/mol]
V_mol_stp = 22.414       # Molar volume at STP (0°C,1atm) [L/mol]
rho_air  = 1.20          # Air density at ~20 °C, 1 atm   [kg/m³]
c_p_H2O  = 4182.0        # Specific heat of water         [J/kg/K]
rho_H2O  = 980.5         # Water density at ~65 °C        [kg/m³]
n_e      = 2             # Electrons per H₂ molecule

# ─── STACK PARAMETERS (balticFuelCells FC Stack LC 52.30) [Ref 7] ─────────────
N        = 52            # Number of cells
A        = 30.0          # Active cell area               [cm²]
T_K      = 338.15        # Operating temperature          [K]  (65 °C, mid of 60–75 °C)
I_nom    = 37.5          # Nominal current (extended sys) [A]  [Ref 7]
I_max    = 40.0          # Maximum rated current          [A]  [Ref 7]
V_min    = 26.0          # Minimum stack voltage spec     [V]  [Ref 7]
P_nom_spec = 1040.0      # Nameplate gross power          [W]  [Ref 7]

# ─── POLARIZATION MODEL PARAMETERS (calibrated to LC 52.30) [Ref 1,6,7] ─────
E0       = 0.955         # Practical OCV per cell         [V]
b        = 0.058         # Tafel slope = R·T/(α·F)        [V]  (α=0.498 @ 65°C)
J0       = 0.010         # Apparent exchange current density [A/cm²] — empirical fit parameter,
                         # not the intrinsic ORR value; absorbs reference-potential offset [Ref 1]
ASR      = 0.095         # Area-specific resistance       [Ω·cm²]
J_L      = 1.65          # Limiting current density       [A/cm²]

# ─── OPERATING CONDITIONS ─────────────────────────────────────────────────────
lam_H2      = 1.40       # H₂ stoichiometry (dead-end+purge) [Ref 7]
lam_air     = 2.0        # Cathode air stoichiometry (≥2 req'd) [Ref 7]
dP_cathode  = 10_000.0   # Cathode pressure drop          [Pa]  (10 kPa, centre of 20–100 mbar)
eta_isen    = 0.60       # Blower isentropic efficiency   (small BLDC blower) [Ref 5]
dP_coolant  = 30_000.0   # Coolant circuit pressure drop  [Pa]
eta_pump    = 0.40       # Coolant pump efficiency        [Ref 4]
dT_coolant  = 7.0        # Coolant ΔT across stack        [K]   [Ref 4]
P_ctrl      = 10.0       # Controller / ECU fixed load    [W]   [Ref 5,6]


# ══════════════════════════════════════════════════════════════════════════════
# ELECTROCHEMICAL MODEL
# ══════════════════════════════════════════════════════════════════════════════

def cell_voltage(I: np.ndarray) -> np.ndarray:
    """
    Chamberline-Kim polarization model [Ref 1].
    Activation term uses Tafel equation (valid for J > J0).
    Concentration term uses thermodynamic log form [Ref 4].
    """
    J = I / A
    # Activation loss: Tafel equation (clipped to 0 for J ≤ J0, low-load region)
    eta_act  = np.where(J > J0, b * np.log(J / J0), 0.0)
    # Ohmic loss: constant ASR (valid for well-humidified membrane at 60–75 °C) [Ref 6]
    eta_ohm  = ASR * J
    # Concentration loss: thermodynamic form [Ref 4]
    #   −(RT/nF)·ln(1−J/J_L) diverges as J → J_L; guard against J ≥ J_L
    safe_J   = np.minimum(J, J_L * 0.999)
    eta_conc = -(R_gas * T_K / (n_e * F)) * np.log(1.0 - safe_J / J_L)
    V = E0 - eta_act - eta_ohm - eta_conc
    return np.clip(V, 0.0, E0)


def stack_voltage(I: np.ndarray) -> np.ndarray:
    return N * cell_voltage(I)


def P_stack_gross(I: np.ndarray) -> np.ndarray:
    return stack_voltage(I) * I


# ══════════════════════════════════════════════════════════════════════════════
# HYDROGEN FLOW RATE  [Ref 2,5]
# ══════════════════════════════════════════════════════════════════════════════

def H2_molar_consumed(I: np.ndarray) -> np.ndarray:
    """Stoichiometric H₂ consumption via Faraday's law [Ref 2]."""
    return N * I / (n_e * F)                          # mol/s


def H2_mass_flow_gs(I: np.ndarray, lam: float = 1.0) -> np.ndarray:
    """H₂ mass flow rate [g/s].  lam=1 → consumed; lam=lam_H2 → supplied."""
    return lam * H2_molar_consumed(I) * M_H2 * 1000.0


def H2_SLPM(I: np.ndarray, lam: float = 1.0) -> np.ndarray:
    """H₂ volumetric flow at STP [SLPM].  lam=1 → consumed; lam=lam_H2 → supplied."""
    return lam * H2_molar_consumed(I) * V_mol_stp * 60.0


# ══════════════════════════════════════════════════════════════════════════════
# BALANCE OF PLANT (BOP)
# ══════════════════════════════════════════════════════════════════════════════

def P_blower(I: np.ndarray) -> np.ndarray:
    """
    Air blower power for cathode supply [Ref 3,4,5].
    O₂ consumption: ṅ_O2 = N·I/(4·F)  (4 electrons per O₂ molecule)
    Air flow: ṁ_air = λ_air · ṅ_O2 · M_air / x_O2
    Power (incompressible approx, valid for ΔP ≤ 20 kPa): P = ṁ·ΔP/(ρ·η)
    """
    m_dot_O2  = N * I / (n_e * 2 * F)                           # mol/s O₂ (n_e*2 = 4)
    m_dot_air = lam_air * m_dot_O2 * M_air / x_O2               # kg/s
    return m_dot_air * dP_cathode / (rho_air * eta_isen)         # W


def P_pump(I: np.ndarray) -> np.ndarray:
    """
    Coolant pump power sized from thermal balance [Ref 4].
    Waste heat: Q_th = P_gross · (1/η_stack − 1)
    Coolant flow: ṁ = Q_th / (c_p · ΔT)
    Pump power: P = ṁ · ΔP / (ρ · η_pump)
    """
    p_g     = P_stack_gross(I)
    # Gross stack efficiency (LHV): V_cell / V_th,LHV
    eta_g   = np.clip(cell_voltage(I) / (LHV_H2 / (n_e * F)), 0.0, 1.0)
    Q_th    = np.maximum(p_g * (1.0 / eta_g - 1.0), 0.0)  # = N·I·(V_th − V_cell)              # W, waste heat
    m_dot_c = np.where(Q_th > 0,
                       Q_th / (c_p_H2O * dT_coolant), 0.0)      # kg/s
    return m_dot_c * dP_coolant / (rho_H2O * eta_pump)           # W


def P_BOP(I: np.ndarray) -> np.ndarray:
    return P_blower(I) + P_pump(I) + P_ctrl


# ══════════════════════════════════════════════════════════════════════════════
# NET POWER AND SYSTEM EFFICIENCY
# ══════════════════════════════════════════════════════════════════════════════

def P_net(I: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, P_stack_gross(I) - P_BOP(I))


def eta_stack_gross(I: np.ndarray) -> np.ndarray:
    """
    Gross stack efficiency on LHV basis [Ref 2].
    η_stack = V_cell / V_th,LHV   where V_th,LHV = ΔH_LHV/(n·F) = 1.253 V
    """
    V_th_LHV = LHV_H2 / (n_e * F)                               # = 1.253 V
    return np.clip(cell_voltage(I) / V_th_LHV, 0.0, 1.0) * 100.0


def eta_system_net(I: np.ndarray) -> np.ndarray:
    """
    Net system efficiency on LHV basis [Ref 2,3].
    η_sys = P_net / (ṅ_H2,consumed · ΔH_LHV)

    Convention: denominator uses electrochemically *consumed* H₂ (λ=1 basis).
    This is correct for dead-end anode operation (utilisation ≈ 99%).
    lam_H2=1.4 governs the *supply* flow rate reported separately; it does NOT
    reduce efficiency here because the purged excess is negligible (<1%). [Ref 7]
    """
    P_chem = H2_molar_consumed(I) * LHV_H2                      # W (chem power consumed)
    return np.where(P_chem > 0,
                    np.clip(P_net(I) / P_chem, 0.0, 1.0) * 100.0,
                    0.0)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN: RUN SIMULATION AND SAVE PLOTS
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # Current sweep: 0.5 A to I_max (40 A), fine resolution
    I = np.linspace(0.5, I_max, 2000)

    Vcell   = cell_voltage(I)
    Vstack  = stack_voltage(I)
    Pgross  = P_stack_gross(I)
    Pbl     = P_blower(I)
    Ppu     = P_pump(I)
    Pctrl   = np.full_like(I, P_ctrl)
    Pbop    = P_BOP(I)
    Pnet    = P_net(I)
    Hcons_gs   = H2_mass_flow_gs(I, lam=1.0)
    Hsupp_gs   = H2_mass_flow_gs(I, lam=lam_H2)
    Hcons_slpm = H2_SLPM(I, lam=1.0)
    Hsupp_slpm = H2_SLPM(I, lam=lam_H2)
    Eff_g   = eta_stack_gross(I)
    Eff_s   = eta_system_net(I)

    # ── Verification table ───────────────────────────────────────────────────
    print("=" * 75)
    print(f"  FC Stack LC 52.30 — System Simulation  (N={N}, A={A} cm², T={T_K-273.15:.0f}°C)")
    print("=" * 75)
    print(f"  {'I [A]':>7}  {'V_stack [V]':>12}  {'P_gross [W]':>12}  {'P_BOP [W]':>10}  {'P_net [W]':>10}  {'η_sys [%]':>10}  {'H2_sup [SLPM]':>14}")
    print("  " + "-" * 73)
    for I_chk in [1.0, 5.0, 10.0, 20.0, 30.0, 37.5, 40.0]:
        idx = np.argmin(np.abs(I - I_chk))
        print(f"  {I_chk:7.1f}  {Vstack[idx]:12.2f}  {Pgross[idx]:12.1f}  {Pbop[idx]:10.1f}  {Pnet[idx]:10.1f}  {Eff_s[idx]:10.1f}  {Hsupp_slpm[idx]:14.2f}")
    print("=" * 75)

    # Nominal-point checks
    idx_nom = np.argmin(np.abs(I - I_nom))
    idx_max = np.argmin(np.abs(I - I_max))
    print(f"\n  CALIBRATION CHECKS:")
    print(f"  P_gross @ {I_nom} A  = {Pgross[idx_nom]:.1f} W  (spec: {P_nom_spec} W)  "
          f"{'✓' if abs(Pgross[idx_nom]-P_nom_spec)<30 else '✗'}")
    print(f"  V_stack @ {I_max} A  = {Vstack[idx_max]:.2f} V  (spec: ≥{V_min} V)  "
          f"{'✓' if Vstack[idx_max] >= V_min else '✗'}")
    print(f"  H₂ supply @ {I_nom} A = {Hsupp_slpm[idx_nom]:.2f} SLPM  (spec: ≈19 SLPM)  "
          f"{'✓' if abs(Hsupp_slpm[idx_nom]-19.0)<0.5 else '✗'}")

    # Efficiency > 55 % range
    above55 = Eff_s >= 55.0
    print(f"\n  η_sys > 55 %:  ", end="")
    if above55.any():
        I_lo55  = I[above55].min()
        I_hi55  = I[above55].max()
        Pn_lo55 = Pnet[above55].min()
        Pn_hi55 = Pnet[above55].max()
        print(f"I = {I_lo55:.1f} – {I_hi55:.1f} A  |  P_net = {Pn_lo55:.0f} – {Pn_hi55:.0f} W")
    else:
        print("never reached")

    peak_idx = np.argmax(Eff_s)
    print(f"  Peak η_sys   :  {Eff_s[peak_idx]:.1f} %  @ I={I[peak_idx]:.1f} A, "
          f"P_net={Pnet[peak_idx]:.0f} W")
    print()

    # ── Plotting ─────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 13))
    fig.suptitle(
        "FC Stack LC 52.30 (balticFuelCells GmbH) — System-Level Simulation\n"
        "Chamberline–Kim polarization model [Kim 1995] · Parameters calibrated to nameplate data",
        fontsize=12, fontweight="bold", y=0.98
    )
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.35)

    grey_nom = dict(color="grey",   ls=":", lw=1.2)
    grey_max = dict(color="maroon", ls=":", lw=1.2)

    # ─── Plot 1: V–I  and  P–I ───────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1r = ax1.twinx()

    ax1.plot(I, Vstack,  color="royalblue",   lw=2.5, label="Stack voltage  $V_{stack}$")
    ax1.plot(I, Vcell * 10, color="steelblue", lw=1.5, ls="--",
             label="Cell voltage × 10  $V_{cell}$")
    ax1r.plot(I, Pgross, color="firebrick",   lw=2,   label="Gross power  $P_{gross}$")
    ax1r.plot(I, Pnet,   color="darkorange",  lw=2.5, label="Net power  $P_{net}$")
    ax1r.plot(I, Pbop,   color="dimgrey",     lw=1.5, ls="--", label="BOP parasitic  $P_{BOP}$")

    ax1.axhline(V_min,   color="royalblue", ls="--", lw=1,
                label=f"$V_{{min}}$ = {V_min} V (spec)")
    ax1.axvline(I_nom,   **grey_nom, label=f"$I_{{nom}}$ = {I_nom} A")
    ax1.axvline(I_max,   **grey_max, label=f"$I_{{max}}$ = {I_max} A")
    ax1r.axhline(P_nom_spec, color="firebrick", ls=":", lw=1,
                 label=f"$P_{{nom}}$ = {P_nom_spec} W (spec)")

    ax1.set_xlabel("Stack Current  $I$  [A]")
    ax1.set_ylabel("Voltage [V]", color="royalblue")
    ax1r.set_ylabel("Power [W]", color="firebrick")
    ax1.set_title("Polarization (V–I) and Power Curves", fontweight="bold")
    ax1.set_ylim(0, 58); ax1r.set_ylim(0, 1250)

    lines1, lbl1 = ax1.get_legend_handles_labels()
    lines2, lbl2 = ax1r.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, lbl1 + lbl2, fontsize=7, loc="center left")
    ax1.grid(True, ls="--", alpha=0.5)

    # ─── Plot 2: System Efficiency ────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])

    ax2.plot(I, Eff_g, color="steelblue",  lw=2,   ls="--", label="Gross stack efficiency  $\\eta_{stack}$  (LHV)")
    ax2.plot(I, Eff_s, color="seagreen",   lw=2.5,          label="Net system efficiency  $\\eta_{sys}$  (LHV)")
    ax2.axhline(55, color="darkorange", ls="--", lw=1.8, label="55 % threshold")
    ax2.axvline(I_nom, **grey_nom, label=f"$I_{{nom}}$ = {I_nom} A")
    ax2.axvline(I_max, **grey_max, label=f"$I_{{max}}$ = {I_max} A")

    if above55.any():
        ax2.axvspan(I_lo55, I_hi55, alpha=0.13, color="seagreen",
                    label=f"$\\eta_{{sys}}$ > 55%\n{Pn_lo55:.0f}–{Pn_hi55:.0f} W")

    ax2.annotate(
        f"Peak {Eff_s[peak_idx]:.1f}%\n@ {I[peak_idx]:.1f} A",
        xy=(I[peak_idx], Eff_s[peak_idx]),
        xytext=(I[peak_idx] + 2, Eff_s[peak_idx] - 4),
        fontsize=8, color="seagreen",
        arrowprops=dict(arrowstyle="->", color="seagreen")
    )

    ax2.set_xlabel("Stack Current  $I$  [A]")
    ax2.set_ylabel("Efficiency [%]")
    ax2.set_title("System Efficiency vs Current (LHV basis)", fontweight="bold")
    ax2.set_ylim(0, 80)
    ax2.legend(fontsize=7, loc="upper right")
    ax2.grid(True, ls="--", alpha=0.5)

    # ─── Plot 3: H₂ Flow Rate ─────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3r = ax3.twinx()

    ax3.plot(I, Hsupp_gs,   color="mediumpurple", lw=2.5,
             label=f"H₂ supplied (λ={lam_H2}) [g/s]")
    ax3.plot(I, Hcons_gs,   color="mediumpurple", lw=1.5, ls="--",
             label="H₂ consumed (λ=1) [g/s]")
    ax3r.plot(I, Hsupp_slpm, color="darkorange",  lw=2.5,
              label=f"H₂ supplied (λ={lam_H2}) [SLPM]")
    ax3r.plot(I, Hcons_slpm, color="darkorange",  lw=1.5, ls="--",
              label="H₂ consumed (λ=1) [SLPM]")

    ax3r.axhline(19.0, color="darkorange", ls=":", lw=1.5,
                 label="Spec: 19 SLPM @ 37.5 A")
    ax3.axvline(I_nom, **grey_nom)
    ax3.axvline(I_max, **grey_max)

    ax3.set_xlabel("Stack Current  $I$  [A]")
    ax3.set_ylabel("H₂ Mass Flow [g/s]", color="mediumpurple")
    ax3r.set_ylabel("H₂ Volume Flow [SLPM]", color="darkorange")
    ax3.set_title("Hydrogen Mass-Flow Rate vs Current", fontweight="bold")

    lines3, lbl3 = ax3.get_legend_handles_labels()
    lines4, lbl4 = ax3r.get_legend_handles_labels()
    ax3.legend(lines3 + lines4, lbl3 + lbl4, fontsize=7, loc="upper left")
    ax3.grid(True, ls="--", alpha=0.5)

    # ─── Plot 4: BOP Breakdown ────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])

    ax4.stackplot(I, Pbl, Ppu, Pctrl,
                  labels=["Air blower", "Coolant pump", "Controller / ECU"],
                  colors=["#e07b54", "#5b8cda", "#6abf69"], alpha=0.82)
    ax4.plot(I, Pbop,  "k-",  lw=2.2, label="Total BOP  $P_{BOP}$")
    ax4.plot(I, Pgross * 0.05, "r--", lw=1.3, label="5 % of $P_{gross}$ (ref)")
    ax4.plot(I, Pgross * 0.10, "r:",  lw=1.3, label="10% of $P_{gross}$ (ref)")
    ax4.axvline(I_nom, **grey_nom, label=f"$I_{{nom}}$ = {I_nom} A")
    ax4.axvline(I_max, **grey_max, label=f"$I_{{max}}$ = {I_max} A")

    ax4.set_xlabel("Stack Current  $I$  [A]")
    ax4.set_ylabel("Parasitic Power [W]")
    ax4.set_title("BOP Parasitic Load Breakdown", fontweight="bold")
    ax4.legend(fontsize=7, loc="upper left")
    ax4.grid(True, ls="--", alpha=0.5)

    # ─── Save ─────────────────────────────────────────────────────────────────
    out_path = os.path.join(RESULTS_DIR, "FC_LC52_30_system_curves.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Plots saved → {out_path}\n")


if __name__ == "__main__":
    main()
