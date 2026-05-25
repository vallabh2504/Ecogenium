import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def generate_system_efficiency_curve(I_array, N=52, A=30.0, E_ocv=0.96, a=0.235, b=0.094, R_int=0.18, j_L=1.33):
    """
    Calculates the Net System Efficiency of a Fuel Cell, accounting for 
    thermodynamic losses and Balance of Plant (BOP) parasitic loads.
    """
    # Physical Constants
    R_const = 8.314       # Universal gas constant (J/(mol K))
    T = 333.15            # Operating temperature in Kelvin (60°C)
    F = 96485             # Faraday's constant (C/mol)
    LHV_voltage = 1.25    # Lower Heating Value constant for Hydrogen
    
    j_array = I_array / A # Current density (A/cm^2)

    # 1. Stack Thermodynamics (The Chemistry)
    eta_act = np.maximum(0, a + b * np.log(j_array))
    eta_ohmic = j_array * R_int
    eta_conc = - (R_const * T) / (2 * F) * np.log(1 - (j_array / j_L))

    V_cell = E_ocv - eta_act - eta_ohmic - eta_conc
    V_stack = V_cell * N
    
    # Total Gross Power produced by the chemical reaction
    P_stack_gross = V_stack * I_array
    
    # Total Chemical Power of the Hydrogen consumed
    P_hydrogen_in = N * I_array * LHV_voltage

    # 2. Balance of Plant (The Parasitic Loads)
    # P_bop = Fixed electronics load + Aerodynamic load of the air compressor
    P_idle = 15.0  # Watts consumed by controllers/sensors at idle
    # Compressor power scales non-linearly with current demand
    P_compressor = 0.4 * (I_array ** 1.5) 
    
    P_bop = P_idle + P_compressor

    # 3. Net System Output (The Reality)
    P_net_output = P_stack_gross - P_bop
    
    # System Efficiency = Net Usable Power / Hydrogen Energy Consumed
    # np.maximum(0, ...) clips negative efficiencies (where BOP consumes more than stack makes)
    System_Efficiency = np.maximum(0, (P_net_output / P_hydrogen_in) * 100)
    
    return P_net_output, System_Efficiency, P_stack_gross, P_bop

def main():
    # Generate an array of current from 0.1A to 35A
    I_model = np.linspace(0.1, 35, 500)
    
    # Calculate System Performance
    P_net, Sys_Eff, P_gross, P_bop = generate_system_efficiency_curve(I_model)
    
    # Filter out the points where Net Power is negative (system cannot sustain itself)
    valid_indices = P_net >= 0
    P_net_valid = P_net[valid_indices]
    Sys_Eff_valid = Sys_Eff[valid_indices]

    # Find the Maximum Efficiency Peak
    max_eff_index = np.argmax(Sys_Eff_valid)
    peak_power = P_net_valid[max_eff_index]
    peak_eff = Sys_Eff_valid[max_eff_index]

    # --- Plotting ---
    fig, axs = plt.subplots(2, 1, figsize=(9, 10))
    fig.suptitle('Fuel Cell Net System Efficiency (BOP Included)', fontsize=14, fontweight='bold')

    # Plot 1: Power Breakdown (Gross vs. Parasitic)
    axs[0].plot(I_model, P_gross, color='blue', linewidth=2, label='Gross Stack Power')
    axs[0].plot(I_model, P_bop, color='red', linewidth=2, linestyle='--', label='Parasitic Load (BOP)')
    axs[0].fill_between(I_model, P_bop, 0, color='red', alpha=0.1)
    axs[0].set_ylabel('Power (W)')
    axs[0].set_xlabel('Current Demand (A)')
    axs[0].set_title('Gross Power vs. Parasitic Loads')
    axs[0].grid(True, linestyle='--', alpha=0.7)
    axs[0].legend()

    # Plot 2: The Target Net System Efficiency Curve (Fig 3 Equivalent)
    axs[1].plot(P_net_valid, Sys_Eff_valid, color='green', linewidth=3, label='Net System Efficiency')
    axs[1].plot(peak_power, peak_eff, marker='*', color='gold', markersize=15, 
                markeredgecolor='black', label=f'Peak: {peak_eff:.1f}% @ {peak_power:.0f}W')
    
    axs[1].set_xlabel('Net Output Power (W)')
    axs[1].set_ylabel('System Efficiency (%)')
    axs[1].set_title('Net System Efficiency vs. Net Output Power')
    axs[1].grid(True, linestyle='--', alpha=0.7)
    axs[1].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()