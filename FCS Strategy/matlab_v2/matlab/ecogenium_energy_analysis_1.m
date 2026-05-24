% =========================================================================
% ECOGENIUM ENERGY CONSUMPTION & POWER DEMAND ANALYSIS
% Vehicle Dynamics-Based Power Calculation
% Version: 1.2 (Added Track Gradient / Incline Power)
% =========================================================================
% Based on:
%   - Vehicle mass: 180 kg
%   - Wheel radius: 0.295 m
%   - Physics-based power demand modeling
% =========================================================================
clear all; close all; clc;

%% ========================================================================
% VEHICLE PARAMETERS
% =========================================================================
fprintf('\n=== VEHICLE PARAMETERS ===\n');

% Given parameters
vehicle_mass = 160;  % kg
wheel_radius = 0.295;  % meters
gravity = 9.81;  % m/s^2

% ASSUMED PARAMETERS (typical for Shell Eco-marathon Urban Concept)
drag_coefficient = 0.15;  % Cd (very aerodynamic for Eco-marathon)
frontal_area = 0.8;  % m^2 (typical for urban concept)
rolling_resistance_coef = 0.006;  % Crr (low rolling resistance tires)
air_density = 1.225;  % kg/m^3 (sea level)
drivetrain_efficiency = 0.85;  % 85% efficiency (motor + transmission)

% Motor specifications (Hydraix twin 380W motors)
motor_power_rated = 760;  % Watts (2 x 380W)
motor_efficiency = 0.96;  % 90% typical for brushless DC motors

fprintf('Vehicle Mass: %.0f kg\n', vehicle_mass);
fprintf('Wheel Radius: %.3f m\n', wheel_radius);
fprintf('Drag Coefficient (Cd): %.2f\n', drag_coefficient);
fprintf('Frontal Area: %.2f m^2\n', frontal_area);
fprintf('Rolling Resistance (Crr): %.4f\n', rolling_resistance_coef);
fprintf('Drivetrain Efficiency: %.0f%%\n', drivetrain_efficiency*100);
fprintf('Motor Rated Power: %.0f W\n', motor_power_rated);

%% ========================================================================
% LOAD CANONICAL DRIVE CYCLE
% =========================================================================
fprintf('\n=== LOADING DRIVE CYCLE ===\n');

% Using the new file that includes the incline data
data = readtable('canonical_with_incline.csv');

% --- FILTER TO A SINGLE LAP ---
single_lap_duration = data.elapsed_sec(end) / 11;
data = data(data.elapsed_sec <= single_lap_duration, :);
% ---------------------------------------

time = data.elapsed_sec;
speed_kmh = data.Speed_kmh;
rpm = data.MotorRPM;
incline_deg = data.inc_angle_deg; % NEW: Extract Incline Angle

% Convert speed to m/s
speed_ms = speed_kmh / 3.6;

% Calculate acceleration
dt = diff(time);
dt = [dt(1); dt];  % Pad to match length
acceleration = [0; diff(speed_ms)] ./ dt;
acceleration = movmean(acceleration, 10);  % Smooth acceleration

fprintf('Drive cycle loaded: %.1f minutes\n', max(time)/60);
fprintf('Max speed: %.1f km/h (%.1f m/s)\n', max(speed_kmh), max(speed_ms));

%% ========================================================================
% POWER DEMAND CALCULATION
% =========================================================================
fprintf('\n=== CALCULATING POWER DEMAND ===\n');

% Initialize power components
P_rolling = zeros(size(speed_ms));
P_aero = zeros(size(speed_ms));
P_accel = zeros(size(speed_ms));
P_gradient = zeros(size(speed_ms)); % NEW: Initialize Gradient Power array
P_total_wheel = zeros(size(speed_ms));
P_motor_output = zeros(size(speed_ms));
P_electrical_input = zeros(size(speed_ms));

for i = 1:length(speed_ms)
    v = speed_ms(i);
    a = acceleration(i);
    
    % 1. Rolling Resistance Power
    P_rolling(i) = rolling_resistance_coef * vehicle_mass * gravity * v;
    
    % 2. Aerodynamic Drag Power
    P_aero(i) = 0.5 * drag_coefficient * frontal_area * air_density * v^3;
    
    % 3. Acceleration Power (kinetic energy change)
    P_accel(i) = vehicle_mass * a * v;
    
    % 4. Gradient Power (hills) - NEW
    % P_grad = m * g * sin(theta) * v
    P_gradient(i) = vehicle_mass * gravity * sin(deg2rad(incline_deg(i))) * v;
    
    % 5. Total power at wheels (NEW: Now includes P_gradient)
    P_total_wheel(i) = P_rolling(i) + P_aero(i) + P_accel(i) + P_gradient(i);
    
    % 6. Motor output power (accounting for drivetrain losses)
    if P_total_wheel(i) > 0
        P_motor_output(i) = P_total_wheel(i) / drivetrain_efficiency;
    else
        % During coasting/braking/downhill, no motor power needed
        P_motor_output(i) = 0;
    end
    
    % 7. Electrical input power (accounting for motor efficiency)
    if P_motor_output(i) > 0
        P_electrical_input(i) = P_motor_output(i) / motor_efficiency;
    else
        P_electrical_input(i) = 0;
    end
end

% Clip negative powers to zero (coasting)
P_electrical_input(P_electrical_input < 0) = 0;

% CRITICAL FIX: Cap unrealistic power values (handle outliers from speed spikes)
max_realistic_power = motor_power_rated * 3;  % Allow 3x rated for brief bursts
P_electrical_input(P_electrical_input > max_realistic_power) = max_realistic_power;
P_rolling(P_rolling > max_realistic_power) = max_realistic_power;
P_aero(P_aero > max_realistic_power) = max_realistic_power;
P_accel(P_accel > max_realistic_power) = max_realistic_power;
P_gradient(P_gradient > max_realistic_power) = max_realistic_power; % NEW

% Remove any NaN or Inf values
P_electrical_input(~isfinite(P_electrical_input)) = 0;
P_rolling(~isfinite(P_rolling)) = 0;
P_aero(~isfinite(P_aero)) = 0;
P_accel(~isfinite(P_accel)) = 0;
P_gradient(~isfinite(P_gradient)) = 0; % NEW

fprintf('Peak power demand: %.0f W\n', max(P_electrical_input));
fprintf('Average power (when moving): %.0f W\n', mean(P_electrical_input(speed_ms > 0)));

%% ========================================================================
% ENERGY CONSUMPTION CALCULATION
% =========================================================================
fprintf('\n=== CALCULATING ENERGY CONSUMPTION ===\n');

% Energy = integral of power over time
energy_increments = P_electrical_input .* dt;  % Watt-seconds (Joules)
cumulative_energy_J = cumsum(energy_increments);
cumulative_energy_Wh = cumulative_energy_J / 3600;  % Convert to Watt-hours
cumulative_energy_kWh = cumulative_energy_Wh / 1000;  % Convert to kWh

total_energy_Wh = cumulative_energy_Wh(end);
total_energy_kWh = cumulative_energy_kWh(end);

fprintf('Total energy consumed: %.2f Wh (%.4f kWh)\n', total_energy_Wh, total_energy_kWh);
fprintf('Energy per km: %.2f Wh/km\n', total_energy_Wh / max(data.Distance_km));

% Calculate energy breakdown
energy_rolling = sum(P_rolling .* dt) / 3600;  % Wh
energy_aero = sum(P_aero .* dt) / 3600;  % Wh
energy_accel = sum(P_accel(P_accel > 0) .* dt(P_accel > 0)) / 3600;  % Wh (only positive)
energy_grad = sum(P_gradient(P_gradient > 0) .* dt(P_gradient > 0)) / 3600; % NEW: Wh (only uphill)

fprintf('\nEnergy Breakdown:\n');
fprintf('  Rolling Resistance: %.1f Wh (%.1f%%)\n', energy_rolling, energy_rolling/total_energy_Wh*100);
fprintf('  Aerodynamic Drag: %.1f Wh (%.1f%%)\n', energy_aero, energy_aero/total_energy_Wh*100);
fprintf('  Acceleration: %.1f Wh (%.1f%%)\n', energy_accel, energy_accel/total_energy_Wh*100);
fprintf('  Gradient (Uphill): %.1f Wh (%.1f%%)\n', energy_grad, energy_grad/total_energy_Wh*100); % NEW
fprintf('  Losses (drivetrain + motor): %.1f Wh (%.1f%%)\n', ...
    total_energy_Wh - (energy_rolling + energy_aero + energy_accel + energy_grad), ...
    (total_energy_Wh - (energy_rolling + energy_aero + energy_accel + energy_grad))/total_energy_Wh*100);

%% ========================================================================
% VISUALIZATION: POWER DEMAND CURVE
% =========================================================================
fprintf('\n=== GENERATING POWER DEMAND VISUALIZATIONS ===\n');

figure('Position', [100, 100, 1600, 1000], 'Name', 'Energy Analysis', 'Color', 'white');

% Subplot 1: Total Power Demand vs Time
subplot(3,2,1);
plot(time/60, P_electrical_input, 'k-', 'LineWidth', 1.5);
hold on;
yline(motor_power_rated, '--r', 'LineWidth', 2, 'DisplayName', 'Motor Rated Power');
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Power (W)', 'FontSize', 12);
title('Total Electrical Power Demand', 'FontSize', 14, 'FontWeight', 'bold');
legend('Power Demand', 'Motor Rating', 'Location', 'best');
grid on;
ylim([0 max([max(P_electrical_input), motor_power_rated])*1.1]);
set(gca, 'Color', 'white');

% Subplot 2: Power Components Breakdown
subplot(3,2,2);
hold on;
% NEW: Added max(P_gradient, 0) to stack only the positive uphill forces
area(time/60, [P_rolling, P_aero, max(P_accel, 0), max(P_gradient, 0)], 'LineWidth', 0.5);
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Power (W)', 'FontSize', 12);
title('Power Component Breakdown', 'FontSize', 14, 'FontWeight', 'bold');
legend('Rolling Resistance', 'Aero Drag', 'Acceleration', 'Gradient', 'Location', 'best');
grid on;
colormap([0.8 0.4 0; 0 0.4 0.8; 0.8 0 0; 0.2 0.8 0.2]); % Added green for Gradient
set(gca, 'Color', 'white');

% Subplot 3: Cumulative Energy Consumption
subplot(3,2,3);
plot(time/60, cumulative_energy_Wh, 'b-', 'LineWidth', 2);
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Energy (Wh)', 'FontSize', 12);
title('Cumulative Energy Consumption', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'Color', 'white');

% Subplot 4: Power vs Speed 
subplot(3,2,4);
scatter(speed_kmh, P_electrical_input, 10, time, 'filled');
xlabel('Speed (km/h)', 'FontSize', 12);
ylabel('Power (W)', 'FontSize', 12);
title('Power Demand vs Speed', 'FontSize', 14, 'FontWeight', 'bold');
cb = colorbar;
cb.Label.String = 'Time (s)';  
grid on;
set(gca, 'Color', 'white');

% Subplot 5: Energy per distance segment
subplot(3,2,5);
window_size = 50;
energy_per_km_local = zeros(size(P_electrical_input));
for i = 1:length(energy_per_km_local)
    if speed_ms(i) > 0.1  % Only when moving
        energy_per_km_local(i) = P_electrical_input(i) / (speed_ms(i) * 3600);
    else
        energy_per_km_local(i) = 0;
    end
end
energy_per_km_local = movmean(energy_per_km_local, window_size);
energy_per_km_local(~isfinite(energy_per_km_local)) = 0;
plot(time/60, energy_per_km_local, 'g-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Energy Intensity (Wh/km)', 'FontSize', 12);
title('Energy Efficiency Profile', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'Color', 'white');

% Subplot 6: Energy breakdown pie chart
subplot(3,2,6);
% NEW: Added energy_grad to the pie chart
energy_breakdown = [energy_rolling, energy_aero, energy_accel, energy_grad, ...
    total_energy_Wh - (energy_rolling + energy_aero + energy_accel + energy_grad)];
labels = {'Rolling', 'Aero', 'Accel', 'Gradient', 'System Losses'};
pie(energy_breakdown, labels);
title('Energy Distribution', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'Color', 'white');

fprintf('Figure created: Power demand and energy analysis\n');

%% ========================================================================
% PERFORMANCE SUMMARY
% =========================================================================
figure('Position', [150, 150, 1400, 600], 'Name', 'Performance Summary', 'Color', 'white');

% Create summary table
subplot(1,2,1);
axis off;
summary_text = {
    '\bf VEHICLE PARAMETERS';
    '================================';
    sprintf('Mass: %.0f kg', vehicle_mass);
    sprintf('Wheel Radius: %.3f m', wheel_radius);
    sprintf('Drag Coef (Cd): %.2f', drag_coefficient);
    sprintf('Frontal Area: %.2f m^2', frontal_area);
    sprintf('Rolling Res (Crr): %.4f', rolling_resistance_coef);
    ' ';
    '\bf PERFORMANCE METRICS';
    '================================';
    sprintf('Total Distance: %.2f km', max(data.Distance_km));
    sprintf('Total Duration: %.1f min', max(time)/60);
    sprintf('Avg Speed: %.1f km/h', mean(speed_kmh(speed_ms > 0)));
    sprintf('Max Speed: %.1f km/h', max(speed_kmh));
    ' ';
    '\bf ENERGY METRICS';
    '================================';
    sprintf('Total Energy: %.2f Wh', total_energy_Wh);
    sprintf('Energy per km: %.2f Wh/km', total_energy_Wh / max(data.Distance_km));
    sprintf('Peak Power: %.0f W', max(P_electrical_input));
    sprintf('Avg Power: %.0f W', mean(P_electrical_input(speed_ms > 0)));
};
text(0.1, 0.95, summary_text, 'VerticalAlignment', 'top', 'FontSize', 11, 'Interpreter', 'tex');

% Power histogram
subplot(1,2,2);
histogram(P_electrical_input(P_electrical_input > 0), 50, 'FaceColor', [0 0.4 0.8]);
xlabel('Power (W)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
title('Power Demand Distribution', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
hold on;
xline(mean(P_electrical_input(P_electrical_input > 0)), '--r', 'LineWidth', 2);
text(mean(P_electrical_input(P_electrical_input > 0)), max(ylim)*0.9, ...
    sprintf('Mean: %.0f W', mean(P_electrical_input(P_electrical_input > 0))), ...
    'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
set(gca, 'Color', 'white');

fprintf('\n=== ENERGY ANALYSIS COMPLETE ===\n');

%% ========================================================================
% EXPORT RESULTS
% =========================================================================
% NEW: Added P_gradient to the exported results
results_table = table(time, speed_kmh, speed_ms, rpm, incline_deg, ...
    P_rolling, P_aero, P_accel, P_gradient, P_total_wheel, P_motor_output, P_electrical_input, ...
    cumulative_energy_Wh, ...
    'VariableNames', {'Time_sec', 'Speed_kmh', 'Speed_ms', 'MotorRPM', 'Incline_deg', ...
    'Power_Rolling_W', 'Power_Aero_W', 'Power_Accel_W', 'Power_Gradient_W', 'Power_Wheel_W', ...
    'Power_Motor_W', 'Power_Electrical_W', 'CumulativeEnergy_Wh'});
writetable(results_table, 'energy_analysis_results.csv');
fprintf('Results exported to: energy_analysis_results.csv\n\n');