% =========================================================================
% ECOGENIUM ENERGY ANALYSIS MODULE
% Shell Eco-marathon - Hydraix I Power and Energy Consumption
% Version 2.2 - Single Lap, Pre-Smoothed Dynamics, & Gradient Integration
% =========================================================================

% clear; clc; close all;

fprintf('\n========================================================\n');
fprintf('MODULE 3: ENERGY AND POWER ANALYSIS (v2.2)\n');
fprintf('========================================================\n');

%% 1. Load Data & Filter to Single Lap
fprintf('\nLoading canonical drive cycle (with merged track incline)...\n');
data = readtable('canonical_with_incline.csv');

% --- HARDWARE CORRECTION: FILTER TO A SINGLE LAP ONLY ---
single_lap_duration = data.elapsed_sec(end) / 11;
data = data(data.elapsed_sec <= single_lap_duration, :);
fprintf('  *** Dataset successfully reduced to a SINGLE LAP globally ***\n');
% --------------------------------------------------------

fprintf('  Data points: %d\n', height(data));
fprintf('  Duration: %.2f minutes\n', data.elapsed_sec(end)/60);
fprintf('  Distance: %.3f km\n', data.Distance_km(end));

%% 2. Vehicle Parameters
fprintf('\nVehicle parameters:\n');

% Assuming standard Eco-marathon prototype weights (Car + Driver)
vehicle_mass = 175;
wheel_radius = 0.275;
gravity = 9.81;
motor_gear_ratio = 5.6;
drag_coefficient = 0.15;
frontal_area = 0.4;
air_density = 1.225;
rolling_resistance_coef = 0.0013;
drivetrain_efficiency = 0.95; % Updated from 0.85 to realistic chain/hub efficiency
motor_power_rated = 1200;
motor_efficiency = 0.96;

fprintf('  Mass: %.0f kg, Wheel radius: %.3f m\n', vehicle_mass, wheel_radius);
fprintf('  Drivetrain Eff: %.0f%%\n', drivetrain_efficiency*100);

%% 3. Calculate Velocity and Acceleration (PRE-SMOOTHED)
fprintf('\nCalculating vehicle dynamics...\n');

velocity_ms = data.Speed_kmh / 3.6;

% FIX 1: Smooth velocity BEFORE taking the derivative to kill sensor chatter
avg_sample_rate = 1 / mean(diff(data.elapsed_sec));
window_size = round(10 * avg_sample_rate); % 10-second window
velocity_smooth = movmean(velocity_ms, window_size);

dt = diff(data.elapsed_sec);
dt(dt < 0.01) = mean(dt); % Prevent tiny dt spikes causing infinite acceleration
dv = diff(velocity_smooth);
acceleration = [0; dv ./ dt];

% Flatten GPS phantom acceleration spikes
acceleration_smooth = movmean(acceleration, 30); 

%% 4. Calculate Forces and Power
fprintf('\nCalculating forces and power demand...\n');

% Use smoothed velocity for physical forces to prevent noise propagation
drag_force = 0.5 * air_density * drag_coefficient * frontal_area * velocity_smooth.^2;
rolling_force = rolling_resistance_coef * vehicle_mass * gravity * ones(size(velocity_smooth));
inertial_force = vehicle_mass * acceleration_smooth;

% NEW: Calculate Gradient Force using the merged incline data
gradient_force = -vehicle_mass * gravity * sin(deg2rad(data.inc_angle_deg));

% Total Force
tractive_force = drag_force + rolling_force + inertial_force + gradient_force;

% FIX 2: Smooth the raw power BEFORE clipping negative values
power_wheel_raw = tractive_force .* velocity_smooth;
power_wheel_smooth = movmean(power_wheel_raw, window_size);

power_motor_raw = power_wheel_smooth / drivetrain_efficiency;
% NOW clip to 0 (assuming no regenerative braking)
power_motor_raw(power_motor_raw < 0) = 0;
power_battery_raw = power_motor_raw / motor_efficiency;

power_motor_smooth = power_motor_raw;
power_battery_smooth = power_battery_raw;

%% 5. Calculate Individual Power Components for Breakdown
power_aero = (drag_force .* velocity_smooth) / drivetrain_efficiency;
power_roll = (rolling_force .* velocity_smooth) / drivetrain_efficiency;
power_accel = (inertial_force .* velocity_smooth) / drivetrain_efficiency;
power_grad = (gradient_force .* velocity_smooth) / drivetrain_efficiency;

power_accel(power_accel < 0) = 0; % No regen

power_aero_smooth = movmean(power_aero, window_size);
power_roll_smooth = movmean(power_roll, window_size);
power_accel_smooth = movmean(power_accel, window_size);
power_grad_smooth = movmean(power_grad, window_size);

%% 6. Calculate Total Energy
energy_Wh = trapz(data.elapsed_sec, power_battery_smooth) / 3600;
energy_per_km = energy_Wh / data.Distance_km(end);

% Component Energy
energy_drag_Wh = trapz(data.elapsed_sec, power_aero_smooth / motor_efficiency) / 3600;
energy_rolling_Wh = trapz(data.elapsed_sec, power_roll_smooth / motor_efficiency) / 3600;
energy_inertial_Wh = trapz(data.elapsed_sec, power_accel_smooth / motor_efficiency) / 3600;
energy_grad_Wh = trapz(data.elapsed_sec, power_grad_smooth / motor_efficiency) / 3600;

% Prevent negative gradient energy from breaking the pie/bar chart logic
energy_grad_Wh(energy_grad_Wh < 0) = 0; 

total_component_energy = energy_drag_Wh + energy_rolling_Wh + energy_inertial_Wh + energy_grad_Wh;
pct_drag = (energy_drag_Wh / total_component_energy) * 100;
pct_rolling = (energy_rolling_Wh / total_component_energy) * 100;
pct_inertial = (energy_inertial_Wh / total_component_energy) * 100;
pct_grad = (energy_grad_Wh / total_component_energy) * 100;

%% 7. Torque and RPM
wheel_rpm = (velocity_smooth / (2 * pi * wheel_radius)) * 60;
motor_rpm_from_velocity = wheel_rpm * motor_gear_ratio;

omega = (data.MotorRPM + 1) * (2 * pi / 60); % +1 prevents div by zero
torque_Nm = power_motor_smooth ./ omega;
torque_Nm(isnan(torque_Nm)) = 0;
torque_Nm(isinf(torque_Nm)) = 0;
torque_Nm(torque_Nm > 50) = 50;

%% ========================================================================
% FIGURE 4: Power and Energy Analysis
% =========================================================================
fprintf('\nGenerating Figure 4 (Power and Energy Analysis)...\n');
figure('Position', [100, 100, 1400, 800], 'Color', 'white');

% Subplot 1: Motor Power Breakdown
subplot(2, 2, 1);
hold on;
plot(data.elapsed_sec/60, power_aero_smooth, 'Color', [1 0.5 0], 'LineWidth', 1.5, 'DisplayName', 'Aero Power');
plot(data.elapsed_sec/60, power_roll_smooth, 'Color', [0.8 0.2 0.2], 'LineWidth', 1.5, 'DisplayName', 'Rolling Power');
plot(data.elapsed_sec/60, power_accel_smooth, 'Color', [0.2 0.6 0.9], 'LineWidth', 1.5, 'DisplayName', 'Inertial (Accel)');
plot(data.elapsed_sec/60, power_grad_smooth, 'Color', [0.2 0.8 0.2], 'LineWidth', 1.5, 'DisplayName', 'Gradient (Hills)');
plot(data.elapsed_sec/60, power_motor_smooth, 'k-', 'LineWidth', 2, 'DisplayName', 'Total Motor Power');

yline(motor_power_rated, 'r--', '1200W Rated Limit', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Power (W)', 'FontSize', 11);
title('Motor Power Breakdown vs Hardware Limit', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
grid on;
ylim([min(power_grad_smooth)*1.1 max(power_motor_smooth)*1.1]);
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 2: Battery Power
subplot(2, 2, 2);
plot(data.elapsed_sec/60, power_battery_smooth, 'r-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Battery Power (W)', 'FontSize', 11);
title(sprintf('Total Battery Power Demand - %.1f Wh/km', energy_per_km), 'FontSize', 12, 'FontWeight', 'bold');
grid on;
ylim([0 max(power_battery_smooth)*1.1]);
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 3: Energy Breakdown
subplot(2, 2, 3);
bar_data = [energy_drag_Wh, energy_rolling_Wh, energy_inertial_Wh, energy_grad_Wh];
bar_colors = [1 0.5 0; 0.8 0.2 0.2; 0.2 0.6 0.9; 0.2 0.8 0.2];
b = bar(bar_data, 'FaceColor', 'flat');
b.CData = bar_colors;
set(gca, 'XTickLabel', {'Aero', 'Rolling', 'Accel', 'Gradient'}, 'FontSize', 10, 'Color', 'white');
ylabel('Energy (Wh)', 'FontSize', 11);
title('Energy Breakdown (1 Lap)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

pct_values = [pct_drag, pct_rolling, pct_inertial, pct_grad];
for i = 1:4
    text(i, bar_data(i) + (max(bar_data)*0.05), sprintf('%.1f%%', pct_values(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
end

% Subplot 4: Cumulative Energy
subplot(2, 2, 4);
cumulative_energy = cumtrapz(data.elapsed_sec, power_battery_smooth) / 3600;
plot(data.elapsed_sec/60, cumulative_energy, 'Color', [0.6 0.2 0.8], 'LineWidth', 2);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Cumulative Energy (Wh)', 'FontSize', 11);
title(sprintf('Cumulative Energy Consumption (Total: %.2f Wh)', energy_Wh), 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

%% ========================================================================
% FIGURE 5: Torque vs RPM + Analysis
% =========================================================================
fprintf('\nGenerating Figure 5 (Torque and RPM Analysis)...\n');
figure('Position', [150, 150, 1400, 800], 'Color', 'white');

% Subplot 1: Torque vs RPM (velocity-based)
subplot(2, 2, 1);
valid_idx = (motor_rpm_from_velocity > 50) & (torque_Nm > 0.1) & (velocity_smooth > 1);
scatter(motor_rpm_from_velocity(valid_idx), torque_Nm(valid_idx), 20, power_motor_smooth(valid_idx), 'filled');
colormap(jet);
cb = colorbar;
cb.Label.String = 'Power (W)';
xlabel('Motor RPM', 'FontSize', 11);
ylabel('Torque (Nm)', 'FontSize', 11);
title('Torque vs RPM Map', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 2: Power Histogram
subplot(2, 2, 2);
histogram(power_motor_smooth(power_motor_smooth > 10), 30, 'FaceColor', [0.2 0.6 0.9], 'EdgeColor', 'black');
xlabel('Power (W)', 'FontSize', 11);
ylabel('Frequency', 'FontSize', 11);
title('Power Distribution', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 3: Speed vs Power
subplot(2, 2, 3);
scatter(data.Speed_kmh, power_motor_smooth, 15, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('Speed (km/h)', 'FontSize', 11);
ylabel('Motor Power (W)', 'FontSize', 11);
title('Power vs Speed', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 4: Force Components (Now including Gradient)
subplot(2, 2, 4);
hold on;
plot(data.elapsed_sec/60, drag_force, 'Color', [1 0.5 0], 'LineWidth', 1.5, 'DisplayName', 'Aero Drag');
plot(data.elapsed_sec/60, rolling_force, 'Color', [0.8 0.2 0.2], 'LineWidth', 1.5, 'DisplayName', 'Rolling');
plot(data.elapsed_sec/60, max(0, inertial_force), 'Color', [0.2 0.6 0.9], 'LineWidth', 1.5, 'DisplayName', 'Inertia');
plot(data.elapsed_sec/60, gradient_force, 'Color', [0.2 0.8 0.2], 'LineWidth', 1.5, 'DisplayName', 'Gradient');
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Force (N)', 'FontSize', 11);
title('Force Components (Including Track Geometry)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

%% Summary Printout
fprintf('\n========================================================\n');
fprintf('ENERGY ANALYSIS SUMMARY (SINGLE LAP)\n');
fprintf('========================================================\n');
fprintf('  Distance: %.3f km\n', data.Distance_km(end));
fprintf('  Total Energy: %.2f Wh\n', energy_Wh);
fprintf('  Efficiency: %.2f Wh/km\n', energy_per_km);
fprintf('  Peak motor power: %.1f W\n', max(power_motor_smooth));
fprintf('========================================================\n\n');