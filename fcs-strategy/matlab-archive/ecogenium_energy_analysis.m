% =========================================================================
% ECOGENIUM ENERGY ANALYSIS MODULE
% Shell Eco-marathon - Hydraix I Power and Energy Consumption
% Version 2.1 - Minimal Updates (January 2026)
% =========================================================================

% clear; clc; close all;

fprintf('\n========================================================\n');
fprintf('MODULE 3: ENERGY AND POWER ANALYSIS (v2.1)\n');
fprintf('========================================================\n');

%% Load Canonical Drive Cycle
fprintf('\nLoading canonical drive cycle...\n');
data = readtable('canonical_drive_cycle_35min.csv');
fprintf('  Data points: %d\n', height(data));
fprintf('  Duration: %.2f minutes\n', data.elapsed_sec(end)/60);
fprintf('  Distance: %.3f km\n', data.Distance_km(end));

%% Vehicle Parameters (UPDATED - January 2026)
fprintf('\nVehicle parameters:\n');

vehicle_mass = 180;
wheel_radius = 0.275;
gravity = 9.81;
motor_gear_ratio = 5.6;
drag_coefficient = 0.15;
frontal_area = 0.4;
air_density = 1.225;
rolling_resistance_coef = 0.0013;
drivetrain_efficiency = 0.85;
motor_power_rated = 1200;
motor_efficiency = 0.96;

fprintf('  Mass: %.0f kg, Wheel radius: %.3f m\n', vehicle_mass, wheel_radius);
fprintf('  Gear ratio: %.1f\n', motor_gear_ratio);
fprintf('  Drag coef: %.2f, Frontal area: %.2f m^2\n', drag_coefficient, frontal_area);
fprintf('  Rolling resistance: %.4f\n', rolling_resistance_coef);
fprintf('  Motor: %.0fW at %.0f%% efficiency\n', motor_power_rated, motor_efficiency*100);

%% Calculate Velocity and Acceleration
fprintf('\nCalculating vehicle dynamics...\n');

velocity_ms = data.Speed_kmh / 3.6;

dt = diff(data.elapsed_sec);
dt(dt < 0.01) = 0.01;
dv = diff(velocity_ms);
acceleration = [0; dv ./ dt];
acceleration_smooth = movmean(acceleration, 5);

%% Calculate Forces and Power
fprintf('\nCalculating forces and power demand...\n');

drag_force = 0.5 * air_density * drag_coefficient * frontal_area * velocity_ms.^2;
rolling_force = rolling_resistance_coef * vehicle_mass * gravity * ones(size(velocity_ms));
inertial_force = vehicle_mass * acceleration_smooth;
tractive_force = drag_force + rolling_force + inertial_force;

power_wheel = tractive_force .* velocity_ms;
power_motor_raw = power_wheel / drivetrain_efficiency;
power_motor_raw(power_motor_raw < 0) = 0;
power_battery_raw = power_motor_raw / motor_efficiency;

% Apply 10-second moving average
avg_sample_rate = 1 / mean(diff(data.elapsed_sec));
window_size = round(10 * avg_sample_rate);
power_motor_smooth = movmean(power_motor_raw, window_size);
power_battery_smooth = movmean(power_battery_raw, window_size);

fprintf('  Peak power (raw): %.1f W\n', max(power_motor_raw));
fprintf('  Peak power (smoothed): %.1f W\n', max(power_motor_smooth));

%% Calculate Energy
energy_raw_Wh = trapz(data.elapsed_sec, power_battery_raw) / 3600;
energy_smooth_Wh = trapz(data.elapsed_sec, power_battery_smooth) / 3600;
energy_diff_pct = ((energy_smooth_Wh - energy_raw_Wh) / energy_raw_Wh) * 100;

fprintf('\n  Energy (raw): %.2f Wh\n', energy_raw_Wh);
fprintf('  Energy (smoothed): %.2f Wh\n', energy_smooth_Wh);
fprintf('  Difference: %.2f%%\n', energy_diff_pct);

if abs(energy_diff_pct) < 5
    fprintf('  ✓ Smoothing change is MINIMAL and ACCEPTABLE (<5%%)\n');
else
    fprintf('  ⚠ Smoothing causes >5%% change\n');
end

energy_per_km = energy_smooth_Wh / data.Distance_km(end);

%% Calculate Torque and Velocity-based RPM
fprintf('\nCalculating torque and velocity-based RPM...\n');

wheel_rpm = (velocity_ms / (2 * pi * wheel_radius)) * 60;
motor_rpm_from_velocity = wheel_rpm * motor_gear_ratio;

motor_rpm_data = data.MotorRPM;
motor_rpm_data(motor_rpm_data < 1) = 1;
omega = motor_rpm_data * (2 * pi / 60);
torque_Nm = power_motor_smooth ./ omega;
torque_Nm(isnan(torque_Nm)) = 0;
torque_Nm(isinf(torque_Nm)) = 0;
torque_Nm(torque_Nm > 50) = 50;

fprintf('  Max torque: %.2f Nm\n', max(torque_Nm));
fprintf('  Max RPM (data): %.0f\n', max(data.MotorRPM));
fprintf('  Max RPM (velocity): %.0f\n', max(motor_rpm_from_velocity));

%% Energy Breakdown
energy_drag_Wh = trapz(data.elapsed_sec, drag_force .* velocity_ms / drivetrain_efficiency / motor_efficiency) / 3600;
energy_rolling_Wh = trapz(data.elapsed_sec, rolling_force .* velocity_ms / drivetrain_efficiency / motor_efficiency) / 3600;
inertial_positive = max(0, inertial_force .* velocity_ms / drivetrain_efficiency / motor_efficiency);
energy_inertial_Wh = trapz(data.elapsed_sec, inertial_positive) / 3600;

pct_drag = (energy_drag_Wh / energy_smooth_Wh) * 100;
pct_rolling = (energy_rolling_Wh / energy_smooth_Wh) * 100;
pct_inertial = (energy_inertial_Wh / energy_smooth_Wh) * 100;

%% FIGURE 4: Power and Energy Analysis
fprintf('\nGenerating Figure 4 (Power and Energy Analysis)...\n');

figure('Position', [100, 100, 1400, 800], 'Color', 'white');

% Subplot 1: Motor Power
subplot(2, 2, 1);
hold on;
plot(data.elapsed_sec/60, power_motor_raw, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
plot(data.elapsed_sec/60, power_motor_smooth, 'b-', 'LineWidth', 2);
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Power (W)', 'FontSize', 11);
title('Motor Power Demand (raw + 10s smoothed)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Raw', 'Smoothed (10s)', 'Location', 'northeast', 'FontSize', 9);
grid on;
ylim([0 max(power_motor_smooth)*1.1]);
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 2: Battery Power
subplot(2, 2, 2);
plot(data.elapsed_sec/60, power_battery_smooth, 'r-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Battery Power (W)', 'FontSize', 11);
title(sprintf('Battery Power (smoothed) - %.1f Wh total', energy_smooth_Wh), ...
      'FontSize', 12, 'FontWeight', 'bold');
grid on;
ylim([0 max(power_battery_smooth)*1.1]);
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 3: Energy Breakdown
subplot(2, 2, 3);
bar_data = [energy_drag_Wh, energy_rolling_Wh, energy_inertial_Wh];
bar_colors = [1 0.5 0; 0.8 0.2 0.2; 0.2 0.6 0.9];
b = bar(bar_data, 'FaceColor', 'flat');
b.CData = bar_colors;
set(gca, 'XTickLabel', {'Aero Drag', 'Rolling', 'Acceleration'}, 'FontSize', 10, 'Color', 'white');
ylabel('Energy (Wh)', 'FontSize', 11);
title('Energy Breakdown', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% Add percentage labels
pct_values = [pct_drag, pct_rolling, pct_inertial];
for i = 1:3
    text(i, bar_data(i)*1.05, sprintf('%.1f%%', pct_values(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
end

% Subplot 4: Cumulative Energy
subplot(2, 2, 4);
cumulative_energy = cumtrapz(data.elapsed_sec, power_battery_smooth) / 3600;
plot(data.elapsed_sec/60, cumulative_energy, 'Color', [0.6 0.2 0.8], 'LineWidth', 2);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Cumulative Energy (Wh)', 'FontSize', 11);
title(sprintf('Energy Consumption - %.2f Wh/km', energy_per_km), ...
      'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

%% FIGURE 5: Torque vs RPM + Analysis
fprintf('\nGenerating Figure 5 (Torque and RPM Analysis)...\n');

figure('Position', [150, 150, 1400, 800], 'Color', 'white');

% Subplot 1: Torque vs RPM (velocity-based)
subplot(2, 2, 1);
valid_idx = (motor_rpm_from_velocity > 50) & (torque_Nm > 0.1) & (velocity_ms > 1);
scatter(motor_rpm_from_velocity(valid_idx), torque_Nm(valid_idx), 20, ...
        power_motor_smooth(valid_idx), 'filled');
colormap(jet);
cb = colorbar;
cb.Label.String = 'Power (W)';
cb.Label.FontSize = 10;
xlabel('Motor RPM (velocity-based)', 'FontSize', 11);
ylabel('Torque (Nm)', 'FontSize', 11);
title('Torque vs RPM (velocity-based)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% Subplot 2: Power Histogram
subplot(2, 2, 2);
histogram(power_motor_smooth(power_motor_smooth > 10), 30, ...
          'FaceColor', [0.2 0.6 0.9], 'EdgeColor', 'black');
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

% Subplot 4: Force Components
subplot(2, 2, 4);
hold on;
plot(data.elapsed_sec/60, drag_force, 'Color', [1 0.5 0], 'LineWidth', 1.5, ...
     'DisplayName', 'Aero Drag');
plot(data.elapsed_sec/60, rolling_force, 'Color', [0.8 0.2 0.2], 'LineWidth', 1.5, ...
     'DisplayName', 'Rolling');
plot(data.elapsed_sec/60, max(0, inertial_force), 'Color', [0.2 0.6 0.9], 'LineWidth', 1.5, ...
     'DisplayName', 'Inertia');
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Force (N)', 'FontSize', 11);
title('Force Components', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

%% Summary
fprintf('\n========================================================\n');
fprintf('ENERGY ANALYSIS SUMMARY\n');
fprintf('========================================================\n');
fprintf('\nPerformance:\n');
fprintf('  Distance: %.3f km in %.2f min\n', data.Distance_km(end), data.elapsed_sec(end)/60);
fprintf('  Energy: %.2f Wh (%.2f Wh/km)\n', energy_smooth_Wh, energy_per_km);
fprintf('  Peak power: %.1f W\n', max(power_motor_smooth));
fprintf('  Peak torque: %.2f Nm\n', max(torque_Nm));
fprintf('\nSmoothing Impact:\n');
fprintf('  Raw: %.2f Wh\n', energy_raw_Wh);
fprintf('  Smoothed: %.2f Wh\n', energy_smooth_Wh);
fprintf('  Change: %.2f%%\n', energy_diff_pct);

fprintf('\n========================================================\n');
fprintf('MODULE 3 COMPLETE\n');
fprintf('Figures 4 and 5 generated successfully.\n');
fprintf('========================================================\n\n');
