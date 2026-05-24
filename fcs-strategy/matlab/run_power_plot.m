% =========================================================================
% HEADLESS WRAPPER — saves Power vs Time plot to results/
% Run with: octave --no-gui run_power_plot.m
% =========================================================================

graphics_toolkit('gnuplot');

% ---- same parameters as ecogenium_energy_analysis_1.m (fixed values) ----
vehicle_mass          = 175;    % kg  (corrected from 160)
wheel_radius          = 0.295;  % m
gravity               = 9.81;
drag_coefficient      = 0.15;
frontal_area          = 0.8;    % m^2
rolling_resistance_coef = 0.006;
air_density           = 1.225;
drivetrain_efficiency = 0.95;   % corrected from 0.85
motor_power_rated     = 760;    % W (2 × 380 W)
motor_efficiency      = 0.96;

fprintf('Loading drive cycle...\n');
data = readtable('canonical_with_incline.csv');

single_lap_duration = data.elapsed_sec(end) / 11;
data = data(data.elapsed_sec <= single_lap_duration, :);

time       = data.elapsed_sec;
speed_kmh  = data.Speed_kmh;
incline_deg = data.inc_angle_deg;

speed_ms   = speed_kmh / 3.6;

% ---- FIXED signal processing: smooth BEFORE differentiating ----
avg_sample_rate_val = 1 / mean(diff(time));
vel_window          = round(10 * avg_sample_rate_val);
speed_ms_smooth     = movmean(speed_ms, vel_window);

dt = diff(time);
dt(dt < 0.01) = mean(diff(time));
dt = [dt(1); dt];
acceleration = [0; diff(speed_ms_smooth)] ./ dt;
acceleration = movmean(acceleration, 30);

fprintf('  Lap duration : %.2f min\n', time(end)/60);
fprintf('  Lap distance : %.3f km\n', data.Distance_km(end));
fprintf('  Data points  : %d\n', numel(time));

% ---- Forces ----
P_rolling  = zeros(size(speed_ms_smooth));
P_aero     = zeros(size(speed_ms_smooth));
P_accel    = zeros(size(speed_ms_smooth));
P_gradient = zeros(size(speed_ms_smooth));
P_motor    = zeros(size(speed_ms_smooth));
P_elec     = zeros(size(speed_ms_smooth));

for i = 1:numel(speed_ms_smooth)
    v = speed_ms_smooth(i);
    a = acceleration(i);

    P_rolling(i)  = rolling_resistance_coef * vehicle_mass * gravity * v;
    P_aero(i)     = 0.5 * drag_coefficient * frontal_area * air_density * v^3;
    P_accel(i)    = vehicle_mass * a * v;
    P_gradient(i) = vehicle_mass * gravity * sin(deg2rad(incline_deg(i))) * v;

    P_wheel = P_rolling(i) + P_aero(i) + P_accel(i) + P_gradient(i);

    if P_wheel > 0
        P_motor(i) = P_wheel / drivetrain_efficiency;
        P_elec(i)  = P_motor(i) / motor_efficiency;
    end
end

P_elec(P_elec < 0) = 0;
P_elec(~isfinite(P_elec)) = 0;

% Component smoothing for breakdown plot
window_size_plot = round(10 * avg_sample_rate_val);
P_aero_s  = movmean(max(P_aero,  0) / drivetrain_efficiency, window_size_plot);
P_roll_s  = movmean(max(P_rolling,0) / drivetrain_efficiency, window_size_plot);
P_accel_s = movmean(max(P_accel, 0) / drivetrain_efficiency, window_size_plot);
P_grad_s  = movmean(max(P_gradient,0) / drivetrain_efficiency, window_size_plot);

fprintf('Peak power demand : %.1f W\n', max(P_elec));
fprintf('Mean power (moving): %.1f W\n', mean(P_elec(speed_ms_smooth > 0)));

energy_Wh     = trapz(time, P_elec) / 3600;
energy_per_km = energy_Wh / data.Distance_km(end);
fprintf('Total energy      : %.2f Wh  (%.2f Wh/km)\n', energy_Wh, energy_per_km);

% =========================================================================
% FIGURE: Power vs Time (4-panel)
% =========================================================================
fig = figure('Visible', 'off');
set(fig, 'Position', [0 0 1400 900]);

% --- Panel 1: Total Electrical Power Demand ---
subplot(2, 2, 1);
plot(time/60, P_elec, 'k-', 'LineWidth', 1.5);
hold on;
% yline equivalent for Octave headless
xl = xlim();
plot(xl, [motor_power_rated motor_power_rated], '--r', 'LineWidth', 2);
plot(xl, [1040 1040], '--b', 'LineWidth', 2);   % FC stack nominal max
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Electrical Power (W)', 'FontSize', 11);
title('Total Electrical Power Demand — Single Lap', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Power Demand', '760 W Motor Rated', '1040 W FC Nominal Max'}, ...
       'Location', 'northeast', 'FontSize', 9);
grid on;
ylim([0, max(max(P_elec)*1.15, motor_power_rated*1.1)]);

% --- Panel 2: Power Component Breakdown ---
subplot(2, 2, 2);
hold on;
plot(time/60, P_aero_s,  'Color', [1 0.5 0],   'LineWidth', 1.5, 'DisplayName', 'Aero');
plot(time/60, P_roll_s,  'Color', [0.8 0.2 0.2],'LineWidth', 1.5, 'DisplayName', 'Rolling');
plot(time/60, P_accel_s, 'Color', [0.2 0.6 0.9],'LineWidth', 1.5, 'DisplayName', 'Inertial');
plot(time/60, P_grad_s,  'Color', [0.2 0.8 0.2],'LineWidth', 1.5, 'DisplayName', 'Gradient');
plot(time/60, P_elec,    'k-',                   'LineWidth', 2,   'DisplayName', 'Total');
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Power (W)', 'FontSize', 11);
title('Power Component Breakdown', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
grid on;

% --- Panel 3: Power vs Speed scatter ---
subplot(2, 2, 3);
scatter(speed_kmh, P_elec, 10, time, 'filled');
colormap(jet);
cb = colorbar();
set(get(cb,'label'), 'String', 'Time (s)');
xlabel('Speed (km/h)', 'FontSize', 11);
ylabel('Electrical Power (W)', 'FontSize', 11);
title('Power Demand vs Speed', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% --- Panel 4: Power histogram ---
subplot(2, 2, 4);
hist(P_elec(P_elec > 10), 40);
xlabel('Power (W)', 'FontSize', 11);
ylabel('Frequency', 'FontSize', 11);
title('Power Distribution', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% Suptitle equivalent
axes('Position', [0 0 1 1], 'Visible', 'off');
text(0.5, 0.99, ...
    sprintf('Vehicle Power Analysis — m=%.0f kg, \\eta_{dt}=%.0f%%  (Peak: %.0f W, Mean: %.0f W, %.2f Wh/km)', ...
        vehicle_mass, drivetrain_efficiency*100, max(P_elec), ...
        mean(P_elec(speed_ms_smooth > 0)), energy_per_km), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'FontSize', 12, 'FontWeight', 'bold');

% ---- Save ----
results_dir = '../../results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
out_path = fullfile(results_dir, 'power_vs_time_single_lap.png');
print(fig, out_path, '-dpng', '-r150');
fprintf('\nSaved: %s\n', out_path);
