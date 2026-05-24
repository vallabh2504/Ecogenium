% =========================================================================
% ECOGENIUM TELEMETRY ANALYSIS SUITE
% Shell Eco-marathon Urban Concept - Hydraix I
% =========================================================================
% Purpose: Comprehensive analysis of race telemetry and drive cycle
% Author: Ecogenium Telemetry Taskforce
% Date: January 2026
% Version: 1.1 (Fixed)
% =========================================================================

% clear all; close all; clc;

%% ========================================================================
% SECTION 1: LOAD CANONICAL 35-MINUTE DRIVE CYCLE
% =========================================================================
fprintf('\n=== LOADING CANONICAL DRIVE CYCLE ===\n');

data = readtable('canonical_drive_cycle_35min.csv');

% Extract variables
time = data.elapsed_sec;
rpm = data.MotorRPM;
speed_units = data.Speed;
speed_kmh = data.Speed_kmh;
distance_km = data.Distance_km;
data_source = string(data.data_source);

fprintf('Total duration: %.1f minutes\n', max(time)/60);
fprintf('Total distance: %.2f km\n', max(distance_km));
fprintf('Average speed: %.1f km/h\n', mean(speed_kmh(speed_kmh > 0)));
fprintf('Real data: %.1f%%, Synthetic: %.1f%%\n', ...
    sum(data_source == "real")/height(data)*100, ...
    sum(data_source == "synthetic")/height(data)*100);

%% ========================================================================
% SECTION 2: VISUALIZATION - SPEED AND RPM PROFILES
% =========================================================================
fprintf('\n=== GENERATING VISUALIZATIONS ===\n');

figure('Position', [100, 100, 1400, 800], 'Name', 'Canonical Drive Cycle', 'Color', 'white');

% Subplot 1: Speed vs Time
subplot(3,1,1);
hold on;
% Highlight real vs synthetic sections
real_idx = data_source == "real";
synthetic_idx = data_source == "synthetic";
plot(time(real_idx)/60, speed_kmh(real_idx), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Real Data');
plot(time(synthetic_idx)/60, speed_kmh(synthetic_idx), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Synthetic Data');
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Speed (km/h)', 'FontSize', 12);
title('Vehicle Speed Profile - 35 Minute Drive Cycle', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
legend('Location', 'best');
ylim([0 max(speed_kmh)*1.1]);
set(gca, 'Color', 'white');

% Subplot 2: Motor RPM vs Time
subplot(3,1,2);
hold on;
plot(time(real_idx)/60, rpm(real_idx), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Real Data');
plot(time(synthetic_idx)/60, rpm(synthetic_idx), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Synthetic Data');
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Motor RPM', 'FontSize', 12);
title('Motor RPM Profile', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
legend('Location', 'best');
ylim([0 max(rpm)*1.1]);
set(gca, 'Color', 'white');

% Subplot 3: Distance vs Time
subplot(3,1,3);
plot(time/60, distance_km, 'k-', 'LineWidth', 2);
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Distance (km)', 'FontSize', 12);
title('Cumulative Distance Traveled', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'Color', 'white');

% Add vertical line at real/synthetic boundary
real_end_time = max(time(real_idx))/60;
for i = 1:3
    subplot(3,1,i);
    xline(real_end_time, '--k', 'LineWidth', 1.5, 'Label', 'Synthetic Start', ...
        'LabelVerticalAlignment', 'bottom', 'FontSize', 10);
end

fprintf('Figure 1: Speed, RPM, and Distance profiles created\n');

%% ========================================================================
% SECTION 3: PULSE-AND-GLIDE ANALYSIS
% =========================================================================
fprintf('\n=== PULSE-AND-GLIDE ANALYSIS ===\n');

figure('Position', [150, 150, 1400, 600], 'Name', 'Pulse-and-Glide Analysis', 'Color', 'white');

% Identify driving modes
throttle_threshold = 400;  % RPM threshold for "full throttle"
throttle_mode = rpm >= throttle_threshold;
coast_mode = rpm == 0;

% Calculate duty cycle
duty_cycle = sum(throttle_mode) / height(data) * 100;
fprintf('Throttle duty cycle: %.1f%%\n', duty_cycle);
fprintf('Coasting time: %.1f%%\n', sum(coast_mode)/height(data)*100);

% Subplot 1: Speed-RPM Phase Plot
subplot(1,2,1);
scatter(rpm(throttle_mode), speed_kmh(throttle_mode), 20, 'r', 'filled', 'DisplayName', 'Full Throttle');
hold on;
scatter(rpm(coast_mode), speed_kmh(coast_mode), 20, 'b', 'filled', 'DisplayName', 'Coasting');
scatter(rpm(~throttle_mode & ~coast_mode), speed_kmh(~throttle_mode & ~coast_mode), ...
    20, [0.5 0.5 0.5], 'filled', 'DisplayName', 'Partial Throttle');
xlabel('Motor RPM', 'FontSize', 12);
ylabel('Speed (km/h)', 'FontSize', 12);
title('Speed-RPM Phase Plot', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
set(gca, 'Color', 'white');

% Subplot 2: Driving Mode Timeline
subplot(1,2,2);
mode_signal = zeros(size(rpm));
mode_signal(throttle_mode) = 2;
mode_signal(~throttle_mode & ~coast_mode) = 1;
mode_signal(coast_mode) = 0;

area(time/60, mode_signal, 'FaceColor', 'flat', 'EdgeColor', 'none');
colormap([0 0.4 0.8; 0.8 0.8 0; 0.8 0 0]);  % Blue, Gray, Red
xlabel('Time (minutes)', 'FontSize', 12);
ylabel('Driving Mode', 'FontSize', 12);
title('Pulse-and-Glide Strategy Timeline', 'FontSize', 14, 'FontWeight', 'bold');
yticks([0 1 2]);
yticklabels({'Coast', 'Partial', 'Full'});
grid on;
ylim([-0.2 2.2]);
set(gca, 'Color', 'white');

fprintf('Figure 2: Pulse-and-glide analysis created\n');

%% ========================================================================
% SECTION 4: STATISTICAL SUMMARY
% =========================================================================
fprintf('\n=== STATISTICAL SUMMARY ===\n');

moving_idx = speed_kmh > 0;
fprintf('\nSpeed Statistics (km/h):\n');
fprintf('  Min: %.1f\n', min(speed_kmh(moving_idx)));
fprintf('  Max: %.1f\n', max(speed_kmh));
fprintf('  Mean: %.1f\n', mean(speed_kmh(moving_idx)));
fprintf('  Median: %.1f\n', median(speed_kmh(moving_idx)));
fprintf('  Std Dev: %.1f\n', std(speed_kmh(moving_idx)));

fprintf('\nRPM Statistics:\n');
fprintf('  Max: %d\n', max(rpm));
fprintf('  Mean (when moving): %.1f\n', mean(rpm(moving_idx)));
fprintf('  Full throttle (528 RPM): %.1f%% of race\n', sum(rpm==528)/height(data)*100);

fprintf('\n=== MAIN ANALYSIS COMPLETE ===\n');
fprintf('Canonical drive cycle analysis saved.\n\n');
