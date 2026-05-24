% =========================================================================
% ECOGENIUM COMPARATIVE RACE ANALYSIS
% Shell Eco-marathon - Hydraix I Performance Comparison
% Version 1.3 - Master Lap Highlight (January 2026)
% =========================================================================


clear; clc; close all;
fprintf('\n========================================================\n');
fprintf('MODULE 2: COMPARATIVE RACE ANALYSIS (v1.3)\n');
fprintf('========================================================\n');

%% Load Data
fprintf('\nLoading race data...\n');

race1 = readtable('race1_cleaned.csv');
race2 = readtable('race2_cleaned.csv');
race4 = readtable('race4_cleaned.csv');

fprintf('  Race 1: %d data points\n', height(race1));
fprintf('  Race 2: %d data points\n', height(race2));
fprintf('  Race 4: %d data points\n', height(race4));

%% Create Comparative Visualization (6 subplots)
fprintf('\nGenerating Figure 3 (6-subplot comparative analysis)...\n');

figure('Position', [100, 100, 1400, 1000], 'Color', 'white');

% Master lap segment coordinates (2798-2998s)
master_start_min = 2798 / 60;
master_end_min = 2998 / 60;

% SUBPLOT 1: Race 1 Speed
subplot(3, 2, 1);
plot(race1.elapsed_sec/60, race1.Speed_kmh, 'b-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Speed (km/h)', 'FontSize', 11);
title('Race 1 - Speed Profile', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% SUBPLOT 2: Race 2 Speed
subplot(3, 2, 2);
plot(race2.elapsed_sec/60, race2.Speed_kmh, 'r-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Speed (km/h)', 'FontSize', 11);
title('Race 2 - Speed Profile', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% SUBPLOT 3: Race 1 RPM
subplot(3, 2, 3);
plot(race1.elapsed_sec/60, race1.MotorRPM, 'b-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Motor RPM', 'FontSize', 11);
title('Race 1 - Motor RPM', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% SUBPLOT 4: Race 2 RPM
subplot(3, 2, 4);
plot(race2.elapsed_sec/60, race2.MotorRPM, 'r-', 'LineWidth', 1.5);
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Motor RPM', 'FontSize', 11);
title('Race 2 - Motor RPM', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% SUBPLOT 5: Race 4 Speed WITH HIGHLIGHT
subplot(3, 2, 5);
hold on;
% Plot speed
plot(race4.elapsed_sec/60, race4.Speed_kmh, 'Color', [0 0.5 0], 'LineWidth', 1.5);
% Get current limits
ylims = ylim;
% Add yellow highlight patch
patch([master_start_min master_end_min master_end_min master_start_min], ...
      [ylims(1) ylims(1) ylims(2) ylims(2)], ...
      [1 1 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Re-plot speed on top
plot(race4.elapsed_sec/60, race4.Speed_kmh, 'Color', [0 0.5 0], 'LineWidth', 1.5);
% Add annotation
text((master_start_min + master_end_min)/2, ylims(2)*0.92, ...
     '\leftarrow Master Lap', 'HorizontalAlignment', 'center', ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.6 0.4 0]);
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Speed (km/h)', 'FontSize', 11);
title('Race 4 - Speed Profile (with Master Lap)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

% SUBPLOT 6: Race 4 RPM WITH HIGHLIGHT
subplot(3, 2, 6);
hold on;
% Plot RPM
plot(race4.elapsed_sec/60, race4.MotorRPM, 'Color', [0 0.5 0], 'LineWidth', 1.5);
% Get current limits
ylims_rpm = ylim;
% Add yellow highlight patch
patch([master_start_min master_end_min master_end_min master_start_min], ...
      [ylims_rpm(1) ylims_rpm(1) ylims_rpm(2) ylims_rpm(2)], ...
      [1 1 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Re-plot RPM on top
plot(race4.elapsed_sec/60, race4.MotorRPM, 'Color', [0 0.5 0], 'LineWidth', 1.5);
hold off;
xlabel('Time (minutes)', 'FontSize', 11);
ylabel('Motor RPM', 'FontSize', 11);
title('Race 4 - Motor RPM (with Master Lap)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10, 'Color', 'white');

%% Calculate and Display Statistics
fprintf('\n========================================================\n');
fprintf('RACE STATISTICS SUMMARY\n');
fprintf('========================================================\n');

races = {race1, race2, race4};
race_names = {'Race 1', 'Race 2', 'Race 4'};

for i = 1:3
    race = races{i};
    fprintf('\n%s:\n', race_names{i});
    fprintf('  Duration: %.1f minutes\n', race.elapsed_sec(end)/60);
    fprintf('  Distance: %.2f km\n', race.Distance_km(end));
    fprintf('  Max Speed: %.1f km/h\n', max(race.Speed_kmh));

    % Calculate average speed for moving segments only
    moving_idx = race.Speed_kmh > 2;
    if any(moving_idx)
        fprintf('  Avg Speed: %.1f km/h\n', mean(race.Speed_kmh(moving_idx)));
    else
        fprintf('  Avg Speed: 0.0 km/h\n');
    end

    fprintf('  Max RPM: %.0f\n', max(race.MotorRPM));
end

fprintf('\n========================================================\n');
fprintf('MASTER LAP SEGMENT INFO (Race 4)\n');
fprintf('========================================================\n');
fprintf('  Time range: %.1f - %.1f minutes\n', master_start_min, master_end_min);
fprintf('  Time range: %.0f - %.0f seconds\n', 2798, 2998);
fprintf('  Duration: %.0f seconds (3 min 20 sec)\n', 2998-2798);
fprintf('  Highlighted in yellow on Race 4 subplots (row 3)\n');
fprintf('  This segment was compressed to 186.4s and replicated 11 times.\n');

fprintf('\n========================================================\n');
fprintf('MODULE 2 COMPLETE\n');
fprintf('Figure 3 generated with 6 subplots and master lap highlight.\n');
fprintf('========================================================\n\n');
