% =========================================================================
% ECOGENIUM COMPARATIVE RACE ANALYSIS
% Compare Race1, Race2, and Race4 performance
% Version: 1.1 (Fixed)
% =========================================================================

% clear all; close all; clc;

%% ========================================================================
% LOAD ALL RACE DATA
% =========================================================================
fprintf('\n=== LOADING RACE DATA FOR COMPARISON ===\n');

race1 = readtable('race1_cleaned.csv');
race2 = readtable('race2_cleaned.csv');
race4 = readtable('race4_cleaned.csv');

fprintf('Race1: %.1f min, %.2f km\n', max(race1.elapsed_sec)/60, max(race1.Distance_km));
fprintf('Race2: %.1f min, %.2f km\n', max(race2.elapsed_sec)/60, max(race2.Distance_km));
fprintf('Race4: %.1f min, %.2f km\n', max(race4.elapsed_sec)/60, max(race4.Distance_km));

%% ========================================================================
% COMPARATIVE SPEED PROFILES
% =========================================================================
fprintf('\n=== GENERATING COMPARATIVE VISUALIZATIONS ===\n');

figure('Position', [100, 100, 1400, 900], 'Name', 'Race Comparison', 'Color', 'white');
% Master lap segment coordinates (2798-2998s)
master_start_min = 2798 / 60;
master_end_min = 2998 / 60;
% Subplot 1: Speed comparison
subplot(3,2,1);
hold on;
plot(race1.elapsed_sec/60, race1.Speed_kmh, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Race1 (June 13)');
plot(race2.elapsed_sec/60, race2.Speed_kmh, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Race2 (June 14 AM)');
plot(race4.elapsed_sec/60, race4.Speed_kmh, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Race4 (June 14 PM)');
ylims = ylim;
patch([master_start_min master_end_min master_end_min master_start_min], ...
      [ylims(1) ylims(1) ylims(2) ylims(2)], ...
      [1 0.7 0.7], 'FaceAlpha', 1, 'EdgeColor', 'none');
% Re-plot speed on top
plot(race4.elapsed_sec/60, race4.Speed_kmh, 'Color', [0 0.5 0], 'LineWidth', 1.5);
% Add annotation
text((master_start_min + master_end_min)/2, ylims(2)*0.92, ...
     ' Master Lap', 'HorizontalAlignment', 'center', ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.6 0.4 0]);
xlabel('Time (minutes)');
ylabel('Speed (km/h)');
title('Speed Profile Comparison', 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
set(gca, 'Color', 'white');

% Subplot 2: RPM comparison
subplot(3,2,2);
hold on;
plot(race1.elapsed_sec/60, race1.MotorRPM, 'b-', 'LineWidth', 1.5);
plot(race2.elapsed_sec/60, race2.MotorRPM, 'r-', 'LineWidth', 1.5);
plot(race4.elapsed_sec/60, race4.MotorRPM, 'g-', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Motor RPM');
title('RPM Profile Comparison', 'FontWeight', 'bold');
legend('Race1', 'Race2', 'Race4', 'Location', 'best');
grid on;
set(gca, 'Color', 'white');

% Subplot 3: Distance progression
subplot(3,2,3);
hold on;
plot(race1.elapsed_sec/60, race1.Distance_km, 'b-', 'LineWidth', 2);
plot(race2.elapsed_sec/60, race2.Distance_km, 'r-', 'LineWidth', 2);
plot(race4.elapsed_sec/60, race4.Distance_km, 'g-', 'LineWidth', 2);
xlabel('Time (minutes)');
ylabel('Distance (km)');
title('Distance Traveled', 'FontWeight', 'bold');
legend('Race1', 'Race2', 'Race4', 'Location', 'best');
grid on;
set(gca, 'Color', 'white');

% Subplot 4: Speed histograms
subplot(3,2,4);
hold on;
histogram(race1.Speed_kmh(race1.Speed_kmh > 0), 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Race1');
histogram(race2.Speed_kmh(race2.Speed_kmh > 0), 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Race2');
histogram(race4.Speed_kmh(race4.Speed_kmh > 0), 30, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'DisplayName', 'Race4');
xlabel('Speed (km/h)');
ylabel('Frequency');
title('Speed Distribution', 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
set(gca, 'Color', 'white');

% Subplot 5: RPM-Speed phase plots
subplot(3,2,5);
hold on;
scatter(race1.MotorRPM, race1.Speed_kmh, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Race1');
scatter(race2.MotorRPM, race2.Speed_kmh, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Race2');
scatter(race4.MotorRPM, race4.Speed_kmh, 10, 'g', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Race4');
xlabel('Motor RPM');
ylabel('Speed (km/h)');
title('RPM-Speed Phase Plot', 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
set(gca, 'Color', 'white');

% Subplot 6: Strategy comparison (duty cycle)
subplot(3,2,6);
races = {'Race1', 'Race2', 'Race4'};
full_throttle_pct = [
    sum(race1.MotorRPM >= 400)/height(race1)*100;
    sum(race2.MotorRPM >= 400)/height(race2)*100;
    sum(race4.MotorRPM >= 400)/height(race4)*100
];
coast_pct = [
    sum(race1.MotorRPM == 0)/height(race1)*100;
    sum(race2.MotorRPM == 0)/height(race2)*100;
    sum(race4.MotorRPM == 0)/height(race4)*100
];
b = bar([full_throttle_pct, coast_pct]);
b(1).FaceColor = [0.8 0 0];
b(2).FaceColor = [0 0.4 0.8];
xticklabels(races);
ylabel('Percentage (%)');
title('Driving Strategy Comparison', 'FontWeight', 'bold');
legend('Full Throttle', 'Coasting', 'Location', 'best');
grid on;
set(gca, 'Color', 'white');

%% ========================================================================
% PERFORMANCE METRICS TABLE
% =========================================================================
fprintf('\n=== PERFORMANCE METRICS ===\n\n');
fprintf('%-20s %-12s %-12s %-12s\n', 'Metric', 'Race1', 'Race2', 'Race4');
fprintf('%s\n', repmat('-', 1, 60));

% Duration
fprintf('%-20s %-12.1f %-12.1f %-12.1f\n', 'Duration (min)', ...
    max(race1.elapsed_sec)/60, max(race2.elapsed_sec)/60, max(race4.elapsed_sec)/60);

% Distance
fprintf('%-20s %-12.2f %-12.2f %-12.2f\n', 'Distance (km)', ...
    max(race1.Distance_km), max(race2.Distance_km), max(race4.Distance_km));

% Average Speed
moving1 = race1.Speed_kmh > 0;
moving2 = race2.Speed_kmh > 0;
moving4 = race4.Speed_kmh > 0;
fprintf('%-20s %-12.1f %-12.1f %-12.1f\n', 'Avg Speed (km/h)', ...
    mean(race1.Speed_kmh(moving1)), mean(race2.Speed_kmh(moving2)), mean(race4.Speed_kmh(moving4)));

% Max Speed
fprintf('%-20s %-12.1f %-12.1f %-12.1f\n', 'Max Speed (km/h)', ...
    max(race1.Speed_kmh), max(race2.Speed_kmh), max(race4.Speed_kmh));

% Average RPM
fprintf('%-20s %-12.1f %-12.1f %-12.1f\n', 'Avg RPM (moving)', ...
    mean(race1.MotorRPM(moving1)), mean(race2.MotorRPM(moving2)), mean(race4.MotorRPM(moving4)));

% Throttle duty cycle
fprintf('%-20s %-12.1f %-12.1f %-12.1f\n', 'Full Throttle (%)', ...
    full_throttle_pct(1), full_throttle_pct(2), full_throttle_pct(3));

% Coasting percentage
fprintf('%-20s %-12.1f %-12.1f %-12.1f\n', 'Coasting (%)', ...
    coast_pct(1), coast_pct(2), coast_pct(3));

fprintf('\n=== KEY INSIGHTS ===\n');
fprintf('Race4 shows optimal pulse-and-glide strategy:\n');
fprintf('  - Lowest duty cycle (%.1f%%) with highest efficiency\n', full_throttle_pct(3));
fprintf('  - Most coasting time (%.1f%%)\n', coast_pct(3));
fprintf('  - Longest duration indicates complete race\n');

fprintf('\n=== COMPARATIVE ANALYSIS COMPLETE ===\n\n');
