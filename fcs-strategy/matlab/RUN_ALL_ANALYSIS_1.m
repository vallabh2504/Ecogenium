% =========================================================================
% ECOGENIUM MASTER ANALYSIS SCRIPT
% Runs all analysis modules in sequence
% =========================================================================
% This script executes the complete telemetry analysis suite:
%   1. Main drive cycle analysis
%   2. Comparative race analysis
%   3. Energy consumption and power demand analysis
% =========================================================================

clear all; close all; clc;

fprintf('\n');
fprintf('========================================================================\n');
fprintf('     ECOGENIUM TELEMETRY ANALYSIS SUITE - MASTER SCRIPT\n');
fprintf('     Shell Eco-marathon 2025 - Hydraix I Performance Analysis\n');
fprintf('========================================================================\n');
fprintf('\n');

%% Check if required files exist
fprintf('Checking for required data files...\n');
required_files = {
    'canonical_drive_cycle_35min.csv', 
    'race1_cleaned.csv', 
    'race2_cleaned.csv', 
    'race4_cleaned.csv'
};

missing_files = false;
for i = 1:length(required_files)
    if ~isfile(required_files{i})
        fprintf('  ERROR: Missing file: %s\n', required_files{i});
        missing_files = true;
    else
        fprintf('  ✓ Found: %s\n', required_files{i});
    end
end

if missing_files
    fprintf('\nPlease ensure all CSV files are in the current directory.\n');
    fprintf('Aborting analysis.\n\n');
    return;
end

fprintf('\nAll required files found. Proceeding with analysis...\n\n');

%% Module 1: Main Drive Cycle Analysis
fprintf('========================================================================\n');
fprintf(' MODULE 1: CANONICAL DRIVE CYCLE ANALYSIS\n');
fprintf('========================================================================\n');
pause(1);

try
    run('ecogenium_main_analysis.m');
    fprintf('\n✓ Module 1 completed successfully\n\n');
catch ME
    fprintf('\n✗ Module 1 failed: %s\n\n', ME.message);
end

pause(2);

%% Module 2: Comparative Race Analysis
fprintf('========================================================================\n');
fprintf(' MODULE 2: COMPARATIVE RACE ANALYSIS\n');
fprintf('========================================================================\n');
pause(1);

try
    run('ecogenium_comparative_analysis.m');
    fprintf('\n✓ Module 2 completed successfully\n\n');
catch ME
    fprintf('\n✗ Module 2 failed: %s\n\n', ME.message);
end

pause(2);

%% Module 3: Energy Consumption Analysis
fprintf('========================================================================\n');
fprintf(' MODULE 3: ENERGY & POWER DEMAND ANALYSIS\n');
fprintf('========================================================================\n');
pause(1);

try
    run('ecogenium_energy_analysis_1.m');
    fprintf('\n✓ Module 3 completed successfully\n\n');
catch ME
    fprintf('\n✗ Module 3 failed: %s\n\n', ME.message);
end

%% Final Summary
fprintf('\n');
fprintf('========================================================================\n');
fprintf(' ANALYSIS SUITE COMPLETE\n');
fprintf('========================================================================\n');
fprintf('\n');
fprintf('Generated files:\n');
fprintf('  • Multiple analysis figures (check Figure windows)\n');
fprintf('  • energy_analysis_results.csv\n');
fprintf('\n');
fprintf('Next Steps:\n');
fprintf('  1. Review all generated figures\n');
fprintf('  2. Adjust parameters in ecogenium_energy_analysis.m if needed\n');
fprintf('  3. Export figures for reports (File > Save As)\n');
fprintf('  4. Use canonical_drive_cycle_35min.csv for simulations\n');
fprintf('\n');
fprintf('========================================================================\n');
fprintf('  For questions, refer to README_ECOGENIUM.txt\n');
fprintf('========================================================================\n');
fprintf('\n');
