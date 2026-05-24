% =========================================================================
% ECOGENIUM MASTER ANALYSIS SCRIPT
% Shell Eco-marathon - Hydraix I Complete Analysis Suite
% Version 2.2 - Aesthetic Update (January 2026)
% =========================================================================
%
% This script runs all three analysis modules in sequence:
%   1. Main Drive Cycle Analysis (ecogenium_main_analysis.m)
%   2. Comparative Race Analysis (ecogenium_comparative_analysis.m) - v1.2
%   3. Energy and Power Analysis (ecogenium_energy_analysis.m) - v2.0
%
% Updates in v2.0:
%   - Figure 3: Highlights master lap segment (2798-2998s in Race4)
%   - Figure 4: 10-second moving average for power curves
%   - Figure 5: New Torque vs RPM analysis
%   - Updated vehicle parameters
%
% =========================================================================

clear; clc; close all;

fprintf('\n');
fprintf('========================================================================\n');
fprintf('                ECOGENIUM TELEMETRY ANALYSIS SUITE                     \n');
fprintf('                  Shell Eco-marathon Urban Concept                     \n');
fprintf('                        Version 2.0 (2026)                             \n');
fprintf('========================================================================\n');
fprintf('\n');

%% Check if all required files exist
fprintf('Checking for required data files...\n');

required_files = {
    'canonical_drive_cycle_35min.csv', ...
    'canonical_with_incline.csv', ...
    'race1_cleaned.csv', ...
    'race2_cleaned.csv', ...
    'race4_cleaned.csv'
};

all_files_exist = true;
for i = 1:length(required_files)
    if exist(required_files{i}, 'file')
        fprintf('  ✓ %s found\n', required_files{i});
    else
        fprintf('  ✗ %s NOT FOUND\n', required_files{i});
        all_files_exist = false;
    end
end

if ~all_files_exist
    error('Missing required data files. Please ensure all CSV files are in the current directory.');
end

fprintf('\nAll files found. Proceeding with analysis...\n');

%% Run Module 1: Main Drive Cycle Analysis
fprintf('\n========================================================================\n');
fprintf('RUNNING MODULE 1: MAIN DRIVE CYCLE ANALYSIS\n');
fprintf('========================================================================\n');
try
    ecogenium_main_analysis;
    fprintf('\n✓ Module 1 completed successfully.\n');
catch ME
    fprintf('\n✗ Module 1 failed with error:\n%s\n', ME.message);
end

%% Run Module 2: Comparative Race Analysis
fprintf('\n========================================================================\n');
fprintf('RUNNING MODULE 2: COMPARATIVE RACE ANALYSIS (v1.2)\n');
fprintf('========================================================================\n');
try
    ecogenium_comparative_analysis_1;
    fprintf('\n✓ Module 2 completed successfully.\n');
catch ME
    fprintf('\n✗ Module 2 failed with error:\n%s\n', ME.message);
end

%% Run Module 3: Energy and Power Analysis
fprintf('\n========================================================================\n');
fprintf('RUNNING MODULE 3: ENERGY AND POWER ANALYSIS (v2.0)\n');
fprintf('========================================================================\n');
try
    ecogenium_energy_analysis;
    fprintf('\n✓ Module 3 completed successfully.\n');
catch ME
    fprintf('\n✗ Module 3 failed with error:\n%s\n', ME.message);
end

%% Final Summary
fprintf('\n========================================================================\n');
fprintf('                    ALL MODULES COMPLETED                               \n');
fprintf('========================================================================\n');
fprintf('\nGenerated Figures:\n');
fprintf('  Figure 1-2: Canonical drive cycle analysis\n');
fprintf('  Figure 3: Comparative race analysis (with master lap highlight)\n');
fprintf('  Figure 4: Power demand curves (raw vs smoothed)\n');
fprintf('  Figure 5: Torque vs RPM analysis\n');
fprintf('\nYou can now:\n');
fprintf('  - Review the figures\n');
fprintf('  - Export figures (File > Save As)\n');
fprintf('  - Modify parameters in individual scripts\n');
fprintf('  - Re-run specific modules\n');
fprintf('\n========================================================================\n');
fprintf('           Ready for 2026 Shell Eco-marathon preparation!              \n');
fprintf('========================================================================\n\n');
