%AUTOTUNE_ALL Sequential simulation-based PID gain optimization workflow.
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
results_dir = fullfile(project_root,'results');
if ~isfolder(results_dir), mkdir(results_dir); end

fprintf('\n============================================================\n');
fprintf(' SIMULATION-BASED PID GAIN OPTIMIZATION\n');
fprintf(' Aerosonde Cascaded Autopilot\n');
fprintf('============================================================\n');
[P,trim] = build_model(~isfile(fullfile(project_root,'UAV_Autopilot.slx')));
P = apply_gain_profile(P,'manual');

fprintf('\nRecording fresh full manual baseline before optimization ...\n');
[comparison.manual,comparison.metrics.manual] = ...
    run_profile_cases(P,trim,'active',false,true);
manualBaseline = comparison.manual;
manualBaselineMetrics = comparison.metrics.manual;
save(fullfile(results_dir,'manual_baseline.mat'),'manualBaseline','manualBaselineMetrics','trim');

history = struct();
[P,history.roll] = tune_roll(P,trim); print_loop_result(history.roll);
[P,history.pitch] = tune_pitch(P,trim); print_loop_result(history.pitch);
[P,history.course] = tune_course(P,trim); print_loop_result(history.course);
[P,history.altitude] = tune_altitude(P,trim); print_loop_result(history.altitude);
[P,history.airspeed] = tune_airspeed(P,trim); print_loop_result(history.airspeed);
P.control.profile = 'autotuned';

% Write a generated candidate before calling profile-based utilities.
save_autotuned_gains(P,false);
fprintf('\nRunning full candidate validation with one unchanged gain set ...\n');
[comparison.autotuned,comparison.metrics.autotuned] = run_profile_cases(P,trim,'active',false);
[autotuningAccepted,comparison.summary,comparison.metric_table] = ...
    evaluate_autotuning(comparison,P,true);
comparison.accepted = autotuningAccepted;
save_autotuned_gains(P,autotuningAccepted);
save(fullfile(results_dir,'comparison_results.mat'),'comparison','trim');
[gainComparisonTable,tuningSummaryTable] = export_tuning_results(P,history);
plot_tuning_convergence(history);
plot_comparison(comparison,P);

if autotuningAccepted
    finalCases = comparison.autotuned;
    finalProfile = 'autotuned';
else
    finalCases = comparison.manual;
    finalProfile = 'manual';
end
Pfinal = apply_gain_profile(P,finalProfile);
plot_results(finalCases.nominal,finalCases.wind,finalCases.waypoint,Pfinal);
plot_3d_trajectory(finalCases.waypoint,Pfinal);
animationPassed = false; animationPath = '';
try
    animationPath = animate_flight(finalCases.waypoint,Pfinal,5);
    animationPassed = isfile(animationPath) && dir(animationPath).bytes > 0;
catch animationError
    warning(animationError.identifier,'%s',animationError.message);
end
save(fullfile(results_dir,'comparison_results.mat'),'comparison','trim', ...
    'autotuningAccepted','gainComparisonTable','tuningSummaryTable', ...
    'animationPassed','animationPath','-append');
print_extended_summary(history,comparison,animationPassed,results_dir);

function print_loop_result(record)
fprintf('\n%s CONTROLLER\n',upper(record.loop));
fprintf('  Manual gains    : %s\n',mat2str(record.manual_gains,6));
fprintf('  Optimized gains : %s\n',mat2str(record.optimized_gains,6));
fprintf('  Initial cost    : %.6g\n',record.initial_cost);
fprintf('  Final cost      : %.6g\n',record.final_cost);
fprintf('  Improvement     : %.2f %%\n',record.improvement_percent);
end

function print_extended_summary(history,c,animation_passed,results_dir)
m=c.metrics.manual; a=c.metrics.autotuned;
fprintf('\n================================================================\n');
fprintf(' AEROSONDE CASCADED PID AUTOPILOT - EXTENDED PROJECT VALIDATION\n');
fprintf('================================================================\n');
loops={'roll','pitch','course','altitude','airspeed'};
fprintf('\nMANUAL BASELINE GAINS / AUTOTUNED GAINS\n');
for i=1:numel(loops)
    fprintf('%-10s : %-25s -> %s\n',capitalize(loops{i}), ...
        mat2str(history.(loops{i}).manual_gains,6),mat2str(history.(loops{i}).optimized_gains,6));
end
fprintf('\nNOMINAL PERFORMANCE              Manual    Autotuned\n');
fprintf('Altitude RMSE [m]               %8.3f   %8.3f\n',m.nominal.altitude_rmse_m,a.nominal.altitude_rmse_m);
fprintf('Airspeed RMSE [m/s]             %8.3f   %8.3f\n',m.nominal.airspeed_rmse_mps,a.nominal.airspeed_rmse_mps);
fprintf('Course RMSE [deg]               %8.3f   %8.3f\n',m.nominal.course_rmse_deg,a.nominal.course_rmse_deg);
fprintf('Elevator RMS [deg]              %8.3f   %8.3f\n',m.nominal.elevator_rms_deg,a.nominal.elevator_rms_deg);
fprintf('Aileron RMS [deg]               %8.3f   %8.3f\n',m.nominal.aileron_rms_deg,a.nominal.aileron_rms_deg);
fprintf('Elevator saturation [%%]         %8.3f   %8.3f\n',100*m.nominal.elevator_sat_fraction,100*a.nominal.elevator_sat_fraction);
fprintf('Aileron saturation [%%]          %8.3f   %8.3f\n',100*m.nominal.aileron_sat_fraction,100*a.nominal.aileron_sat_fraction);
fprintf('\nWIND RMSE (h / Va / chi)         %.3f / %.3f / %.3f   %.3f / %.3f / %.3f\n', ...
    m.wind.altitude_rmse_m,m.wind.airspeed_rmse_mps,m.wind.course_rmse_deg, ...
    a.wind.altitude_rmse_m,a.wind.airspeed_rmse_mps,a.wind.course_rmse_deg);
fprintf('WAYPOINTS / FINAL ERROR [m]      %d/4 / %.3f       %d/4 / %.3f\n', ...
    m.waypoint.waypoints_reached,m.waypoint.final_waypoint_distance_m, ...
    a.waypoint.waypoints_reached,a.waypoint.final_waypoint_distance_m);
visuals={'manual_vs_autotuned.png','autotuning_convergence.png','gain_comparison.png', ...
    'trajectory_3d.png','performance_dashboard.png'};
fprintf('\nVISUAL OUTPUTS\n');
for i=1:numel(visuals)
    status=isfile(fullfile(results_dir,visuals{i}));
    fprintf('%-32s : %s\n',visuals{i},pass_text(status));
end
fprintf('%-32s : %s\n','Flight animation',pass_text(animation_passed));
if c.accepted, result='ACCEPTED'; else, result='REJECTED'; end
fprintf('\nAUTOTUNING RESULT: %s\n',result);
all_visuals=all(cellfun(@(f)isfile(fullfile(results_dir,f)),visuals)) && animation_passed;
if all_visuals, validation='PASS'; else, validation='NEEDS ATTENTION'; end
fprintf('FINAL PROJECT VALIDATION: %s\n',validation);
fprintf('================================================================\n');
end

function text=pass_text(value)
if value, text='PASS'; else, text='FAIL'; end
end

function text=capitalize(text)
text(1)=upper(text(1));
end
