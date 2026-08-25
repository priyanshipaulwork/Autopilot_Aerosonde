%RUN_COMPARISON Run six full cases and generate quantitative comparisons.
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
[P,trim] = build_model(~isfile(fullfile(project_root,'UAV_Autopilot.slx')));
[comparison,autotuningAccepted] = run_comparison_profiles(P,trim,true);
plot_comparison(comparison,P);
if autotuningAccepted
    finalWaypointData = comparison.autotuned.waypoint;
else
    finalWaypointData = comparison.manual.waypoint;
end
plot_3d_trajectory(finalWaypointData,P);

