%RUN_WAYPOINT_CASE Fly the four-waypoint rectangular mission.
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
if ~exist('controllerProfile','var'), controllerProfile = 'manual'; end
[waypointData, waypointMetrics, P, trim] = simulate_case('waypoint',controllerProfile);
save(fullfile(project_root,'results','waypoint_results.mat'), ...
    'waypointData','waypointMetrics','trim','controllerProfile');

nominal_file = fullfile(project_root,'results','nominal_results.mat');
wind_file = fullfile(project_root,'results','wind_results.mat');
nominalData = []; windData = [];
if isfile(nominal_file)
    saved_nominal = load(nominal_file,'nominalData','controllerProfile');
    if isfield(saved_nominal,'controllerProfile') ...
            && strcmpi(saved_nominal.controllerProfile,controllerProfile)
        nominalData = saved_nominal.nominalData;
    end
end
if isfile(wind_file)
    saved_wind = load(wind_file,'windData','controllerProfile');
    if isfield(saved_wind,'controllerProfile') ...
            && strcmpi(saved_wind.controllerProfile,controllerProfile)
        windData = saved_wind.windData;
    end
end
% Wind-case execution already creates the four tracking/control figures.
% The waypoint case refreshes only its trajectory figure.
plot_results([], [], waypointData, P);
if ~isempty(nominalData) && ~isempty(windData)
    projectPassed = print_validation_summary(P, trim, nominalData, windData, waypointData);
end
