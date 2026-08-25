%RUN_WIND_CASE Evaluate unchanged gains in a [5 3 0] m/s steady NED wind.
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
nominal_file = fullfile(project_root,'results','nominal_results.mat');
if ~exist('controllerProfile','var'), controllerProfile = 'manual'; end
if isfile(nominal_file)
    saved_nominal = load(nominal_file,'nominalData','controllerProfile');
    saved_profile_matches = isfield(saved_nominal,'controllerProfile') ...
        && strcmpi(saved_nominal.controllerProfile,controllerProfile);
    if saved_profile_matches
        nominalData = saved_nominal.nominalData;
    else
        [nominalData, ~] = simulate_case('nominal',controllerProfile);
    end
else
    [nominalData, ~] = simulate_case('nominal',controllerProfile);
end
[windData, windMetrics, P, trim] = simulate_case('wind',controllerProfile);
save(fullfile(project_root,'results','wind_results.mat'), ...
    'windData','windMetrics','trim','controllerProfile');
plot_results(nominalData, windData, [], P);
