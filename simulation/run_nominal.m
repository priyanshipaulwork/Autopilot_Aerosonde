%RUN_NOMINAL Run deterministic altitude, airspeed, and course commands.
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
if ~exist('controllerProfile','var'), controllerProfile = 'manual'; end
[nominalData, nominalMetrics, P, trim] = simulate_case('nominal',controllerProfile);
if ~isfolder(fullfile(project_root,'results')), mkdir(fullfile(project_root,'results')); end
save(fullfile(project_root,'results','nominal_results.mat'), ...
    'nominalData','nominalMetrics','trim','controllerProfile');
plot_results(nominalData, [], [], P);
