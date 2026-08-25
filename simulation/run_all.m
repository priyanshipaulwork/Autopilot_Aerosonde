%RUN_ALL Rebuild and execute all cases, figures, metrics, and acceptance tests.
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
build_model(true);
controllerProfile = 'manual';
if exist('autotuned_gains','file') == 2
    generatedGains = autotuned_gains();
    if isfield(generatedGains,'generated') && generatedGains.generated ...
            && isfield(generatedGains,'accepted') && generatedGains.accepted
        controllerProfile = 'autotuned';
    end
end
fprintf('Standard validation profile: %s\n',upper(controllerProfile));
run(fullfile(project_root,'simulation','run_nominal.m'));
run(fullfile(project_root,'simulation','run_wind_case.m'));
run(fullfile(project_root,'simulation','run_waypoint_case.m'));
