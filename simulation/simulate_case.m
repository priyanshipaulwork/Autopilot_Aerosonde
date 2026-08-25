function [data, metrics, P, trim] = simulate_case(scenario, profile, print_metrics, P, trim, close_model)
%SIMULATE_CASE Configure, run, extract, and validate one deterministic case.
%   SIMULATE_CASE(SCENARIO,PROFILE,PRINT_METRICS,P,TRIM,CLOSE_MODEL)
%   optionally reuses a prepared parameter/trim pair. PROFILE is 'manual',
%   'autotuned', or 'active'. CLOSE_MODEL defaults true; the tuner sets it
%   false only while deliberately retaining one prepared model instance.
%   Controller and waypoint persistent states are always reset.

project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
model_file = fullfile(project_root, 'UAV_Autopilot.slx');
if nargin < 2 || isempty(profile), profile = 'manual'; end
if nargin < 3 || isempty(print_metrics), print_metrics = true; end
if nargin < 6 || isempty(close_model), close_model = true; end
if nargin < 5 || isempty(P) || isempty(trim)
    [P, trim] = build_model(~isfile(model_file));
end
if ~strcmpi(profile,'active')
    P = apply_gain_profile(P,profile);
end
P.trim = trim;

P.sim.scenario = lower(scenario);
switch P.sim.scenario
    case 'nominal'
        P.sim.duration = 120;
        P.sim.wind_ned = [0;0;0];
    case 'wind'
        P.sim.duration = 120;
        P.sim.wind_ned = [5;3;0];
    case 'waypoint'
        % The actual 4/4 capture occurs near 65 s. Stop just afterward so
        % final-distance metrics describe mission completion rather than
        % continued post-mission fixed-wing flight around the last point.
        P.sim.duration = 66;
        P.sim.wind_ned = [0;0;0];
    otherwise
        error('Unknown scenario "%s".', scenario);
end

% Preserve the trimmed air-relative velocity when initializing in wind.
x0 = trim.x0;
R_body_to_ned = body_to_ned(x0(7),x0(8),x0(9));
x0(4:6) = x0(4:6) + R_body_to_ned.'*P.sim.wind_ned;
wind_ned = P.sim.wind_ned;

clear roll_controller pitch_controller course_controller
clear altitude_controller airspeed_controller waypoint_guidance
model_name = 'UAV_Autopilot';
load_system(model_file);
workspace = get_param(model_name, 'ModelWorkspace');
assignin(workspace, 'P', P);
assignin(workspace, 'x0', x0);
assignin(workspace, 'wind_ned', wind_ned);
assignin('base', 'P', P);
assignin('base', 'x0', x0);
assignin('base', 'wind_ned', wind_ned);
set_param(model_name, 'StopTime', num2str(P.sim.duration), ...
    'FixedStep', num2str(P.sim.dt, '%.8g'));

if print_metrics
    fprintf('Running %s case (%s profile) for %.0f s with wind NED [%g %g %g] m/s ...\n', ...
        P.sim.scenario, P.control.profile, P.sim.duration, wind_ned(1), wind_ned(2), wind_ned(3));
end
simulation_output = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
data = extract_results(simulation_output, P.sim.scenario, P);
metrics = compute_metrics(data, P, print_metrics);
if ~metrics.finite
    error('%s simulation produced NaN or Inf values.', P.sim.scenario);
end
if metrics.max_abs_elevator_deg > 25.001 || metrics.max_abs_aileron_deg > 25.001 ...
        || metrics.min_throttle < -1e-9 || metrics.max_throttle > 1+1e-9
    error('%s simulation violated an actuator constraint.', P.sim.scenario);
end
if close_model
    % P, x0, wind, and run-time solver settings are intentionally transient.
    % Explicitly discard them so batch MATLAB never prompts to write
    % scenario-specific values back into the generated model.
    close_system(model_name,0);
end
end

function R = body_to_ned(phi, theta, psi)
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi; ...
     cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi; ...
     -sth,     sphi*cth,                   cphi*cth];
end
