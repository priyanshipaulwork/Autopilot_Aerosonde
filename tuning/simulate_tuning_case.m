function data = simulate_tuning_case(loop_name, P, trim)
%SIMULATE_TUNING_CASE Run one short nonlinear Simulink tuning maneuver.
%   DATA = SIMULATE_TUNING_CASE(LOOP_NAME,P,TRIM) reuses the built model
%   and trim state. It performs no model reconstruction, trim solve,
%   metrics printing, plotting, or filesystem writes.

loop_name = char(lower(string(loop_name)));
scenario = ['tune_' loop_name];
durations = struct('roll',25,'pitch',30,'course',35,'altitude',40,'airspeed',30);
if ~isfield(durations,loop_name)
    error('Unknown tuning loop "%s".',loop_name);
end

P.sim.scenario = scenario;
P.sim.duration = durations.(loop_name);
P.sim.dt = P.tuning.dt;
P.sim.wind_ned = [0;0;0];
P.trim = trim;
x0 = trim.x0;
wind_ned = P.sim.wind_ned;

clear roll_controller pitch_controller course_controller
clear altitude_controller airspeed_controller waypoint_guidance
model_name = 'UAV_Autopilot';
model_file = fullfile(fileparts(fileparts(mfilename('fullpath'))),[model_name '.slx']);
if ~bdIsLoaded(model_name), load_system(model_file); end
workspace = get_param(model_name,'ModelWorkspace');
assignin(workspace,'P',P); assignin(workspace,'x0',x0); assignin(workspace,'wind_ned',wind_ned);
assignin('base','P',P); assignin('base','x0',x0); assignin('base','wind_ned',wind_ned);
stop_time = num2str(P.sim.duration); fixed_step = num2str(P.sim.dt,'%.8g');
if ~strcmp(get_param(model_name,'StopTime'),stop_time)
    set_param(model_name,'StopTime',stop_time);
end
if ~strcmp(get_param(model_name,'FixedStep'),fixed_step)
    set_param(model_name,'FixedStep',fixed_step);
end
simulation_output = sim(model_name,'ReturnWorkspaceOutputs','on');
data = extract_results(simulation_output,char(scenario),P);
end
