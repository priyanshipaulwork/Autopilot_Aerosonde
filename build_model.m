function [P, trim] = build_model(force_rebuild)
%BUILD_MODEL Programmatically create the UAV_Autopilot Simulink model.
%   [P,TRIM] = BUILD_MODEL() numerically trims the Aerosonde model, safely
%   rebuilds UAV_Autopilot.slx, and returns the simulation configuration.
%   BUILD_MODEL(FALSE) loads an existing model while refreshing its model
%   workspace and trim condition.

if nargin < 1
    force_rebuild = true;
end
project_root = fileparts(mfilename('fullpath'));
addpath(genpath(project_root));

P = aircraft_parameters();
trim = solve_trim(P);
P.trim = trim;
x0 = trim.x0;
wind_ned = P.sim.wind_ned;
print_trim(trim);

model_name = 'UAV_Autopilot';
model_file = fullfile(project_root, [model_name '.slx']);
if ~force_rebuild && isfile(model_file)
    load_system(model_file);
    populate_model_workspace(model_name, P, x0, wind_ned);
    return
end

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_file)
    delete(model_file);
end
load_system('simulink');
new_system(model_name);

set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', num2str(P.sim.dt, '%.8g'), ...
    'StopTime', num2str(P.sim.duration), ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'ReturnWorkspaceOutputs', 'on');

add_block('simulink/Sources/Clock', [model_name '/Simulation Time'], ...
    'Position', [35 70 65 90]);
add_block('simulink/Sources/Constant', [model_name '/Steady Wind NED'], ...
    'Value', 'wind_ned', 'Position', [35 430 100 460]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
    [model_name '/Commands and Guidance'], 'Position', [150 40 340 145]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
    [model_name '/Cascaded Autopilot'], 'Position', [415 50 605 190]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
    [model_name '/Actuator Limits'], 'Position', [665 90 790 145]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
    [model_name '/Nonlinear 6-DOF Aerosonde'], 'Position', [845 250 1055 350]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
    [model_name '/State and Air-Data Computation'], 'Position', [470 350 700 445]);
add_block('simulink/Signal Routing/Mux', [model_name '/Results Mux'], ...
    'Inputs', '5', 'DisplayOption', 'bar', 'Position', [1115 75 1120 350]);
add_block('simulink/Sinks/Out1', [model_name '/Results'], ...
    'Position', [1195 195 1225 215]);

build_commands_subsystem([model_name '/Commands and Guidance']);
build_autopilot_subsystem([model_name '/Cascaded Autopilot']);
build_actuator_subsystem([model_name '/Actuator Limits']);
build_plant_subsystem([model_name '/Nonlinear 6-DOF Aerosonde']);
build_airdata_subsystem([model_name '/State and Air-Data Computation']);

% Main architecture and explicit feedback paths.
add_line(model_name, 'Simulation Time/1', 'Commands and Guidance/3', 'autorouting', 'on');
add_line(model_name, 'Simulation Time/1', 'Cascaded Autopilot/4', 'autorouting', 'on');
add_line(model_name, 'Steady Wind NED/1', 'Nonlinear 6-DOF Aerosonde/2', 'autorouting', 'on');
add_line(model_name, 'Steady Wind NED/1', 'State and Air-Data Computation/2', 'autorouting', 'on');
add_line(model_name, 'Nonlinear 6-DOF Aerosonde/1', 'State and Air-Data Computation/1', 'autorouting', 'on');
add_line(model_name, 'Nonlinear 6-DOF Aerosonde/1', 'Commands and Guidance/1', 'autorouting', 'on');
add_line(model_name, 'Nonlinear 6-DOF Aerosonde/1', 'Cascaded Autopilot/2', 'autorouting', 'on');
add_line(model_name, 'Nonlinear 6-DOF Aerosonde/1', 'Results Mux/1', 'autorouting', 'on');
add_line(model_name, 'State and Air-Data Computation/1', 'Commands and Guidance/2', 'autorouting', 'on');
add_line(model_name, 'State and Air-Data Computation/1', 'Cascaded Autopilot/3', 'autorouting', 'on');
add_line(model_name, 'State and Air-Data Computation/1', 'Results Mux/2', 'autorouting', 'on');
add_line(model_name, 'Commands and Guidance/1', 'Cascaded Autopilot/1', 'autorouting', 'on');
add_line(model_name, 'Commands and Guidance/1', 'Results Mux/3', 'autorouting', 'on');
add_line(model_name, 'Cascaded Autopilot/1', 'Actuator Limits/1', 'autorouting', 'on');
add_line(model_name, 'Cascaded Autopilot/2', 'Results Mux/4', 'autorouting', 'on');
add_line(model_name, 'Actuator Limits/1', 'Nonlinear 6-DOF Aerosonde/1', 'autorouting', 'on');
add_line(model_name, 'Actuator Limits/1', 'Results Mux/5', 'autorouting', 'on');
logged_line = add_line(model_name, 'Results Mux/1', 'Results/1', 'autorouting', 'on');
set_param(logged_line, 'Name', 'results');
Simulink.sdi.markSignalForStreaming([model_name '/Results Mux'], 1, 'on');

annotation = Simulink.Annotation(model_name, sprintf([ ...
    'Cascaded PID Aerosonde Autopilot\n' ...
    'Signal order: Guidance -> Course/Altitude/Airspeed -> Attitude -> Actuators -> 6-DOF Plant']));
annotation.Position = [380 485 830 535];

populate_model_workspace(model_name, P, x0, wind_ned);
save_system(model_name, model_file);
fprintf('Created %s\n', model_file);
end

function build_commands_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/State'], ...
    'Port', '1', 'Position', [25 35 55 55]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Air Data'], ...
    'Port', '2', 'Position', [25 90 55 110]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Time'], ...
    'Port', '3', 'Position', [25 145 55 165]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Feedback Mux'], ...
    'Inputs', '3', 'Position', [95 35 100 165]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Scenario or Waypoint Guidance'], ...
    'MATLABFcn', 'command_guidance(u(20),u(1),u(2),-u(3),u(16),P)', ...
    'OutputDimensions', '4', 'Position', [150 70 365 130]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Commands'], ...
    'Port', '1', 'Position', [415 90 445 110]);
add_line(subsystem, 'State/1', 'Feedback Mux/1');
add_line(subsystem, 'Air Data/1', 'Feedback Mux/2');
add_line(subsystem, 'Time/1', 'Feedback Mux/3');
add_line(subsystem, 'Feedback Mux/1', 'Scenario or Waypoint Guidance/1');
add_line(subsystem, 'Scenario or Waypoint Guidance/1', 'Commands/1');
end

function build_autopilot_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Commands'], ...
    'Port', '1', 'Position', [20 30 50 50]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/State'], ...
    'Port', '2', 'Position', [20 75 50 95]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Air Data'], ...
    'Port', '3', 'Position', [20 120 50 140]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Time'], ...
    'Port', '4', 'Position', [20 165 50 185]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Controller Inputs'], ...
    'Inputs', '4', 'Position', [85 30 90 185]);
add_block('simulink/Ports & Subsystems/Subsystem', [subsystem '/Lateral Channel'], ...
    'Position', [145 25 345 105]);
add_block('simulink/Ports & Subsystems/Subsystem', [subsystem '/Longitudinal Channel'], ...
    'Position', [145 125 345 225]);
build_lateral_subsystem([subsystem '/Lateral Channel']);
build_longitudinal_subsystem([subsystem '/Longitudinal Channel']);
add_block('simulink/Sources/Constant', [subsystem '/Rudder Zero'], ...
    'Value', '0', 'Position', [390 145 425 170]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Control Mux'], ...
    'Inputs', '4', 'Position', [485 45 490 190]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Commanded Attitudes'], ...
    'Inputs', '2', 'Position', [485 225 490 285]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Raw Controls'], ...
    'Port', '1', 'Position', [545 105 575 125]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Attitude Commands'], ...
    'Port', '2', 'Position', [545 245 575 265]);
add_line(subsystem, 'Commands/1', 'Controller Inputs/1');
add_line(subsystem, 'State/1', 'Controller Inputs/2');
add_line(subsystem, 'Air Data/1', 'Controller Inputs/3');
add_line(subsystem, 'Time/1', 'Controller Inputs/4');
add_line(subsystem, 'Controller Inputs/1', 'Lateral Channel/1');
add_line(subsystem, 'Controller Inputs/1', 'Longitudinal Channel/1');
% Control order: elevator, aileron, rudder, throttle.
add_line(subsystem, 'Longitudinal Channel/1', 'Control Mux/1');
add_line(subsystem, 'Lateral Channel/1', 'Control Mux/2');
add_line(subsystem, 'Rudder Zero/1', 'Control Mux/3');
add_line(subsystem, 'Longitudinal Channel/2', 'Control Mux/4');
add_line(subsystem, 'Lateral Channel/2', 'Commanded Attitudes/1');
add_line(subsystem, 'Longitudinal Channel/3', 'Commanded Attitudes/2');
add_line(subsystem, 'Control Mux/1', 'Raw Controls/1');
add_line(subsystem, 'Commanded Attitudes/1', 'Attitude Commands/1');
end

function build_lateral_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Packed Inputs'], ...
    'Position', [20 75 50 95]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Course PI'], ...
    'MATLABFcn', ['course_controller(u(3),u(20),P.sim.dt,P,' ...
                  'u(24)<=P.sim.dt/2,u(24))'], ...
    'OutputDimensions', '1', 'Position', [90 35 245 75]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Roll Inputs'], ...
    'Inputs', '2', 'Position', [285 55 290 130]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Roll PI-D'], ...
    'MATLABFcn', ['roll_controller(u(1),u(12),u(15),P.sim.dt,P,' ...
                  'u(25)<=P.sim.dt/2,u(25))'], ...
    'OutputDimensions', '1', 'Position', [330 70 475 110]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Aileron'], ...
    'Port', '1', 'Position', [530 80 560 100]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Bank Command'], ...
    'Port', '2', 'Position', [530 25 560 45]);
add_line(subsystem, 'Packed Inputs/1', 'Course PI/1');
add_line(subsystem, 'Course PI/1', 'Roll Inputs/1');
add_line(subsystem, 'Packed Inputs/1', 'Roll Inputs/2');
add_line(subsystem, 'Roll Inputs/1', 'Roll PI-D/1');
add_line(subsystem, 'Roll PI-D/1', 'Aileron/1');
add_line(subsystem, 'Course PI/1', 'Bank Command/1');
end

function build_longitudinal_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Packed Inputs'], ...
    'Position', [20 105 50 125]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Altitude PI'], ...
    'MATLABFcn', ['altitude_controller(u(1),-u(7),P.sim.dt,P,' ...
                  'u(24)<=P.sim.dt/2,u(24))'], ...
    'OutputDimensions', '1', 'Position', [90 25 245 65]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Pitch Inputs'], ...
    'Inputs', '2', 'Position', [280 45 285 115]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Pitch PI-D'], ...
    'MATLABFcn', ['pitch_controller(u(1),u(13),u(16),P.sim.dt,P,' ...
                  'u(25)<=P.sim.dt/2,u(25))'], ...
    'OutputDimensions', '1', 'Position', [325 55 475 95]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Airspeed PI'], ...
    'MATLABFcn', ['airspeed_controller(u(2),u(17),P.sim.dt,P,' ...
                  'u(24)<=P.sim.dt/2,u(24))'], ...
    'OutputDimensions', '1', 'Position', [90 150 245 190]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Elevator'], ...
    'Port', '1', 'Position', [535 65 565 85]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Throttle'], ...
    'Port', '2', 'Position', [535 160 565 180]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Pitch Command'], ...
    'Port', '3', 'Position', [535 15 565 35]);
add_line(subsystem, 'Packed Inputs/1', 'Altitude PI/1');
add_line(subsystem, 'Altitude PI/1', 'Pitch Inputs/1');
add_line(subsystem, 'Packed Inputs/1', 'Pitch Inputs/2');
add_line(subsystem, 'Pitch Inputs/1', 'Pitch PI-D/1');
add_line(subsystem, 'Packed Inputs/1', 'Airspeed PI/1');
add_line(subsystem, 'Pitch PI-D/1', 'Elevator/1');
add_line(subsystem, 'Airspeed PI/1', 'Throttle/1');
add_line(subsystem, 'Altitude PI/1', 'Pitch Command/1');
end

function build_actuator_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Raw Controls'], ...
    'Position', [20 60 50 80]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Surface and Throttle Saturation'], ...
    'MATLABFcn', 'limit_actuators(u,P)', 'OutputDimensions', '4', ...
    'Position', [95 45 300 95]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Limited Controls'], ...
    'Position', [350 60 380 80]);
add_line(subsystem, 'Raw Controls/1', 'Surface and Throttle Saturation/1');
add_line(subsystem, 'Surface and Throttle Saturation/1', 'Limited Controls/1');
end

function build_plant_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Controls'], ...
    'Port', '1', 'Position', [20 60 50 80]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Wind NED'], ...
    'Port', '2', 'Position', [20 120 50 140]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Dynamics Inputs'], ...
    'Inputs', '3', 'Position', [140 45 145 170]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Forces Moments and Rigid Body'], ...
    'MATLABFcn', 'aircraft_dynamics(u(1:12),u(13:16),u(17:19),P)', ...
    'OutputDimensions', '12', 'Position', [200 75 425 125]);
add_block('simulink/Continuous/Integrator', [subsystem '/State Integrator'], ...
    'InitialCondition', 'x0', 'Position', [485 80 520 120]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/State'], ...
    'Port', '1', 'Position', [580 90 610 110]);
add_line(subsystem, 'State Integrator/1', 'Dynamics Inputs/1', 'autorouting', 'on');
add_line(subsystem, 'Controls/1', 'Dynamics Inputs/2', 'autorouting', 'on');
add_line(subsystem, 'Wind NED/1', 'Dynamics Inputs/3', 'autorouting', 'on');
add_line(subsystem, 'Dynamics Inputs/1', 'Forces Moments and Rigid Body/1');
add_line(subsystem, 'Forces Moments and Rigid Body/1', 'State Integrator/1');
add_line(subsystem, 'State Integrator/1', 'State/1');
end

function build_airdata_subsystem(subsystem)
Simulink.SubSystem.deleteContents(subsystem);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/State'], ...
    'Port', '1', 'Position', [20 45 50 65]);
add_block('simulink/Ports & Subsystems/In1', [subsystem '/Wind NED'], ...
    'Port', '2', 'Position', [20 105 50 125]);
add_block('simulink/Signal Routing/Mux', [subsystem '/Air Data Inputs'], ...
    'Inputs', '2', 'Position', [100 40 105 130]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [subsystem '/Wind Relative Air Data'], ...
    'MATLABFcn', 'air_data(u(1:12),u(13:15))', ...
    'OutputDimensions', '7', 'Position', [160 60 350 110]);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/Air Data'], ...
    'Port', '1', 'Position', [400 75 430 95]);
add_line(subsystem, 'State/1', 'Air Data Inputs/1');
add_line(subsystem, 'Wind NED/1', 'Air Data Inputs/2');
add_line(subsystem, 'Air Data Inputs/1', 'Wind Relative Air Data/1');
add_line(subsystem, 'Wind Relative Air Data/1', 'Air Data/1');
end

function populate_model_workspace(model_name, P, x0, wind_ned)
workspace = get_param(model_name, 'ModelWorkspace');
assignin(workspace, 'P', P);
assignin(workspace, 'x0', x0);
assignin(workspace, 'wind_ned', wind_ned);
% Interpreted MATLAB Function blocks evaluate names in the MATLAB base
% workspace, while Integrator/Constant parameters use the model workspace.
% Keep the three run-specific values synchronized in both workspaces.
assignin('base', 'P', P);
assignin('base', 'x0', x0);
assignin('base', 'wind_ned', wind_ned);
end

function trim = solve_trim(P)
% Four-variable, straight-and-level trim using only base MATLAB fminsearch.
initial_guess = [deg2rad(3); deg2rad(3); deg2rad(-3); 0.35];
options = optimset('Display', 'off', 'MaxIter', 2000, 'MaxFunEvals', 6000, ...
    'TolX', 1e-10, 'TolFun', 1e-12);
[solution, cost, exitflag] = fminsearch(@(z) trim_cost(z, P), initial_guess, options);

alpha = solution(1); theta = solution(2);
delta_e = solution(3); delta_t = solution(4);
x0 = [0; 0; -P.sim.h0; P.trim.Va*cos(alpha); 0; P.trim.Va*sin(alpha); ...
      0; theta; 0; 0; 0; 0];
delta = [delta_e; 0; 0; delta_t];
xdot = aircraft_dynamics(x0, delta, [0;0;0], P);
residual_vector = [xdot(3); xdot(4); xdot(6); xdot(11)];

trim.Va = P.trim.Va;
trim.h = P.sim.h0;
trim.alpha = alpha;
trim.theta = theta;
trim.delta_e = delta_e;
trim.delta_t = delta_t;
trim.x0 = x0;
trim.residual = norm(residual_vector);
trim.cost = cost;
trim.exitflag = exitflag;
if exitflag <= 0 || delta_t < 0 || delta_t > 1 || ...
        abs(delta_e) > P.limits.delta_e || trim.residual > 0.05
    error('Trim solve failed physical/residual checks (cost %.3g, residual %.3g).', ...
        cost, trim.residual);
end
end

function cost = trim_cost(z, P)
alpha = z(1); theta = z(2); delta_e = z(3); delta_t = z(4);
x = [0; 0; -P.sim.h0; P.trim.Va*cos(alpha); 0; P.trim.Va*sin(alpha); ...
     0; theta; 0; 0; 0; 0];
xdot = aircraft_dynamics(x, [delta_e;0;0;delta_t], [0;0;0], P);
scaled_residual = [xdot(4); xdot(6); 4*xdot(11); 2*xdot(3); 5*(theta-alpha)];
penalty = 1e4*(max(0, abs(alpha)-deg2rad(20))^2 ...
              + max(0, abs(theta)-deg2rad(20))^2 ...
              + max(0, abs(delta_e)-P.limits.delta_e)^2 ...
              + max(0, -delta_t)^2 + max(0, delta_t-1)^2);
cost = scaled_residual.'*scaled_residual + penalty;
end

function print_trim(trim)
fprintf('\nAEROSONDE TRIM\n');
fprintf('  Va        = %8.4f m/s\n', trim.Va);
fprintf('  alpha     = %8.4f deg\n', rad2deg(trim.alpha));
fprintf('  theta     = %8.4f deg\n', rad2deg(trim.theta));
fprintf('  elevator  = %8.4f deg\n', rad2deg(trim.delta_e));
fprintf('  throttle  = %8.4f\n', trim.delta_t);
fprintf('  residual  = %8.3e\n\n', trim.residual);
end
