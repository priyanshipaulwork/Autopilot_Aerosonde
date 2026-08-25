function P = aircraft_parameters()
%AIRCRAFT_PARAMETERS Aerosonde reference data and autopilot configuration.
%   P = AIRCRAFT_PARAMETERS() returns one structure containing SI-unit
%   aircraft, aerodynamic, actuator, controller, and simulation parameters.

% Environment and rigid-body properties.
P.g    = 9.81;                  % gravitational acceleration [m/s^2]
P.mass = 13.5;                  % mass [kg]
P.Jx   = 0.8244;                % roll inertia [kg m^2]
P.Jy   = 1.135;                 % pitch inertia [kg m^2]
P.Jz   = 1.759;                 % yaw inertia [kg m^2]
P.Jxz  = 0.1204;                % cross inertia [kg m^2]

% Geometry and atmosphere.
P.S_wing   = 0.55;              % wing planform area [m^2]
P.b        = 2.8956;            % wingspan [m]
P.c        = 0.18994;           % mean aerodynamic chord [m]
P.S_prop   = 0.2027;            % propeller disk reference area [m^2]
P.rho      = 1.2682;            % air density [kg/m^3]
P.e_oswald = 0.9;               % Oswald efficiency factor [-]
P.AR       = P.b^2/P.S_wing;    % wing aspect ratio [-]

% Longitudinal aerodynamic coefficients.
P.C_L_0       = 0.28;
P.C_L_alpha   = 3.45;
P.C_L_q       = 0.0;
P.C_L_delta_e = -0.36;
P.C_D_0       = 0.03;
P.C_D_alpha   = 0.30;
P.C_D_p       = 0.0437;
P.C_D_q       = 0.0;
P.C_D_delta_e = 0.0;
P.C_m_0       = -0.02338;
P.C_m_alpha   = -0.38;
P.C_m_q       = -3.6;
P.C_m_delta_e = -0.5;

% Lateral-directional aerodynamic coefficients.
P.C_Y_0       = 0.0;
P.C_Y_beta    = -0.98;
P.C_Y_p       = 0.0;
P.C_Y_r       = 0.0;
P.C_Y_delta_a = 0.0;
P.C_Y_delta_r = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0       = 0.0;
P.C_n_beta    = 0.25;
P.C_n_p       = 0.022;
P.C_n_r       = -0.35;
P.C_n_delta_a = 0.06;
P.C_n_delta_r = -0.032;

% Simplified propeller and nonlinear lift-blending data.
P.C_prop  = 1.0;
P.k_motor = 80;                 % idealized motor speed scale [m/s]
P.M       = 50;                 % lift blending sharpness [-]
P.epsilon = 0.1592;             % retained reference parameter [-]
P.alpha0  = 0.4712;             % stall transition angle [rad]

% Command and actuator constraints.
P.limits.delta_e = deg2rad(25);
P.limits.delta_a = deg2rad(25);
P.limits.delta_r = deg2rad(25);
P.limits.delta_t_min = 0;
P.limits.delta_t_max = 1;
P.limits.phi_cmd   = deg2rad(35);
P.limits.theta_cmd = deg2rad(20);

% Immutable manually tuned baseline. Inner attitude loops are intentionally
% faster than the outer course and altitude loops.
P.control.manual.roll = struct('Kp',1.05,'Ki',0.12,'Kd',0.18);
P.control.manual.pitch = struct('Kp',1.65,'Ki',0.10,'Kd',0.28);
P.control.manual.course = struct('Kp',1.20,'Ki',0.08);
P.control.manual.altitude = struct('Kp',0.020,'Ki',0.0015);
P.control.manual.airspeed = struct('Kp',0.070,'Ki',0.025);

% Backward-compatible active fields read by the five controller functions.
P.control.roll = P.control.manual.roll;
P.control.pitch = P.control.manual.pitch;
P.control.course = P.control.manual.course;
P.control.altitude = P.control.manual.altitude;
P.control.airspeed = P.control.manual.airspeed;
P.control.profile = 'manual';

% Default trim values are replaced by build_model's numerical trim solve.
P.trim.Va      = 25;            % trim airspeed [m/s]
P.trim.h       = 100;           % trim altitude [m]
P.trim.alpha   = deg2rad(3);
P.trim.theta   = deg2rad(3);
P.trim.delta_e = deg2rad(-3);
P.trim.delta_t = 0.35;

% Deterministic simulation setup.
P.sim.dt       = 0.01;          % fixed integration step [s]
P.sim.duration = 120;           % default stop time [s]
P.sim.scenario = 'nominal';
P.sim.wind_ned = [0; 0; 0];     % steady wind, NED axes [m/s]
P.sim.Va_cmd   = 25;            % commanded airspeed [m/s]
P.sim.h0       = 100;           % initial altitude [m]
P.sim.waypoint_radius = 40;     % horizontal capture radius [m]
P.sim.waypoints = [400,   0, 120; ...
                   400, 400, 120; ...
                     0, 400, 140; ...
                     0,   0, 120];

% Practical simulation-based tuning budget and bounded gain transform.
% Short maneuvers use a coarser step only during optimization; full
% validation continues to use P.sim.dt = 0.01 s.
P.tuning.dt = 0.02;
% The interpreted nonlinear model takes about 37 s for a cold 25 s
% maneuver on the validation machine. Fast Restart plus this bounded
% 24-evaluation budget keeps the complete five-loop study practical.
P.tuning.MaxIterations = 14;
P.tuning.MaxFunctionEvaluations = 24;
P.tuning.TolX = 1e-3;
P.tuning.TolFun = 1e-3;
P.tuning.lower_multiplier = struct('Kp',0.25,'Ki',0.10,'Kd',0.25);
P.tuning.upper_multiplier = struct('Kp',4.00,'Ki',5.00,'Kd',4.00);
end
