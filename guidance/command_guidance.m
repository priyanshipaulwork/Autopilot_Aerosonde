function commands = command_guidance(time, pn, pe, h, chi, P)
%COMMAND_GUIDANCE Generate deterministic scenario commands for Simulink.
%   COMMANDS = COMMAND_GUIDANCE(TIME,PN,PE,H,CHI,P) returns
%   [h_cmd; Va_cmd; chi_cmd; active_waypoint_index]. Positions and altitude
%   are metres; course is radians. H and CHI document the feedback context.

reset = time <= P.sim.dt/2;
switch lower(P.sim.scenario)
    case 'waypoint'
        [chi_cmd, h_cmd, active_index] = waypoint_guidance( ...
            pn, pe, P.sim.waypoints, P.sim.waypoint_radius, reset);
    case 'tune_roll'
        h_cmd = P.sim.h0;
        chi_cmd = deg2rad(20)*(time >= 5);
        active_index = 0;
    case 'tune_pitch'
        h_cmd = P.sim.h0 + 10*(time >= 5);
        chi_cmd = 0;
        active_index = 0;
    case 'tune_course'
        h_cmd = P.sim.h0;
        chi_cmd = deg2rad(30)*(time >= 5);
        active_index = 0;
    case 'tune_altitude'
        h_cmd = P.sim.h0 + 20*(time >= 5);
        chi_cmd = 0;
        active_index = 0;
    case 'tune_airspeed'
        h_cmd = P.sim.h0;
        chi_cmd = 0;
        active_index = 0;
    otherwise
        h_cmd = P.sim.h0;
        if time >= 20
            h_cmd = 120;
        end
        chi_cmd = 0;
        if time >= 50
            chi_cmd = deg2rad(30);
        end
        active_index = 0;
end
Va_cmd = P.sim.Va_cmd;
if strcmpi(P.sim.scenario,'tune_airspeed')
    Va_cmd = P.sim.Va_cmd + 3*(time >= 5);
end
commands = [h_cmd; Va_cmd; chi_cmd; active_index];

% Explicitly acknowledge feedback values used by the subsystem interface.
if ~isfinite(h) || ~isfinite(chi)
    commands(:) = NaN;
end
end
