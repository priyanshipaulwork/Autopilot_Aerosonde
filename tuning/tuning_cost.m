function [J, components] = tuning_cost(loop_name, data, P)
%TUNING_COST Dimensionless tracking, effort, saturation, and safety cost.
%   J = TUNING_COST(LOOP_NAME,DATA,P) returns a large finite penalty for
%   failed or unstable candidates so fminsearch can continue safely.

components = struct('tracking',0,'final',0,'effort',0, ...
    'saturation',0,'overshoot',0,'instability',0);
if isempty(data) || ~isfield(data,'time')
    J = 1e6; components.instability = J; return
end
required = [data.h;data.Va;data.phi;data.theta;data.chi; ...
    data.delta_e;data.delta_a;data.delta_t];
if any(~isfinite(required))
    J = 1e6; components.instability = J; return
end

if max(abs(data.phi)) > deg2rad(60) || max(abs(data.theta)) > deg2rad(40) ...
        || min(data.h) < 20 || max(data.h) > 250 ...
        || min(data.Va) < 12 || max(data.Va) > 45
    J = 1e6 + 1e3*(max(abs(data.phi))+max(abs(data.theta)));
    components.instability = J;
    return
end

evaluation = data.time >= 5;
final_window = data.time >= data.time(end)-3;
elevator_sat = mean(abs(data.delta_e) >= 0.98*P.limits.delta_e);
aileron_sat = mean(abs(data.delta_a) >= 0.98*P.limits.delta_a);

switch lower(string(loop_name))
    case "roll"
        scale = deg2rad(5);
        tracking_error = wrap_angle(data.phi_cmd-data.phi);
        components.tracking = mean((tracking_error(evaluation)/scale).^2);
        components.final = (mean(tracking_error(final_window))/scale)^2;
        components.effort = mean((data.delta_a/P.limits.delta_a).^2);
        components.saturation = aileron_sat;
        components.overshoot = (max(0,max(abs(data.phi))-max(abs(data.phi_cmd)))/scale)^2;
        weights = [1.0 0.45 0.08 2.5 0.20];
    case "pitch"
        scale = deg2rad(3);
        tracking_error = data.theta_cmd-data.theta;
        components.tracking = mean((tracking_error(evaluation)/scale).^2);
        components.final = (mean(tracking_error(final_window))/scale)^2;
        components.effort = mean(((data.delta_e-P.trim.delta_e)/P.limits.delta_e).^2);
        components.saturation = elevator_sat;
        components.overshoot = (max(0,max(abs(data.theta))-max(abs(data.theta_cmd)))/scale)^2;
        weights = [1.0 0.45 0.08 2.5 0.20];
    case "course"
        scale = deg2rad(5);
        tracking_error = wrap_angle(data.chi_cmd-data.chi);
        components.tracking = mean((tracking_error(evaluation)/scale).^2);
        components.final = (mean(tracking_error(final_window))/scale)^2;
        components.effort = 0.6*mean((data.phi_cmd/P.limits.phi_cmd).^2) ...
            + 0.4*mean((data.delta_a/P.limits.delta_a).^2);
        components.saturation = aileron_sat;
        components.overshoot = (max(0,max(data.chi)-max(data.chi_cmd))/scale)^2;
        weights = [1.0 0.50 0.08 2.5 0.20];
    case "altitude"
        scale = 5;
        tracking_error = data.h_cmd-data.h;
        components.tracking = mean((tracking_error(evaluation)/scale).^2);
        components.final = (mean(tracking_error(final_window))/scale)^2;
        components.effort = 0.5*mean((data.theta_cmd/P.limits.theta_cmd).^2) ...
            + 0.5*mean(((data.delta_e-P.trim.delta_e)/P.limits.delta_e).^2);
        components.saturation = elevator_sat;
        components.overshoot = (max(0,max(data.h)-max(data.h_cmd))/scale)^2;
        weights = [1.0 0.50 0.08 2.5 0.20];
    case "airspeed"
        scale = 1;
        tracking_error = data.Va_cmd-data.Va;
        components.tracking = mean((tracking_error(evaluation)/scale).^2);
        components.final = (mean(tracking_error(final_window))/scale)^2;
        components.effort = mean((data.delta_t-P.trim.delta_t).^2);
        low_sat = mean(data.delta_t <= P.limits.delta_t_min+0.01);
        high_sat = mean(data.delta_t >= P.limits.delta_t_max-0.01);
        components.saturation = low_sat+high_sat;
        components.overshoot = (max(0,max(data.Va)-max(data.Va_cmd))/scale)^2;
        weights = [1.0 0.50 0.12 3.0 0.20];
    otherwise
        error('Unknown tuning loop "%s".',loop_name);
end

if components.saturation > 0.10
    components.instability = 50*(components.saturation-0.10)^2;
end
values = [components.tracking components.final components.effort ...
          components.saturation components.overshoot];
J = weights*values.' + components.instability;
end

function angle = wrap_angle(angle)
angle = atan2(sin(angle),cos(angle));
end
