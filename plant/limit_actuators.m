function delta_limited = limit_actuators(delta, P)
%LIMIT_ACTUATORS Apply elevator, aileron, rudder, and throttle limits.
%   DELTA_LIMITED = LIMIT_ACTUATORS(DELTA,P) accepts commands ordered as
%   [delta_e; delta_a; delta_r; delta_t]. Surface angles are in radians.

delta = delta(:);
delta_limited = [saturate(delta(1), -P.limits.delta_e, P.limits.delta_e); ...
                 saturate(delta(2), -P.limits.delta_a, P.limits.delta_a); ...
                 saturate(delta(3), -P.limits.delta_r, P.limits.delta_r); ...
                 saturate(delta(4), P.limits.delta_t_min, P.limits.delta_t_max)];
end

function y = saturate(u, lower, upper)
y = min(max(u, lower), upper);
end

