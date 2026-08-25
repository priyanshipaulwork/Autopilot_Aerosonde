function xdot = aircraft_dynamics(x, delta, wind_ned, P)
%AIRCRAFT_DYNAMICS Nonlinear 12-state rigid-body fixed-wing equations.
%   XDOT = AIRCRAFT_DYNAMICS(X,DELTA,WIND_NED,P) propagates NED position,
%   body-axis ground velocity, 3-2-1 Euler attitude, and body rates. SI
%   units and radians are used throughout.

x = x(:); delta = delta(:); wind_ned = wind_ned(:);
velocity_body = x(4:6);
phi = x(7); theta = x(8); psi = x(9);
omega_body = x(10:12);

[forces_body, moments_body] = forces_moments(x, delta, wind_ned, P);
R_body_to_ned = body_to_ned(phi, theta, psi);
position_dot = R_body_to_ned*velocity_body;
velocity_dot = forces_body/P.mass - cross(omega_body, velocity_body);

% Protect the Euler-rate map away from its theta = +/-90 deg singularity.
cos_theta = cos(theta);
if abs(cos_theta) < 1e-3
    cos_theta = sign_nonzero(cos_theta)*1e-3;
end
tan_theta = sin(theta)/cos_theta;
euler_map = [1, sin(phi)*tan_theta,  cos(phi)*tan_theta; ...
             0, cos(phi),           -sin(phi); ...
             0, sin(phi)/cos_theta,  cos(phi)/cos_theta];
euler_dot = euler_map*omega_body;

inertia = [P.Jx, 0, -P.Jxz; 0, P.Jy, 0; -P.Jxz, 0, P.Jz];
omega_dot = inertia \ (moments_body - cross(omega_body, inertia*omega_body));
xdot = [position_dot; velocity_dot; euler_dot; omega_dot];
end

function value = sign_nonzero(value)
if value >= 0
    value = 1;
else
    value = -1;
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

