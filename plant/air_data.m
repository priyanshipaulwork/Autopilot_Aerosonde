function air = air_data(x, wind_ned)
%AIR_DATA Calculate wind-relative air data and ground course.
%   AIR = AIR_DATA(X,WIND_NED) returns [Va; alpha; beta; chi; Vg_n;
%   Vg_e; Vg_d]. X uses the documented 12-state NED/body convention and
%   WIND_NED is a 3-by-1 steady wind vector [m/s].

x = x(:);
wind_ned = wind_ned(:);
phi = x(7); theta = x(8); psi = x(9);

R_body_to_ned = body_to_ned(phi, theta, psi);
velocity_ground_body = x(4:6);
velocity_wind_body = R_body_to_ned.' * wind_ned;
velocity_air_body = velocity_ground_body - velocity_wind_body;

Va = max(norm(velocity_air_body), 1e-6);
alpha = atan2(velocity_air_body(3), velocity_air_body(1));
beta_argument = velocity_air_body(2)/Va;
beta = asin(min(max(beta_argument, -1), 1));

velocity_ground_ned = R_body_to_ned * velocity_ground_body;
chi = atan2(velocity_ground_ned(2), velocity_ground_ned(1));
air = [Va; alpha; beta; chi; velocity_ground_ned];
end

function R = body_to_ned(phi, theta, psi)
% 3-2-1 body-to-NED direction cosine matrix.
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi; ...
     cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi; ...
     -sth,     sphi*cth,                   cphi*cth];
end

