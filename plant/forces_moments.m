function [forces_body, moments_body, air] = forces_moments(x, delta, wind_ned, P)
%FORCES_MOMENTS Aerosonde aerodynamic, propulsion, and gravity model.
%   [F,M,AIR] = FORCES_MOMENTS(X,DELTA,WIND_NED,P) returns body-axis
%   forces [N], moments [N m], and [Va alpha beta chi Vg_n Vg_e Vg_d].
%   DELTA = [elevator; aileron; rudder; throttle], with angles in radians.

x = x(:); delta = delta(:); wind_ned = wind_ned(:);
phi = x(7); theta = x(8); psi = x(9);
p = x(10); q = x(11); r = x(12);
delta_e = delta(1); delta_a = delta(2);
delta_r = delta(3); delta_t = min(max(delta(4), 0), 1);

air = air_data(x, wind_ned);
Va = air(1); alpha = air(2); beta = air(3);
Va_safe = max(Va, 1.0);
qbar = 0.5*P.rho*Va^2;

% Smoothly blend the attached-flow lift curve into a flat-plate model.
exp1 = exp(min(max(-P.M*(alpha-P.alpha0), -60), 60));
exp2 = exp(min(max( P.M*(alpha+P.alpha0), -60), 60));
sigma = (1 + exp1 + exp2)/((1 + exp1)*(1 + exp2));
CL_linear = P.C_L_0 + P.C_L_alpha*alpha;
CL_flat_plate = 2*sign_nonzero(alpha)*sin(alpha)^2*cos(alpha);
CL_static = (1-sigma)*CL_linear + sigma*CL_flat_plate;
CL = CL_static + P.C_L_q*(P.c/(2*Va_safe))*q + P.C_L_delta_e*delta_e;

% Parabolic induced-drag polar, based on the attached-flow lift curve.
CD = P.C_D_p + CL_linear^2/(pi*P.e_oswald*P.AR) ...
    + P.C_D_q*(P.c/(2*Va_safe))*q + P.C_D_delta_e*delta_e;
CD = max(CD, 0.005);

CY = P.C_Y_0 + P.C_Y_beta*beta ...
    + P.C_Y_p*(P.b/(2*Va_safe))*p ...
    + P.C_Y_r*(P.b/(2*Va_safe))*r ...
    + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;

lift = qbar*P.S_wing*CL;
drag = qbar*P.S_wing*CD;
side_force = qbar*P.S_wing*CY;
force_aero = [-drag*cos(alpha)+lift*sin(alpha); ...
               side_force; ...
              -drag*sin(alpha)-lift*cos(alpha)];

% Deliberately simple textbook propeller approximation along body +X.
thrust = 0.5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2 - Va^2);
force_prop = [thrust; 0; 0];

R_body_to_ned = body_to_ned(phi, theta, psi);
force_gravity = R_body_to_ned.' * [0; 0; P.mass*P.g];
forces_body = force_aero + force_prop + force_gravity;

Cell = P.C_ell_0 + P.C_ell_beta*beta ...
     + P.C_ell_p*(P.b/(2*Va_safe))*p ...
     + P.C_ell_r*(P.b/(2*Va_safe))*r ...
     + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r;
Cm = P.C_m_0 + P.C_m_alpha*alpha ...
   + P.C_m_q*(P.c/(2*Va_safe))*q + P.C_m_delta_e*delta_e;
Cn = P.C_n_0 + P.C_n_beta*beta ...
   + P.C_n_p*(P.b/(2*Va_safe))*p ...
   + P.C_n_r*(P.b/(2*Va_safe))*r ...
   + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r;
moments_body = qbar*P.S_wing .* [P.b*Cell; P.c*Cm; P.b*Cn];
end

function value = sign_nonzero(value)
% Return +1 at zero so the expression remains deterministic.
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

