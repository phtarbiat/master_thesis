function z_k = gmsFriction_odeSlip_midpoint(alpha, C, h, omega_k, omega_pk, tauFVW_i_k, tauFVW_i_pk, z_pk, dz_pk)
% GMS Friction Model: Numeric solution of the slipping ODE using the
% midpoint rule
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Calculate midpoint values
omega_m = midpoint_fun(omega_k,omega_pk);
tauFVW_i_m = midpoint_fun(tauFVW_i_k,tauFVW_i_pk);
z_m = z_pk + 0.5.*h.*dz_pk;

% Limit maximum value of z_m
if sign(z_m) == sign(omega_k)
    idx = (abs(z_m) > tauFVW_i_k);
    z_m(idx) = sign(z_m(idx)) .* tauFVW_i_k;
end


% Calculate new internal state
dz_m = gmsFriction_odeSlip(alpha, C, omega_m, tauFVW_i_m, z_m);
z_k = z_pk + h.*dz_m;

% Limit maximum value of z_k
if sign(z_k) == sign(omega_k)
    idx = (abs(z_k) > tauFVW_i_k);
    z_k(idx) = sign(z_k(idx)) .* tauFVW_i_k;
end