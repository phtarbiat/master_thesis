function z_k = gmsFriction_odeStick_midpoint(h, omega_k, omega_pk, tauFVW_i_k, tauFVW_i_pk, z_pk)
% GMS Friction Model: Numeric solution of the sticking ODE using the
% midpoint rule
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Calculate midpoint values
omega_m = midpoint_fun(omega_k,omega_pk);


% Calculate new internal state
dz_m = gmsFriction_odeStick(omega_m);
z_k = z_pk + h.*dz_m;

% Limit maximum value of z_k
if sign(z_k) == sign(omega_k)
    idx = (abs(z_k) > tauFVW_i_k);
    z_k(idx) = sign(z_k(idx)) .* tauFVW_i_k;
end