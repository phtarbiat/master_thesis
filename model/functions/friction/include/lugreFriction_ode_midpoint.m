function z_k = lugreFriction_ode_midpoint(sigma_0, h, omega_k, omega_pk, tauFVW_k, tauFVW_pk, z_pk, dz_pk)
% LuGre Friction Model: Numeric solution of the ODE using the
% midpoint rule
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Calculate midpoint values
omega_m = midpoint_fun(omega_k,omega_pk);
tauFVW_m = midpoint_fun(tauFVW_k,tauFVW_pk);
z_m = z_pk + 0.5.*h.*dz_pk;

% Limit maximum value of z_m
idx = abs(z_m) > tauFVW_m./sigma_0;
z_m(idx) = sign(z_m(idx)) .* (tauFVW_m(idx)./sigma_0(idx));


% Calculate new internal state  
dz_m = lugreFriction_ode(sigma_0, omega_m, tauFVW_m, z_m);
z_k = z_pk + h.*dz_m;

% Limit maximum value of z_k
idx = abs(z_k) > tauFVW_k./sigma_0;
z_k(idx) = sign(z_k(idx)) .* (tauFVW_k(idx)./sigma_0(idx));