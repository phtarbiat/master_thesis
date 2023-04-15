function [tauF_k, tauFVW_k, z_k, dz_k] = lugreFriction_model_midpoint(paramStruct, h, omega_k, omega_pk, tauFVW_pk, tauLoad_k, T_k, z_pk, dz_pk)
% LuGre Friction Model using the midpoint rule for numerical integration
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
idxOmegaPos = omega_k >= 0; % Check movement direction for every joint

% Static friction
c_c  = paramStruct.staticFriction.c_c_neg;
c_c(idxOmegaPos,:) = paramStruct.staticFriction.c_c_pos(idxOmegaPos,:);
c_load = paramStruct.staticFriction.c_load;
c_omega = paramStruct.staticFriction.c_omega_neg;
c_omega(idxOmegaPos,:) = paramStruct.staticFriction.c_omega_pos(idxOmegaPos,:);
c_T = paramStruct.staticFriction.c_T_neg;
c_T(idxOmegaPos,:) = paramStruct.staticFriction.c_T_pos(idxOmegaPos,:);

% LuGre Friction
sigma_0 = paramStruct.lugreObserver.sigma_0;
sigma_1 = 0; % Damping coefficient is set to zero


% Calculate Static Friction
[tauFVW_k, tauFVS_k] = staticFriction_fun(c_c, c_load, c_omega, c_T, omega_k, tauLoad_k, T_k);
tauFVS_k = sign(omega_k) .* tauFVS_k;
idxOmegaZero = omega_k == 0;
tauFVW_k(idxOmegaZero) = 1e-10;


% Calculate internal state and its derivative
z_k = lugreFriction_ode_midpoint(sigma_0, h, omega_k, omega_pk, tauFVW_k, tauFVW_pk, z_pk, dz_pk);
dz_k = lugreFriction_ode(sigma_0, omega_k, tauFVW_k, z_k);


% Calculate LuGre Friction
tauF_k = lugreFriction_fun(sigma_0, sigma_1, tauFVS_k, z_k, dz_k);