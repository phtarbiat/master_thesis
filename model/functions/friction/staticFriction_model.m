function tauF_k = staticFriction_model(paramStruct, omega_k, tauLoad_k, T_k)
% Static Friction Model considering velcoity, temperature and load using 
% the midpoint rule for numerical integration
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


% Calculate static friction
[tauFVW_k, tauFVS_k] = staticFriction_fun(c_c, c_load, c_omega, c_T, omega_k, tauLoad_k, T_k);
tauF_k = sign(omega_k) .* (tauFVW_k + tauFVS_k);