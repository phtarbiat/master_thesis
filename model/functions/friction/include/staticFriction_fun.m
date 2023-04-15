function [tauFVW_k, tauFVS_k] = staticFriction_fun(c_c, c_load, c_omega, c_T, omega_k, tauLoad_k, T_k)
% Static Friction Model: friction torque equation considering velocity,
% load torque and temperature dependence
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Velocity weakening friction
tauFVW_k = ...
    c_c(:,1) + c_load.*tauLoad_k.^2;


% Velocity strengthening friction
tauFVS_k = ...
    (c_omega(:,1) - c_T(:,1).*T_k) .* (1 - exp(-abs(omega_k)./(c_omega(:,2) + c_T(:,2).*T_k)) + c_omega(:,3).*abs(omega_k).^0.5);