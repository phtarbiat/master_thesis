function [thetaDiffE_k, omegaDiffE_k] = compKinError_fun(theta_k, omega_k, kinError_k, dkinError_k)
% Calculate joint deflection and first time derivative with considering the
% kinematic error
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Position deflection
thetaDiffE_k = (theta_k(4:6) - theta_k(1:3)) - kinError_k;

% Velocity deflection
omegaDiffE_k = (omega_k(4:6) - omega_k(1:3)) - dkinError_k;