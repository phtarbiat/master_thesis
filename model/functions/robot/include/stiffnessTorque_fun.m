function [tauE, dtauE] = stiffnessTorque_fun(k, thetaDiff_k, omegaDiff_k)
% Calculate joint spring torque and the first time derivative from joint stiffness
% and deflection
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


tauE = k .* thetaDiff_k;
dtauE = k .* omegaDiff_k;