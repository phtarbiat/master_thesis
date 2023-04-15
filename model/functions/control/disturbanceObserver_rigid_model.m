function uDisObs_k = disturbanceObserver_rigid_model(paramStruct, h, u_k, theta_k, omega_k, omega_pk, uDisObs_pk)
% Inverse dyanmics disturbance Observer for the rigid body model
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
% Robot model
g = paramStruct.robot.g;
r_P1P2 = paramStruct.robot.r_P1P2;
r_P2P3 = paramStruct.robot.r_P2P3;
r_P3P4 = paramStruct.robot.r_P3P4;
baseParam = paramStruct.robot.baseParam;

% Disturbance Observer
cFilter = paramStruct.controller.cFilter;


% Calculate numeric derivative of motor velocity
domega_k = discreteDiff_fun(h, omega_k, omega_pk);


% Inverse dynamics motor
tauMotor_k = inverseDynamicsMotor_disObserver_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta_k, omega_k, domega_k);


% Calculate observer output
uDisObs_noFilter_k = tauMotor_k - u_k;


% Filter observer output
uDisObs_k = lowPassOrder1(cFilter, uDisObs_noFilter_k, uDisObs_pk);