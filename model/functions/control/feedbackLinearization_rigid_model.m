function u_k = feedbackLinearization_rigid_model(paramStruct, theta_des_k, omega_des_k, domega_des_k, theta_k, omega_k)
% Computed-Torque feedback linearization for the rigid body model
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


% Feedback linearization
k_theta = paramStruct.controller.k_theta;
k_omega = paramStruct.controller.k_omega;


% Calculate v
errorTheta_k = theta_des_k - theta_k;
errorOmega_k = omega_des_k - omega_k;
v_k = k_theta.*errorTheta_k + k_omega.*errorOmega_k + domega_des_k;


% Calculate control output
u_k = feedbackLinearization_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta_k, omega_k, v_k);