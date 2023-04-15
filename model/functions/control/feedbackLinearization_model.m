function u_k= feedbackLinearization_model(paramStruct, k, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k, ddomegaLoad_des_k, dddomegaLoad_des_k, thetaLoad_k, omegaLoad_k, domegaLoad_k, ddomegaLoad_k)
% Feedback linearization for the FJR reduced model
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
% Robot model
g = paramStruct.robot.g;
i_g = paramStruct.robot.i_g;
r_P1P2 = paramStruct.robot.r_P1P2;
r_P2P3 = paramStruct.robot.r_P2P3;
r_P3P4 = paramStruct.robot.r_P3P4;
baseParam = paramStruct.robot.baseParam;

% Feedback linearization
k_theta = paramStruct.controller.k_theta;
k_omega = paramStruct.controller.k_omega;
k_domega = paramStruct.controller.k_domega;
k_ddomega = paramStruct.controller.k_ddomega;


% Calculate v
errorTheta_k = thetaLoad_des_k - thetaLoad_k;
errorOmega_k = omegaLoad_des_k - omegaLoad_k;
errorDOmega_k = domegaLoad_des_k - domegaLoad_k;
errorDDOmega_k = ddomegaLoad_des_k - ddomegaLoad_k;
v_k = k_theta.*errorTheta_k + k_omega.*errorOmega_k + k_domega.*errorDOmega_k + k_ddomega.*errorDDOmega_k + dddomegaLoad_des_k;


% Calculate control output
u_1_k = feedbackLinearization1_fun(g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, k, thetaLoad_k, omegaLoad_k, domegaLoad_k, ddomegaLoad_k, v_k);
u_2_k = feedbackLinearization2_fun(g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad_k, omegaLoad_k, domegaLoad_k);
u_k = u_1_k + u_2_k;