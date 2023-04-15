function [thetaMotor_des_k, omegaMotor_des_k] = desiredMotorTraj_fun(paramStruct, k, h, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k, thetaMotor_des_pk)
% Calculation of the desired position and velocity trajectory of the motor
% using the FJR model
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
g = paramStruct.robot.g;
i_g = paramStruct.robot.i_g;
r_P1P2 = paramStruct.robot.r_P1P2;
r_P2P3 = paramStruct.robot.r_P2P3;
r_P3P4 = paramStruct.robot.r_P3P4;
baseParam = paramStruct.robot.baseParam;


% Calculate desired Motor position
thetaMotor_des_k = ...
    thetaMotorDes_reduced_fun(g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, k, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k);

% Calculate desired Motor velocity by numeric differentiation
omegaMotor_des_k = discreteDiff_fun(h, thetaMotor_des_k, thetaMotor_des_pk);