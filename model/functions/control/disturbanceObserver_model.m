function uDisObs_k = disturbanceObserver_model(paramStruct, h, uFbackLin_k, omegaMotor_k, omegaMotor_pk, tauE_k, uDisObs_pk)
% Inverse dynamics disturbance Observer for the FJR reduced model
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
% Robot model
i_g = paramStruct.robot.i_g;
baseParam = paramStruct.robot.baseParam;

% Observer
cFilter = paramStruct.controller.cFilter;


% Calculate numeric derivative of motor velocity
domegaMotor_k = discreteDiff_fun(h, omegaMotor_k, omegaMotor_pk);


% Inverse dynamics motor
tauMotor_k = inverseDynamicsMotor_disObserver_reduced_fun(i_g, baseParam, domegaMotor_k, tauE_k);


% Calculate observer output
uDisObs_noFilter_k = tauMotor_k - uFbackLin_k;


% Filter observer output
uDisObs_k = lowPassOrder1(cFilter, uDisObs_noFilter_k, uDisObs_pk);