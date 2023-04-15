function objFunValue = objFun_lugreFriction(paramStruct, expData, h)
% Objective function for identifying the LuGre model
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load experiment data
omegaMotor = expData.omegaMotor;
T = expData.TMotor;
nSamples = length(omegaMotor);
tauF = zeros(nSamples,1);

% Initialize memory values
omegaMotor_pk = 0;
tauFVW_pk = 0;

z_pk = 0;
dz_pk = 0;


% Simulate static friction model for the whole measurement
for iSample = 1:nSamples

    omegaMotor_k = omegaMotor(iSample);
    T_k = T(iSample);

    % Run dynamic friction model
    [tauF(iSample), tauFVW_k, z_k, dz_k] = lugreFriction_model_midpoint(paramStruct, h, omegaMotor_k, omegaMotor_pk, tauFVW_pk, 0, T_k, z_pk, dz_pk);

    % Update memory values
    omegaMotor_pk = omegaMotor_k;
    tauFVW_pk = tauFVW_k;
    z_pk = z_k;
    dz_pk = dz_k;
    
end

% Calculate RMS for all measurments together
objFunValue = mean((expData.tauMotor - tauF).^2);