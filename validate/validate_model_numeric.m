%% Numeric validation of the identified model
% Calculate motor torque and acceleration by the inverse and direct dynamics 
% of the robot for every time step of a given validation trajectory
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Simulation settings 
ROBOTER_STRUCTURE = 1;                              % Robot structure (0 = 500mm arms; 1 = 200mm arms)
FLEXIBLE_MODEL = 0;                                 % Consider flexible or rigid robot model

INPUT_FOLDER = 'measurments/12_08_validation';      % Measurment data folder
INPUT_FOLDER_MODEL = 'identification/models';       % Folder with robot model
OUTPUT_FOLDER = 'validate/output';                  % Output folder for validation data                    

SAMPLE_FREQ = 500;                                  % Sampling frequency of the measurment data
TIME_AVERAGE = 0.05;                                % Length of averaging window for filtering data


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load data


% Load robot model
paramStruct = load('identification/models/model_complete.mat');
paramStruct.kinError.addKinError = 1;

% Load CAD model
kinStruct = loadRobotKinematic(ROBOTER_STRUCTURE);
cadStruct = loadRobotDynamicCAD(ROBOTER_STRUCTURE,kinStruct,FLEXIBLE_MODEL);
baseParam_cad = cadStruct.baseParam;


% Load and extract measurment data
expData = load([INPUT_FOLDER, '/expData_',num2str(1),'.mat']);

timeVec = expData.t; % Time
thetaLoad_temp = expData.thetaLoad; % Load position
thetaMotor_temp = expData.thetaMot; % Motor position
tauMotor_temp = expData.torque .* paramStruct.robot.i_g'; % Motor torque
TMotor = expData.temperature; % Motor temperature

nSamples = length(timeVec); % Number of samples

% Filter position and torque with averaging window
thetaLoad_f = movmean(thetaLoad_temp,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_f = movmean(thetaMotor_temp,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_f = movmean(tauMotor_temp,TIME_AVERAGE*SAMPLE_FREQ);

% Calculate velocity and acceleration by central differentiation
deltaTime = 1 / SAMPLE_FREQ;

omegaLoad_f = zeros(nSamples,3);
omegaLoad_f(2:end-1,:) = (thetaLoad_f(3:end,:) - thetaLoad_f(1:end-2,:)) ./ (2 * deltaTime);
domegaLoad_f = zeros(nSamples,3);
domegaLoad_f(2:end-1,:) = (omegaLoad_f(3:end,:) - omegaLoad_f(1:end-2,:)) ./ (2 * deltaTime);

omegaMotor_f = zeros(nSamples,3);
omegaMotor_f(2:end-1,:) = (thetaMotor_f(3:end,:) - thetaMotor_f(1:end-2,:)) ./ (2 * deltaTime);
domegaMotor_f = zeros(nSamples,3);
domegaMotor_f(2:end-1,:) = (omegaMotor_f(3:end,:) - omegaMotor_f(1:end-2,:)) ./ (2 * deltaTime);

%% Calculate Model output


% Extract kinematic parameters
g = paramStruct.robot.g;
i_g = paramStruct.robot.i_g;
r_P1P2 = paramStruct.robot.r_P1P2;
r_P2P3 = paramStruct.robot.r_P2P3;
r_P3P4 = paramStruct.robot.r_P3P4;


% Initialize iteration vectors
tauMotor_mdl = zeros(nSamples,3);
tauMotor_mdl_reg = zeros(nSamples,3);
tauMotor_mdl_cad = zeros(nSamples,3);
if FLEXIBLE_MODEL
    domega_mdl = zeros(nSamples,6);
    domega_mdl_cad = zeros(nSamples,6);

    kinError = zeros(nSamples,3);
    thetDiffE = zeros(nSamples,3);
    tauE = zeros(nSamples,3);
else
    domega_mdl = zeros(nSamples,3);
    domega_mdl_cad = zeros(nSamples,3);
end


% Initialize memory values of LuGre friction model
omegaMotor_pk = zeros(3,1);
tauFVW_pk = zeros(3,1);
z_pk = zeros(3,1);
dz_pk = zeros(3,1);


% Iterate through measurment samples
for iSample = 1:nSamples

    % Load input data of current sample
    theta_i = [thetaLoad_f(iSample,:)'; thetaMotor_f(iSample,:)']; % Position
    omega_i = [omegaLoad_f(iSample,:)'; omegaMotor_f(iSample,:)']; % Velocity
    domega_i = [domegaLoad_f(iSample,:)'; domegaMotor_f(iSample,:)']; % Acceleration
    tauMotor_i = tauMotor_f(iSample,:)'; % Motor torque
    TMotor_i = TMotor(iSample,:)'; % Motor temperature

    % Calculate friction torque
    [tauF_i, tauFVW, z, dz] = ...
            lugreFriction_model_midpoint(paramStruct, 1/SAMPLE_FREQ, omega_i(4:6), omegaMotor_pk, tauFVW_pk, zeros(3,1), TMotor_i, z_pk, dz_pk);
    tauF_i = tauF_i .* i_g;

    if FLEXIBLE_MODEL
        % Calculate joint stiffness torque
        % Calculate kinematic error and according position deflection
        kinError_i = kinError_fun(paramStruct, theta_i(4:6));
        [thetaDiffE_i, ~] = compKinError_fun(theta_i, zeros(6,1), kinError_i, zeros(3,1));
        % Calculate joint stiffness
        degK_i = stiffness_fun(paramStruct, thetaDiffE_i);
        % Calculate joint stiffness torque
        tauE_i = degK_i .* thetaDiffE_i;

        % Save values to vector
        kinError(iSample,:) = kinError_i';
        thetDiffE(iSample,:) = thetDiffE_i';
        tauE(iSample,:) = tauE_i';
    end


    % Calculate regressor matrix 
    if FLEXIBLE_MODEL

    else
        Y_i = YFun(g,i_g,r_P1P2,r_P2P3,r_P3P4,deg2rad([theta_i(4:6); theta_i(4:6)]),deg2rad([omega_i(4:6); omega_i(4:6)]),deg2rad([domega_i(4:6); domega_i(4:6)]));
        Y_i(4:6,:) = []; % Only consider link equations
    end

    % Calculate inverse and direct dynamics for identified and CAD base parameters
    if FLEXIBLE_MODEL

    else
        tauMotor_mdl_reg(iSample,:) = (Y_i*paramStruct.robot.baseParam + tauF_i)';

        tauMotor_mdl(iSample,:) = ...
            inverseDynamics_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, paramStruct.robot.baseParam, deg2rad(theta_i(4:6)), deg2rad(omega_i(4:6)), deg2rad(domega_i(4:6)), tauF_i)';
        domega_mdl(iSample,:) = ...
            directDynamics_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, paramStruct.robot.baseParam, deg2rad(theta_i(4:6)), deg2rad(omega_i(4:6)), tauF_i, tauMotor_i)';

        tauMotor_mdl_cad(iSample,:) = ...
            inverseDynamics_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam_cad, deg2rad(theta_i(4:6)), deg2rad(omega_i(4:6)), deg2rad(domega_i(4:6)), tauF_i)';
        domega_mdl_cad(iSample,:) = ...
            directDynamics_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam_cad, deg2rad(theta_i(4:6)), deg2rad(omega_i(4:6)), tauF_i, tauMotor_i)';

    end


    % Save memory values of LuGre friction model
    omegaMotor_pk = omega_i(4:6);
    tauFVW_pk = tauFVW;
    z_pk = z;
    dz_pk = dz;
    
end


% Convert from rad to deg
domega_mdl = rad2deg(domega_mdl);
domega_mdl_cad = rad2deg(domega_mdl_cad);

%% Calculate error metrics


% Compare measurment data with model output
mae_tau = zeros(4,1);
mae_tau_reg = zeros(4,1);
mae_tau_cad = zeros(4,1);
mae_domega = zeros(4,1);
mae_domega_cad = zeros(4,1);
mape_tau = zeros(4,1);
mape_tau_reg = zeros(4,1);
mape_tau_cad = zeros(4,1);
mape_domega = zeros(4,1);
mape_domega_cad = zeros(4,1);

mse_tau = zeros(4,1);
mse_tau_reg = zeros(4,1);
mse_tau_cad = zeros(4,1);
mse_domega = zeros(4,1);
mse_domega_cad = zeros(4,1);
mspe_tau = zeros(4,1);
mspe_tau_reg = zeros(4,1);
mspe_tau_cad = zeros(4,1);
mspe_domega = zeros(4,1);
mspe_domega_cad = zeros(4,1);

for iJoint = 1:3

    [mae_tau(iJoint), mape_tau(iJoint)] = calcMAE(tauMotor_f(:,iJoint),tauMotor_mdl(:,iJoint));
    [mse_tau(iJoint), mspe_tau(iJoint)] = calcMSE(tauMotor_f(:,iJoint),tauMotor_mdl(:,iJoint));

    [mae_tau_reg(iJoint), mape_tau_reg(iJoint)] = calcMAE(tauMotor_f(:,iJoint),tauMotor_mdl_reg(:,iJoint));
    [mse_tau_reg(iJoint), mspe_tau_reg(iJoint)] = calcMSE(tauMotor_f(:,iJoint),tauMotor_mdl_reg(:,iJoint));

    [mae_tau_cad(iJoint), mape_tau_cad(iJoint)] = calcMAE(tauMotor_f(:,iJoint),tauMotor_mdl_cad(:,iJoint));
    [mse_tau_cad(iJoint), mspe_tau_cad(iJoint)] = calcMSE(tauMotor_f(:,iJoint),tauMotor_mdl_cad(:,iJoint));

    [mae_domega(iJoint), mape_domega(iJoint)] = calcMAE(domegaMotor_f(:,iJoint),domega_mdl(:,iJoint));
    [mse_domega(iJoint), mspe_domega(iJoint)] = calcMSE(domegaMotor_f(:,iJoint),domega_mdl(:,iJoint));

    [mae_domega_cad(iJoint), mape_domega_cad(iJoint)] = calcMAE(domegaMotor_f(:,iJoint),domega_mdl_cad(:,iJoint));
    [mse_domega_cad(iJoint), mspe_domega_cad(iJoint)] = calcMSE(domegaMotor_f(:,iJoint),domega_mdl_cad(:,iJoint));

end

[mae_tau(4), mape_tau(4)] = calcMAE(tauMotor_f(:),tauMotor_mdl(:));
[mse_tau(4), mspe_tau(4)] = calcMSE(tauMotor_f(:),tauMotor_mdl(:));

[mae_tau_reg(4), mape_tau_reg(4)] = calcMAE(tauMotor_f(:),tauMotor_mdl_reg(:));
[mse_tau_reg(4), mspe_tau_reg(4)] = calcMSE(tauMotor_f(:),tauMotor_mdl_reg(:));

[mae_tau_cad(4), mape_tau_cad(4)] = calcMAE(tauMotor_f(:),tauMotor_mdl_cad(:));
[mse_tau_cad(4), mspe_tau_cad(4)] = calcMSE(tauMotor_f(:),tauMotor_mdl_cad(:));

[mae_domega(4), mape_domega(4)] = calcMAE(domegaMotor_f(:),domega_mdl(:));
[mse_domega(4), mspe_domega(4)] = calcMSE(domegaMotor_f(:),domega_mdl(:));

[mae_domega_cad(4), mape_domega_cad(4)] = calcMAE(domegaMotor_f(:),domega_mdl_cad(:));
[mse_domega_cad(4), mspe_domega_cad(4)] = calcMSE(domegaMotor_f(:),domega_mdl_cad(:));

%% Plot data


% Torque
figure
hold on
plot(timeVec,tauMotor_f(:,1))
plot(timeVec,tauMotor_mdl(:,1),'--')
plot(timeVec,tauMotor_mdl_cad(:,1),':')
hold off
title('Achse 1')

figure
hold on
plot(timeVec,tauMotor_f(:,2))
plot(timeVec,tauMotor_mdl(:,2),'--')
plot(timeVec,tauMotor_mdl_cad(:,2),':')
hold off
title('Achse 2')

figure
hold on
plot(timeVec,tauMotor_f(:,3))
plot(timeVec,tauMotor_mdl(:,3),'--')
plot(timeVec,tauMotor_mdl_cad(:,3),':')
hold off
title('Achse 3')


% Acceleration
figure
hold on
plot(timeVec,domegaMotor_f(:,1))
plot(timeVec,domega_mdl(:,1),'--')
plot(timeVec,domega_mdl_cad(:,1),':')
hold off
title('Achse 1')

figure
hold on
plot(timeVec,domegaMotor_f(:,2))
plot(timeVec,domega_mdl(:,2),'--')
plot(timeVec,domega_mdl_cad(:,2),':')
hold off
title('Achse 2')

figure
hold on
plot(timeVec,domegaMotor_f(:,3))
plot(timeVec,domega_mdl(:,3),'--')
plot(timeVec,domega_mdl_cad(:,3),':')
hold off
title('Achse 3')

%% Save data


dataStruct = struct;

dataStruct.data.time = timeVec;

dataStruct.data.tauMotor= tauMotor_f;
dataStruct.data.tauMotor_mdl = tauMotor_mdl;
dataStruct.data.tauMotor_mdl_reg = tauMotor_mdl_reg;
dataStruct.data.tauMotor_mdl_cad = tauMotor_mdl_cad;

if FLEXIBLE_MODEL
    dataStruct.data.theta = [thetaLoad_f, thetaMotor_f];
    dataStruct.data.omega = [omegaLoad_f, omegaMotor_f];
    dataStruct.data.domega = [domegaLoad_f, domegaMotor_f];
else
    dataStruct.data.theta = thetaMotor_f;
    dataStruct.data.omega = omegaMotor_f;
    dataStruct.data.domega = domegaMotor_f;
end
dataStruct.data.domega_mdl = domega_mdl;
dataStruct.data.domega_mdl_cad = domega_mdl_cad;


dataStruct.metrics.mae_tau = mae_tau;
dataStruct.metrics.mape_tau = mape_tau;
dataStruct.metrics.mse_tau = mse_tau;
dataStruct.metrics.mspe_tau = mspe_tau;

dataStruct.metrics.mae_tau_reg = mae_tau_reg;
dataStruct.metrics.mape_tau_reg = mape_tau_reg;
dataStruct.metrics.mse_tau_reg = mse_tau_reg;
dataStruct.metrics.mspe_tau_reg = mspe_tau_reg;

dataStruct.metrics.mae_tau_cad = mae_tau_cad;
dataStruct.metrics.mape_tau_cad = mape_tau_cad;
dataStruct.metrics.mse_tau_cad = mse_tau_cad;
dataStruct.metrics.mspe_tau_cad = mspe_tau_cad;

dataStruct.metrics.mae_domega = mae_domega;
dataStruct.metrics.mape_domega = mape_domega;
dataStruct.metrics.mse_domega = mse_domega;
dataStruct.metrics.mspe_domega = mspe_domega;

dataStruct.metrics.mae_domega_cad = mae_domega_cad;
dataStruct.metrics.mape_domega_cad = mape_domega_cad;
dataStruct.metrics.mse_domega_cad = mse_domega_cad;
dataStruct.metrics.mspe_domega_cad = mspe_domega_cad;

save([OUTPUT_FOLDER '/data_modelValidation.mat'],'-struct','dataStruct');