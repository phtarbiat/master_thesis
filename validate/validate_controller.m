%% Validation of measurement data for the comparison of different model-based controllers
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Settings
INPUT_FOLDER = { ...                                % Measurment data folder
    'measurments/01_16_control_CT_RK_SB' ...
    'measurments/01_16_control_CT_RK' ...
    'measurments/01_16_control_CT' ...
    'measurments/01_16_control_CT_RK_SB_CAD' ...
    'measurments/12_08_validation' ...
    };
OUTPUT_FOLDER = 'validate/output';      % Output folder for validation data                    

SAMPLE_FREQ = 500;                                  % Sampling frequency of the measurment data
TIME_AVERAGE = 0.05;                                % Length of averaging window for filtering data


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load data


% Load kinematic parameters
paramStruct = loadRobotKinematic(1);


% Load and extract measurment data
expData_com = load([INPUT_FOLDER{1}, '/expData.mat']);
expData_DisObs = load([INPUT_FOLDER{2}, '/expData.mat']);
expData_FricComp = load([INPUT_FOLDER{3}, '/expData.mat']);
expData_cad = load([INPUT_FOLDER{4}, '/expData.mat']);
expData_val = load([INPUT_FOLDER{5}, '/expData_1.mat']);

% General values
timeVec = expData_com.t; % Time
thetaMotor_traj = expData_com.refValuePos; % Reference position trajectory
omegaMotor_traj = expData_com.refValueVel; % Reference velocity trajectory
domegaMotor_traj = expData_com.refValueAcc; % Reference acceleration trajectory

uGes_com = expData_com.uGes .* paramStruct.i_g'; % Reference acceleration trajectory
uFbackLin_com = expData_com.uFbackLin .* paramStruct.i_g'; % Reference acceleration trajectory
uFricComp_com = expData_com.uFricComp .* paramStruct.i_g'; % Reference acceleration trajectory
uDisObs_com = expData_com.uDisObs .* paramStruct.i_g'; % Reference acceleration trajectory

% Motor position
thetaMotor_com = expData_com.thetaMot; 
thetaMotor_DisObs = expData_DisObs.thetaMot;
thetaMotor_FricComp = expData_FricComp.thetaMot;
thetaMotor_cad = expData_cad.thetaMot;
thetaMotor_val = expData_val.thetaMot;

% Motor torque
tauMotor_com = expData_com.torque .* paramStruct.i_g';
tauMotor_DisObs = expData_DisObs.torque .* paramStruct.i_g';
tauMotor_FricComp = expData_FricComp.torque .* paramStruct.i_g';
tauMotor_cad = expData_cad.torque .* paramStruct.i_g';
tauMotor_val = expData_val.torque .* paramStruct.i_g';

nSamples = length(timeVec); % Number of samples

% Filter position and torque with averaging window
thetaMotor_com_f = movmean(thetaMotor_com,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_DisObs_f = movmean(thetaMotor_DisObs,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_FricComp_f = movmean(thetaMotor_FricComp,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_cad_f = movmean(thetaMotor_cad,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_val_f = movmean(thetaMotor_val,TIME_AVERAGE*SAMPLE_FREQ);

tauMotor_com_f = movmean(tauMotor_com,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_DisObs_f = movmean(tauMotor_DisObs,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_FricComp_f = movmean(tauMotor_FricComp,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_cad_f = movmean(tauMotor_cad,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_val_f = movmean(tauMotor_val,TIME_AVERAGE*SAMPLE_FREQ);

% Calculate velocity and acceleration by central differentiation
deltaTime = 1 / SAMPLE_FREQ;

omegaMotor_com_f = zeros(nSamples,3);
omegaMotor_com_f(2:end-1,:) = (thetaMotor_com_f(3:end,:) - thetaMotor_com_f(1:end-2,:)) ./ (2 * deltaTime);
omegaMotor_DisObs_f = zeros(nSamples,3);
omegaMotor_DisObs_f(2:end-1,:) = (thetaMotor_DisObs_f(3:end,:) - thetaMotor_DisObs_f(1:end-2,:)) ./ (2 * deltaTime);
omegaMotor_FricComp_f = zeros(nSamples,3);
omegaMotor_FricComp_f(2:end-1,:) = (thetaMotor_FricComp_f(3:end,:) - thetaMotor_FricComp_f(1:end-2,:)) ./ (2 * deltaTime);
omegaMotor_cad_f = zeros(nSamples,3);
omegaMotor_cad_f(2:end-1,:) = (thetaMotor_cad_f(3:end,:) - thetaMotor_cad_f(1:end-2,:)) ./ (2 * deltaTime);
omegaMotor_val_f = zeros(nSamples,3);
omegaMotor_val_f(2:end-1,:) = (thetaMotor_val_f(3:end,:) - thetaMotor_val_f(1:end-2,:)) ./ (2 * deltaTime);

%% Calculate tracking errors


errors = struct;

% Position errors
errors.com.theta = thetaMotor_traj - thetaMotor_com_f;
errors.DisObs.theta = thetaMotor_traj - thetaMotor_DisObs_f;
errors.FricComp.theta = thetaMotor_traj - thetaMotor_FricComp_f;
errors.cad.theta = thetaMotor_traj - thetaMotor_cad_f;
errors.val.theta = thetaMotor_traj - thetaMotor_val_f;

% Velocity errors
errors.com.omega = omegaMotor_traj - omegaMotor_com_f;
errors.DisObs.omega = omegaMotor_traj - omegaMotor_DisObs_f;
errors.FricComp.omega = omegaMotor_traj - omegaMotor_FricComp_f;
errors.cad.omega = omegaMotor_traj - omegaMotor_cad_f;
errors.val.omega = omegaMotor_traj - omegaMotor_val_f;

%% Calculate error metrics


% Names of controllers and errors
controllerNames = {'com' 'DisObs' 'FricComp' 'cad' 'val'};
errorNames = {'theta' 'omega'};

% Calculate MAE and MSE
mae = struct;
mse = struct;

for iC = 1:length(controllerNames)
    for iE = 1:length(errorNames)

        mae.(errorNames{iE}).(controllerNames{iC}) = zeros(4,1);
        mse.(errorNames{iE}).(controllerNames{iC}) = zeros(4,1);

        for iJoint = 1:3

            mae.(errorNames{iE}).(controllerNames{iC})(iJoint) = ...
                sum(abs(errors.(controllerNames{iC}).(errorNames{iE})(:,iJoint)));
            mse.(errorNames{iE}).(controllerNames{iC})(iJoint) = ...
                sum((errors.(controllerNames{iC}).(errorNames{iE})(:,iJoint)).^2);

        end

        mae.(errorNames{iE}).(controllerNames{iC})(4) = ...
            sum(abs(errors.(controllerNames{iC}).(errorNames{iE})(:)));
        mse.(errorNames{iE}).(controllerNames{iC})(4) = ...
            sum((errors.(controllerNames{iC}).(errorNames{iE})(:)).^2);

    end
end

%% Plot data


% u
figure
hold on
plot(timeVec,uGes_com(:,1))
plot(timeVec,uFbackLin_com(:,1),'--')
plot(timeVec,uFricComp_com(:,1),'--')
plot(timeVec,uDisObs_com(:,1),'--')
hold off
title('Achse 1')
legend('uGes','uFbackLin','uFricComp','uDisObs')

figure
hold on
plot(timeVec,tauMotor_com_f(:,1),'-')
plot(timeVec,uGes_com(:,1),'--')
hold off
title('Achse 1')
legend('tauMotor','uGes')

% Position comparison
figure
hold on
plot(timeVec,thetaMotor_traj(:,1))
plot(timeVec,thetaMotor_com_f(:,1),'--')
plot(timeVec,thetaMotor_DisObs_f(:,1),'--')
plot(timeVec,thetaMotor_FricComp_f(:,1),'--')
hold off
title('Achse 1')
legend('traj','com','woDisObs','woFricComp')

figure
hold on
plot(timeVec,thetaMotor_traj(:,2))
plot(timeVec,thetaMotor_com_f(:,2),'--')
plot(timeVec,thetaMotor_DisObs_f(:,2),'--')
plot(timeVec,thetaMotor_FricComp_f(:,2),'--')
hold off
title('Achse 2')
legend('traj','com','woDisObs','woFricComp')

figure
hold on
plot(timeVec,thetaMotor_traj(:,3))
plot(timeVec,thetaMotor_com_f(:,3),'--')
plot(timeVec,thetaMotor_DisObs_f(:,3),'--')
plot(timeVec,thetaMotor_FricComp_f(:,3),'--')
hold off
title('Achse 3')
legend('traj','com','woDisObs','woFricComp')

% Velocity comparision
figure
hold on
plot(timeVec,omegaMotor_traj(:,1))
plot(timeVec,omegaMotor_com_f(:,1),'--')
plot(timeVec,omegaMotor_DisObs_f(:,1),'--')
plot(timeVec,omegaMotor_FricComp_f(:,1),'--')
hold off
title('Achse 1')
legend('traj','com','woDisObs','woFricComp')

figure
hold on
plot(timeVec,omegaMotor_traj(:,2))
plot(timeVec,omegaMotor_com_f(:,2),'--')
plot(timeVec,omegaMotor_DisObs_f(:,2),'--')
plot(timeVec,omegaMotor_FricComp_f(:,2),'--')
hold off
title('Achse 2')
legend('traj','com','woDisObs','woFricComp')

figure
hold on
plot(timeVec,omegaMotor_traj(:,3))
plot(timeVec,omegaMotor_com_f(:,3),'--')
plot(timeVec,omegaMotor_DisObs_f(:,3),'--')
plot(timeVec,omegaMotor_FricComp_f(:,3),'--')
hold off
title('Achse 3')
legend('traj','com','woDisObs','woFricComp')

%% Save data


dataStruct = struct;

dataStruct.data.time = timeVec;

dataStruct.data.thetaMotor_traj = thetaMotor_traj;
dataStruct.data.omegaMotor_traj = omegaMotor_traj;
dataStruct.data.domegaMotor_traj = domegaMotor_traj;

dataStruct.data.uGes_com = uGes_com;
dataStruct.data.uFbackLin_com = uFbackLin_com;
dataStruct.data.uFricComp_com = uFricComp_com;
dataStruct.data.uDisObs_com = uDisObs_com;

dataStruct.data.theta.com = thetaMotor_com_f;
dataStruct.data.omega.com = omegaMotor_com_f;
dataStruct.data.tauMotor.com = tauMotor_com_f;

dataStruct.data.theta.DisObs = thetaMotor_DisObs_f;
dataStruct.data.omega.DisObs = omegaMotor_DisObs_f;
dataStruct.data.tauMotor.DisObs = tauMotor_DisObs_f;

dataStruct.data.theta.FricComp = thetaMotor_FricComp_f;
dataStruct.data.omega.FricComp = omegaMotor_FricComp_f;
dataStruct.data.tauMotor.FricComp = tauMotor_FricComp_f;

dataStruct.data.theta.cad = thetaMotor_cad_f;
dataStruct.data.omega.cad = omegaMotor_cad_f;
dataStruct.data.tauMotor.cad = tauMotor_cad_f;

dataStruct.data.theta.val = thetaMotor_val_f;
dataStruct.data.omega.val = omegaMotor_val_f;
dataStruct.data.tauMotor.val = tauMotor_val_f;

for iC = 1:length(controllerNames)
    dataStruct.data.eTheta.(controllerNames{iC}) = errors.(controllerNames{iC}).theta;
    dataStruct.data.eOmega.(controllerNames{iC}) = errors.(controllerNames{iC}).omega;
end

dataStruct.metrics.mae = mae;
dataStruct.metrics.mse = mse;

save([OUTPUT_FOLDER '/data_controllerValidation.mat'],'-struct','dataStruct');