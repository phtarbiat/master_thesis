%% Script for defining the inputs of the objective function of the identification of the dynamic friction models
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Identification settings
AXIS = 1;                                                       % Robot axis

INPUT_FOLDERS = { ...                                           % Measurment data folder
    'measurments/12_05_dynamicFriction_axis1' ...
    'measurments/11_16_dynamicFriction_axis2' ...
    'measurments/12_01_dynamicFriction_axis3' ...
    };
INPUT_FOLDER_MODEL = 'identification/models/staticFriction';    % Folder with static friction model
OUTPUT_FOLDER = 'identification/models/dynamicFriction';        % Output folder for model

SAMPLE_FREQ = 500;                                              % Sampling frequency of the measurment
TIME_AVERAGE = 0.05;                                            % Length of averaging window

FRICTION_MODEL = 1;                                             % Choosen dynamic friction model
NUM_ELEMENTS_GMS = 4;                                           % Number of elements of GMS friction model


h = 1 / SAMPLE_FREQ; % Step length


% Load static friction model
mdl_staticFriction_omegaT = load([INPUT_FOLDER_MODEL '/model_staticFriction_omegaT_axis' num2str(AXIS) '.mat']);
mdl_staticFriction_load = load([INPUT_FOLDER_MODEL '/model_staticFriction_load_axis' num2str(AXIS) '.mat']);

% Create paramater struct for numeric simulation
paramStruct = struct;
paramStruct.staticFriction = mdl_staticFriction_omegaT.param;
paramStruct.staticFriction.c_load = mdl_staticFriction_load.param.c_load;


% Load measurment data
expData_sim = struct;
expData = load([INPUT_FOLDERS{AXIS} '/expData.mat']);

expData_sim.timeVec = expData.t; % Time
expData_sim.TMotor = expData.temperature(:,AXIS); % Motor temperature

thetaMotor = expData.thetaMot(:,AXIS); % Motor position
tauMotor = expData.torque(:,AXIS); % Motor torque

% Filter position and torque with averaging window
thetaMotor_f = movmean(thetaMotor,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_f = movmean(tauMotor,TIME_AVERAGE*SAMPLE_FREQ);

% Calculate motor velocity by central differentiation
deltaTime = 1 / SAMPLE_FREQ;
nSamples = length(thetaMotor);
omegaMotor_f = zeros(nSamples,1);
omegaMotor_f(2:end-1) = (thetaMotor_f(3:end) - thetaMotor_f(1:end-2)) ./ (2 * deltaTime);

expData_sim.tauMotor = tauMotor_f; % Motor torque
expData_sim.omegaMotor = omegaMotor_f; % Motor velcoity


if FRICTION_MODEL == 0 % LuGre

    paramStruct.lugreObserver.sigma_0 = 1;
    %paramStruct.lugreObserver.sigma_1 = 1;

    objFunValue = objFun_lugreFriction(paramStruct, expData_sim, h)

elseif FRICTION_MODEL == 1 % GMS

    paramStruct.gmsObserver.n = NUM_ELEMENTS_GMS;
    paramStruct.gmsObserver.C = 1;
    paramStruct.gmsObserver.sigma_0 = ones(1,NUM_ELEMENTS_GMS);
    %paramStruct.gmsObserver.sigma_1 = ones(1,NUM_ELEMENTS_GMS);
    paramStruct.gmsObserver.alpha = ones(1,NUM_ELEMENTS_GMS);

    objFunValue = objFun_gmsFriction(paramStruct, expData_sim, h)

end