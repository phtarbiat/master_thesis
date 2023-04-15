%% Identification of load dependent joint friction
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Identification settings 
AXIS = 1;                                                       % Robot axis

INPUT_FOLDERS = { ...                                           % Measurment data folders
    '12_07_staticFriction_load_axis1' ...
    '12_14_staticFriction_load_axis2' ...
    '12_14_staticFriction_load_axis3' ...
    };
INPUT_FOLDER_MODEL = 'identification/models/staticFriction';    % Folder with static friction model
OUTPUT_FOLDER = 'identification/models/staticFriction';         % Output folder for model 
SAVE_IDENTIFICATION_DATA = 1;

SAMPLE_FREQ = 500;                                              % Sampling frequency of the measurment
TIME_AVERAGE = 0.05;                                            % Length of averaging window for filtering data

TIME_DYNAMIC_START = 0.1;                                       % Time considered for dynamic effect at start of constant velocity
TIME_DYNAMIC_END = 0.01;                                           % Time considered for dynamic effect at end of constant velocity


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load data


% Get input folder and position difference per step
dirIn = INPUT_FOLDERS{AXIS};


% Load kinematic parameters
kinStruct = loadRobotKinematic(0);
i_g = kinStruct.i_g(AXIS);


% Load data
expData = load([dirIn, '/expData.mat']);

% Extract signals
timeVec = expData.t; % Time
thetaLoad = expData.thetaLoad(:,AXIS);
thetaMotor = expData.thetaMot(:,AXIS); % Motor position
tauMotor = expData.torque(:,AXIS);% .* i_g; % Motor torque
TMotor = expData.temperature(:,AXIS); % Motor temperature
thetaMotor_traj = expData.refValuePos(:,AXIS); % Reference trajectory

nSamples = length(timeVec);

% Filter position and torque with averaging window
thetaMotor_f = movmean(thetaMotor,TIME_AVERAGE*SAMPLE_FREQ);
thetaLoad_f = movmean(thetaLoad,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_f = movmean(tauMotor,TIME_AVERAGE*SAMPLE_FREQ);

% Calculate velocity by central differentiation
deltaTime = mean(diff(timeVec));
omegaMotor_f = zeros(nSamples,1);
omegaMotor_f(2:end-1) = (thetaMotor_f(3:end) - thetaMotor_f(1:end-2)) ./ (2 * deltaTime);

%% Process data


% Find midpoint of measurment
idxMid = round(0.5 * nSamples);


% Find start and end position of the constant velcoity movement
idxStart = [ ...
    find(thetaMotor_traj ~= thetaMotor_traj(1),1); ...
    find(thetaMotor_traj == thetaMotor_traj(idxMid),1,'last') + 1 ...
    ];
idxEnd =[ ...
    find(thetaMotor_traj == thetaMotor_traj(idxMid),1) - 1; ...
    find(thetaMotor_traj ~= thetaMotor_traj(1),1,'last') ...
    ];


% Add start and end time to account 
idxStart = idxStart + TIME_DYNAMIC_START*SAMPLE_FREQ;
idxEnd = idxEnd - TIME_DYNAMIC_END*SAMPLE_FREQ;


% Extract signals according to start and end points
thetaMotorCut = [ ...
    thetaMotor_f(idxStart(1):idxEnd(1)) ...
    flip(thetaMotor_f(idxStart(2):idxEnd(2))) ...
    ];
omegaMotorCut = [ ...
    omegaMotor_f(idxStart(1):idxEnd(1)) ...
    flip(omegaMotor_f(idxStart(2):idxEnd(2))) ...
    ];
tauMotorCut = [ ...
    tauMotor_f(idxStart(1):idxEnd(1)) ...
    flip(tauMotor_f(idxStart(2):idxEnd(2))) ...
    ];
TMotorCut = [ ...
    TMotor(idxStart(1):idxEnd(1)) ...
    flip(TMotor(idxStart(2):idxEnd(2))) ...
    ];

%% Compensate velocity and temperature dependent Friction


% Load velocity and temperature dependent friction model
mdl_staticFriction = load([INPUT_FOLDER_MODEL '/model_staticFriction_omegaT_axis' num2str(AXIS) '.mat']);
paramStruct.staticFriction = mdl_staticFriction.param;
paramStruct.staticFriction.c_load = 0;


% Calculate friction torque compensation for every signal sample
tauFComp = zeros(size(omegaMotorCut));
for iDirection = 1:2
    for iSample = 1:size(omegaMotorCut,1)

        tauFComp(iSample,iDirection) = ...
            staticFriction_model(paramStruct, omegaMotorCut(iSample,iDirection), 0, TMotorCut(iSample,iDirection));

    end
end


% Compensate velocity and temperature dependent friction torque
tauMotor_woFriction = tauMotorCut - tauFComp;

%% Identify load dependent Friction


% Average load and load depenent friction torque over positive and negative
% direction movement
tauLoadVec = 0.5 .* (tauMotor_woFriction(:,1) + tauMotor_woFriction(:,2));
tauFVec = 0.5 .* (tauMotor_woFriction(:,1) - tauMotor_woFriction(:,2));


% Find parameters that minimize MSE between model and measurment data
% Calculate observation matrix
tauLoadVec2 = tauLoadVec.^2;

% Calculate model parameter by LLS
param = (inv(tauLoadVec2' * tauLoadVec2) * tauLoadVec2') * tauFVec


% Calculate model prediction
tauF_mdl = param .* tauLoadVec.^2;


% Compare measurment data with model output
[mae, mape] = calcMAE(tauFVec,tauF_mdl)
[mse, mspe] = calcMSE(tauFVec,tauF_mdl)

maxAE = max(abs(tauFVec - tauF_mdl))


% Plot measurement and model data
figure
hold on
plot(tauLoadVec,tauFVec,'.')
plot(tauLoadVec,tauF_mdl,'-','LineWidth',3)
hold off
legend('measurment','model')
xlabel('tauLoad')
ylabel('tauF')

%% Save model


mdlStruct = struct;

mdlStruct.param.c_load = param;

if SAVE_IDENTIFICATION_DATA

    mdlStruct.data.tauLoad = tauLoadVec;
    mdlStruct.data.tauF = tauFVec;

    mdlStruct.metrics.mae = mae;
    mdlStruct.metrics.mape = mape;
    mdlStruct.metrics.mse = mse;
    mdlStruct.metrics.mspe = mspe;
    mdlStruct.metrics.maxAE = maxAE;

end

save([OUTPUT_FOLDER, '/model_staticFriction_load_axis', num2str(AXIS), '.mat'],'-struct','mdlStruct');