%% Initialization of identified model and measurement for Simulink simulation
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Simulation settings
ROBOTER_STRUCTURE = 1;                          % Robot structure (0 = 500mm arms; 1 = 200mm arms)
FLEXIBLE_MODEL = 0;                             % Consider flexible or rigid robot model
CAD_MODEL = 0;                                  % Consider CAD parameters
INPUT_FOLDER = 'measurments/12_08_validation';  % Measurment data folder
INPUT_FOLDER_MODEL = 'identification/models';   % Folder with robot model

%% Create variables for Simulink model


% Load parameters
paramStruct = load('identification/models/model_complete.mat');
paramStruct.robot.structure = ROBOTER_STRUCTURE;

if FLEXIBLE_MODEL
    paramStruct.robot.comModel = 0;
    paramStruct.kinError.addKinError = 1;
end
if CAD_MODEL
    kinStruct = loadRobotKinematic(ROBOTER_STRUCTURE);
    cadStruct = loadRobotDynamicCAD(ROBOTER_STRUCTURE,kinStruct,FLEXIBLE_MODEL);

    paramStruct.robot.baseParam = cadStruct.baseParam;
    
end


% Create Bus from paramStruct
createParamBus(paramStruct,'main_bus','base')


% Load and extract measurment data
expData = load([INPUT_FOLDER, '/expData_',num2str(1),'.mat']);

timeVec = expData.t; % Time
tauMotor = expData.torque .* paramStruct.robot.i_g' .* 0; % Motor torque
TMotor = expData.temperature; % Motor temperature

u = timeseries(tauMotor,timeVec);
T = timeseries(TMotor,timeVec);

if FLEXIBLE_MODEL
    theta_0 = deg2rad([0; 90; 0; 0; 90; 0]);%deg2rad([expData.thetaLoad(1,:)'; expData.thetaMot(1,:)']);
    omega_0 = ones(6,1);
else
    theta_0 = deg2rad(expData.thetaMot(1,:)');
    omega_0 = zeros(3,1);
end

t_end = timeVec(end);
hFriction = 1/1000;