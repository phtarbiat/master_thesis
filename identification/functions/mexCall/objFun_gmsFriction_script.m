% OBJFUN_GMSFRICTION_SCRIPT   Generate MEX-function objFun_gmsFriction_mex from
%  objFun_gmsFriction.
% 
% Script generated from project 'objFun_gmsFriction.prj' on 15-Dec-2022.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.


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

expData = load([INPUT_FOLDERS{AXIS} '/expData.mat']);
timeVec = expData.t; % Time
nSamples = length(timeVec);


%% Create configuration object of class 'coder.MexCodeConfig'.
cfg = coder.config('mex');
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;

%% Define argument types for entry-point 'objFun_gmsFriction'.
ARGS = cell(1,1);
ARGS{1} = cell(3,1);
ARGS_1_1 = struct;
ARGS_1_1_staticFriction = struct;
ARGS_1_1_staticFriction.c_c_pos = coder.typeof(0,[1 2]);
ARGS_1_1_staticFriction.c_c_neg = coder.typeof(0,[1 2]);
ARGS_1_1_staticFriction.c_stri_pos = coder.typeof(0,[1 3]);
ARGS_1_1_staticFriction.c_stri_neg = coder.typeof(0,[1 3]);
ARGS_1_1_staticFriction.c_omega_pos = coder.typeof(0,[1 3]);
ARGS_1_1_staticFriction.c_omega_neg = coder.typeof(0,[1 3]);
ARGS_1_1_staticFriction.c_T_pos = coder.typeof(0,[1 2]);
ARGS_1_1_staticFriction.c_T_neg = coder.typeof(0,[1 2]);
ARGS_1_1_staticFriction.c_load = coder.typeof(0);
ARGS_1_1.staticFriction = coder.typeof(ARGS_1_1_staticFriction);
ARGS_1_1_gmsObserver = struct;
ARGS_1_1_gmsObserver.n = coder.typeof(0);
ARGS_1_1_gmsObserver.C = coder.typeof(0);
ARGS_1_1_gmsObserver.sigma_0 = coder.typeof(0,[1 4]);
ARGS_1_1_gmsObserver.alpha = coder.typeof(0,[1 4]);
ARGS_1_1.gmsObserver = coder.typeof(ARGS_1_1_gmsObserver);
ARGS{1}{1} = coder.typeof(ARGS_1_1);
ARGS_1_2 = struct;
ARGS_1_2.timeVec = coder.typeof(0,[nSamples    1]);
ARGS_1_2.TMotor = coder.typeof(0,[nSamples    1]);
ARGS_1_2.tauMotor = coder.typeof(0,[nSamples    1]);
ARGS_1_2.omegaMotor = coder.typeof(0,[nSamples    1]);
ARGS{1}{2} = coder.typeof(ARGS_1_2);
ARGS{1}{3} = coder.typeof(0);

%% Invoke MATLAB Coder.
cd('F:\Matlab\geometric-robot-modeling-main\robot_modeling_flexible\identification\functions');
codegen -config cfg objFun_gmsFriction -args ARGS{1}

