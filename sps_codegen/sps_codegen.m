%% Build C++ function files of Matlab functions for use on the robots PLU
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Settings
OUTPUT_FOLDER = 'sps_codegen';
FLEXIBLE_MODEL = 0;


%% Create Output Folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end
cd(OUTPUT_FOLDER); % Change to output folder


%% Create configuration object of class 'coder.EmbeddedCodeConfig'.
cfg = coder.config('lib','ecoder',true);
cfg.TargetLang = 'C++';
cfg.FilePartitionMethod = 'SingleFile';
cfg.GenCodeOnly = true;
cfg.GenerateReport = true;
cfg.MaxIdLength = 1024;
cfg.ReportPotentialDifferences = false;

%% Create necessary variables


% Step size
h = 1/500; 

% Load model parameters
paramStruct = load('identification/models/model_complete.mat');
g = paramStruct.robot.g;
r_P1P2 = paramStruct.robot.r_P1P2;
r_P2P3 = paramStruct.robot.r_P2P3;
r_P3P4 = paramStruct.robot.r_P3P4;
baseParam = paramStruct.robot.baseParam;
c_k = paramStruct.robot.c_k;

% Input signals
theta_k = zeros(6,1); % Position
omega_k = zeros(6,1); % Velocity
T_k = zeros(3,1); % Temperature

% Desired trajectory
thetaLoad_des_k = zeros(3,1);
omegaLoad_des_k = zeros(3,1);
domegaLoad_des_k = zeros(3,1);
ddomegaLoad_des_k = zeros(3,1);
dddomegaLoad_des_k = zeros(3,1);

% Previous time step values
kinError_pk = zeros(3,1);
thetaMotor_des_pk = zeros(3,1);
omegaMotor_des_pk = zeros(3,1);
omegaLoad_pk = zeros(3,1);
omegaMotor_pk = zeros(3,1);
tauFVW_pk = zeros(3,1);
z_pk = zeros(3,1);
dz_pk = zeros(3,1);
uDisObs_pk = zeros(3,1);

%% Testrun code


if FLEXIBLE_MODEL
    % Kinematic Error and Stiffness
    % Kinematic Error
    kinError_k = kinError_fun(paramStruct, theta_k(4:6));
    dkinError_k = discreteDiff_fun(h, kinError_k, kinError_pk);
    
    % Stiffness
    [thetaDiffE_k, omegaDiffE_k] = compKinError_fun(theta_k, omega_k, kinError_k, dkinError_k);
    k_k = stiffness_fun(paramStruct, thetaDiffE_k);
    
    % Stiffness Torque
    [tauE_k, dtauE_k] = stiffnessTorque_fun(k_k, thetaDiffE_k, omegaDiffE_k);
    
    
    % Feedback Linearization
    % Load Acceleration and Jerk
    domegaLoad_k = ...
        directDynamicsLoad_reduced_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta_k(1:3), omega_k(1:3), tauE_k);
    ddomegaLoad_k = ...
        ddirectDynamicsLoad_reduced_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta_k(1:3), omega_k(1:3), domegaLoad_k, dtauE_k);
    
    % Feedback Linearization
    uFbackLin_k = ...
        feedbackLinearization_model(paramStruct, k_k, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k, ddomegaLoad_des_k, dddomegaLoad_des_k, theta_k(1:3), omega_k(1:3), domegaLoad_k, ddomegaLoad_k);
else
    % Feedback Linearization
    uFbackLin_rigid_k = feedbackLinearization_rigid_model(paramStruct, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k, theta_k(1:3), omega_k(1:3));
end


% Gravity Compensation
uGrav_k = gravityCompensation_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta_k(1:3));


% Feedforward Friction Compensation
% Desired Load Torque
tauLoad_des_k = ...
    inverseDynamics_rigid_fun(g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k, zeros(3,1));

if FLEXIBLE_MODEL
    % Desired Joint Stiffness
    k_des_k = stiffness_des_fun(c_k, tauLoad_des_k);
    
    % Desired Motor Trajectory
    [thetaMotor_des_k, omegaMotor_des_k] = ...
        desiredMotorTraj_fun(paramStruct, k_des_k, h, thetaLoad_des_k, omegaLoad_des_k, domegaLoad_des_k, thetaMotor_des_pk);
end

% LuGre Friction Model
[tauF_k, tauFVW_k, z_k, dz_k] = ...
    lugreFriction_model_midpoint(paramStruct, h, omega_k(4:6), omegaMotor_pk, tauFVW_pk, tauLoad_des_k, T_k, z_pk, dz_pk);
% Static Friction Model
%tauF_k = staticFriction_model(paramStruct, omegaMotor_des_k, tauLoad_des_k, T_k);


% Disturbance Observer
if FLEXIBLE_MODEL
    uDisObs_k = disturbanceObserver_model(paramStruct, h, uFbackLin_k, omega_k(4:6), omegaMotor_pk, tauE_k, uDisObs_pk);
else
    uDisObs_rigid_k = disturbanceObserver_rigid_model(paramStruct, h, uFbackLin_rigid_k, theta_k(4:6), omega_k(4:6), omegaMotor_pk, uDisObs_pk);
end

%% Generate Code


if FLEXIBLE_MODEL

    % kinError_fun
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.h = h;
    argStruct.thetaMotor_k = theta_k(4:6);
    argStruct.kinError_pk = kinError_pk;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg kinError_fun -args ARGS{1}
    
    
    % compKinError_fun
    argStruct = struct;
    argStruct.theta_k = theta_k;
    argStruct.omega_k = omega_k;
    argStruct.kinError_k = kinError_k;
    argStruct.dkinError_k = dkinError_k;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg compKinError_fun -args ARGS{1}
    
    
    % stiffness_fun
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.thetaDiffE_k = thetaDiffE_k;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg stiffness_fun -args ARGS{1}
    
    
    % stiffnessTorque_fun
    argStruct = struct;
    argStruct.k_k = k_k;
    argStruct.thetaDiffE_k = thetaDiffE_k;
    argStruct.omegaDiffE_k = omegaDiffE_k;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg stiffnessTorque_fun -args ARGS{1}
    
    
    % directDynamicsLoad_reduced_fun
    argStruct = struct;
    argStruct.g = g;
    argStruct.r_P1P2 = r_P1P2;
    argStruct.r_P2P3 = r_P2P3;
    argStruct.r_P3P4 = r_P3P4;
    argStruct.baseParam = baseParam;
    argStruct.thetaLoad_k = theta_k(1:3);
    argStruct.omegaLoad_k = omega_k(1:3);
    argStruct.tauE_k = tauE_k(1:3);
    
    ARGS = createArguments(argStruct);
    codegen -config cfg directDynamicsLoad_reduced_fun -args ARGS{1}
    
    
    % ddirectDynamicsLoad_reduced_fun
    argStruct = struct;
    argStruct.g = g;
    argStruct.r_P1P2 = r_P1P2;
    argStruct.r_P2P3 = r_P2P3;
    argStruct.r_P3P4 = r_P3P4;
    argStruct.baseParam = baseParam;
    argStruct.thetaLoad_k = theta_k(1:3);
    argStruct.omegaLoad_k = omega_k(1:3);
    argStruct.domegaLoad_k = domegaLoad_k;
    argStruct.dtauE_k = dtauE_k(1:3);
    
    ARGS = createArguments(argStruct);
    codegen -config cfg ddirectDynamicsLoad_reduced_fun -args ARGS{1}
    
    
    % feedbackLinearization_model
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.k_k = k_k;
    argStruct.thetaLoad_des_k = thetaLoad_des_k;
    argStruct.omegaLoad_des_k = omegaLoad_des_k;
    argStruct.domegaLoad_des_k = domegaLoad_des_k;
    argStruct.ddomegaLoad_des_k = ddomegaLoad_des_k;
    argStruct.dddomegaLoad_des_k = dddomegaLoad_des_k;
    argStruct.thetaLoad_k = theta_k(1:3);
    argStruct.omegaLoad_k = omega_k(1:3);
    argStruct.domegaLoad_k = domegaLoad_k;
    argStruct.ddomegaLoad_k = ddomegaLoad_k;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg feedbackLinearization_model -args ARGS{1}

else

    % feedbackLinearization_rigid_model
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.thetaLoad_des_k = thetaLoad_des_k;
    argStruct.omegaLoad_des_k = omegaLoad_des_k;
    argStruct.domegaLoad_des_k = domegaLoad_des_k;
    argStruct.theta_k = theta_k(4:6);
    argStruct.omega_k = omega_k(4:6);
    
    ARGS = createArguments(argStruct);
    codegen -config cfg feedbackLinearization_rigid_model -args ARGS{1}

end


% gravityCompensation_fun
argStruct = struct;
argStruct.g = g;
argStruct.r_P1P2 = r_P1P2;
argStruct.r_P2P3 = r_P2P3;
argStruct.r_P3P4 = r_P3P4;
argStruct.baseParam = baseParam;
argStruct.thetaLoad_k = theta_k(1:3);

ARGS = createArguments(argStruct);
codegen -config cfg gravityCompensation_fun -args ARGS{1}


% inverseDynamics_rigid_fun
argStruct = struct;
argStruct.g = g;
argStruct.r_P1P2 = r_P1P2;
argStruct.r_P2P3 = r_P2P3;
argStruct.r_P3P4 = r_P3P4;
argStruct.baseParam = baseParam;
argStruct.thetaLoad_des_k = thetaLoad_des_k;
argStruct.omegaLoad_des_k = omegaLoad_des_k;
argStruct.domegaLoad_des_k = domegaLoad_des_k;
argStruct.tauF_k = tauF_k;

ARGS = createArguments(argStruct);
codegen -config cfg inverseDynamics_rigid_fun -args ARGS{1}


if FLEXIBLE_MODEL

    % stiffness_des_fun
    argStruct = struct;
    argStruct.c_k = c_k;
    argStruct.tauLoad_des_k = tauLoad_des_k;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg stiffness_des_fun -args ARGS{1}
    
    
    % desiredMotorTraj_fun
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.k_des_k = k_des_k;
    argStruct.h = h;
    argStruct.thetaLoad_des_k = thetaLoad_des_k;
    argStruct.omegaLoad_des_k = omegaLoad_des_k;
    argStruct.domegaLoad_des_k = domegaLoad_des_k;
    argStruct.thetaMotor_des_pk = thetaMotor_des_pk;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg desiredMotorTraj_fun -args ARGS{1}

end


% lugreFriction_model_midpoint
argStruct = struct;
argStruct.paramStruct = paramStruct;
argStruct.h = h;
argStruct.omega_des_k = omegaLoad_des_k;
argStruct.omega_des_pk = omegaMotor_des_pk;
argStruct.tauFVW_pk = tauFVW_pk;
argStruct.tauLoad_des_k = tauLoad_des_k;
argStruct.T_k = T_k;
argStruct.z_pk = z_pk;
argStruct.dz_pk = dz_pk;

ARGS = createArguments(argStruct);
codegen -config cfg lugreFriction_model_midpoint -args ARGS{1}

%{
% staticFriction_model
argStruct = struct;
argStruct.paramStruct = paramStruct;
argStruct.omegaMotor_des_k = omegaMotor_des_k;
argStruct.tauLoad_des_k = tauLoad_des_k;
argStruct.T_k = T_k;

ARGS = createArguments(argStruct);
codegen -config cfg staticFriction_model -args ARGS{1}
%}

if FLEXIBLE_MODEL

    % disturbanceObserver_model
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.h = h;
    argStruct.uFbackLin_k = uFbackLin_k;
    argStruct.omegaMotor_k = omega_k(4:6);
    argStruct.omegaMotor_pk = omegaMotor_pk;
    argStruct.tauE_k = tauE_k;
    argStruct.uDisObs_pk = uDisObs_pk;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg disturbanceObserver_model -args ARGS{1}

else

    % disturbanceObserver_rigid_model
    argStruct = struct;
    argStruct.paramStruct = paramStruct;
    argStruct.h = h;
    argStruct.uFbackLin_k = uFbackLin_rigid_k;
    argStruct.theta_k = theta_k(4:6);
    argStruct.omega_k = omega_k(4:6);
    argStruct.omega_pk = omegaMotor_pk;
    argStruct.uDisObs_pk = uDisObs_pk;
    
    ARGS = createArguments(argStruct);
    codegen -config cfg disturbanceObserver_rigid_model -args ARGS{1}

end

% Go back to original folder
cd ..

%%
function args = createArguments(argStruct)
% Create argument struct for use of struct data-type in Matlab Coder

    fieldsStruct = fieldnames(argStruct);

    args = cell(1,1);
    args{1} = cell(length(fieldsStruct),1);

    for iField = 1:length(fieldsStruct)

        curVar = argStruct.(fieldsStruct{iField});

        if isa(curVar,'double')

            if length(curVar) == 1
                args{1}{iField} = coder.typeof(0);
            else
                [nRow, nCol] = size(curVar);
                args{1}{iField} = coder.typeof(0,[nRow nCol]);
            end

        elseif isa(curVar,'struct')

            args{1}{iField} = coder.typeof(createStructArgument(curVar));

        else

            error('non-supported data type')

        end

    end

end


function argStruct = createStructArgument(inStruct)

    argStruct = struct;

    fieldsStruct = fieldnames(inStruct);

    for iField = 1:length(fieldsStruct)

        arg_temp = struct;

        elements = fieldnames(inStruct.(fieldsStruct{iField}));

        for iElement = 1:length(elements)

            curVar = inStruct.(fieldsStruct{iField}).(elements{iElement});

            if isa(curVar,'double')

                if length(curVar) == 1
                    arg_temp.(elements{iElement}) = coder.typeof(0);
                else
                    [nRow, nCol] = size(curVar);
                    arg_temp.(elements{iElement}) = coder.typeof(0,[nRow nCol]);
                end

            else

                error('non-supported data type')

            end

        end

        argStruct.(fieldsStruct{iField}) = coder.typeof(arg_temp);

    end

end