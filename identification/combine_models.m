%% Combine parameters of submodels
% Used for combining all models for simulation and control and to combine
% models needed for identification of the base parameters (FOR_BASE_PARAM
% flag)
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);

%% Settings
MODEL_FOLDER = 'identification/models';     % Folder of model

ROBOT_STRUCTURE = 1;        % Robot structure (0 = 500mm arms; 1 = 200mm arms)
FLEXIBLE_MODEL = 1;
FRICTION_MODEL = 2;         % Add different friction models (0 = all; 1 = static; 2 = lugre; 3 = gms)
FOR_BASE_PARAM = 0;         % Create complete parameter struct (0) or only for base parameter identification (1)
ADD_CONTROLLER = 1;         % Add controller gains

% Create parameter struct
paramStruct = struct;


%% Robot Kinematic Parameters

paramStruct.robot = loadRobotKinematic(ROBOT_STRUCTURE);
paramStruct.robot = rmfield(paramStruct.robot,'structure');

%% Robot Base Parameters

if ~FOR_BASE_PARAM
    mdlStruct = load([MODEL_FOLDER '/robot/model_robot.mat']);
    
    if FLEXIBLE_MODEL
        paramStruct.robot.baseParam = mdlStruct.param.baseParam;
    else
        paramStruct.robot.baseParam = mdlStruct.param.baseParam_RR;
    end

end

%% Joint Stiffness

% data sheet values
paramStruct.robot.k_dataSheet = [ ...
    31000 50000 57000; ...
    16000 25000 29000; ...
    4700 6100 7100 ...
    ] .* (pi/180);

for iJoint = 1:3

    mdlStruct = load([MODEL_FOLDER '/stiffness/model_stiffness_axis' num2str(iJoint) '.mat']);

    if iJoint == 1
        paramStruct.robot.c_k = mdlStruct.param.c_k;
        paramStruct.robot.k_thetaDiff_lim = mdlStruct.param.k_thetaDiff_lim;
    else
        paramStruct.robot.c_k(end+1,:) = mdlStruct.param.c_k;
        paramStruct.robot.k_thetaDiff_lim(end+1,:) = mdlStruct.param.k_thetaDiff_lim;
    end

end

idxInf = paramStruct.robot.k_thetaDiff_lim == Inf;
paramStruct.robot.k_thetaDiff_lim(:,all(idxInf,1)) = [];
paramStruct.robot.k_thetaDiff_lim(paramStruct.robot.k_thetaDiff_lim == Inf) = 10^10;

%% Kinematic Error

sizeVecLookup = zeros(3,1);
sizeVecPError = zeros(3,1);

for iJoint = 1:3

    mdlStruct = load([MODEL_FOLDER '/kinError/model_kinError_axis' num2str(iJoint) '.mat']);
    
    sizeVecLookup(iJoint) = length(mdlStruct.param.xLookup);

    if isfield(mdlStruct.param,'c_pError')
        sizeVecPError(iJoint) = length(mdlStruct.param.c_pError);
    end

end

[~, idxMaxLengthLookup] = max(sizeVecLookup);
[~, idxMaxLengthPError] = max(sizeVecPError);

for iJoint = 1:3

    mdlStruct = load([MODEL_FOLDER '/kinError/model_kinError_axis' num2str(iJoint) '.mat']);

    xLookup_temp = mdlStruct.param.xLookup;
    yLookup_temp = mdlStruct.param.yLookup;

    if iJoint ~= idxMaxLengthLookup

        lengthOffset = 0.5 .* (sizeVecLookup(idxMaxLengthLookup) - length(xLookup_temp));
        deltaXLookup = xLookup_temp(2) - xLookup_temp(1);

        xLookup_temp = [ ...
            (xLookup_temp(1)-lengthOffset*deltaXLookup):deltaXLookup:(xLookup_temp(1)-deltaXLookup), ...
            xLookup_temp, ...
            (xLookup_temp(end)+deltaXLookup):deltaXLookup:(xLookup_temp(end)+lengthOffset*deltaXLookup) ...
            ];
        yLookup_temp = [zeros(1,lengthOffset), yLookup_temp, zeros(1,lengthOffset)];

    end

    if iJoint == idxMaxLengthPError

        c_pError_temp = mdlStruct.param.c_pError;

    else

        if isfield(mdlStruct.param,'c_pError')
            c_pError_temp = mdlStruct.param.c_pError; 
            lengthOffset = sizeVecPError(idxMaxLengthLookup) - length(c_pError_temp);
            c_pError_temp = [c_pError_temp, zeros(1,lengthOffset)];
        else
            c_pError_temp = zeros(1,sizeVecPError(idxMaxLengthPError));
        end

    end

    if isfield(mdlStruct.param,'c_pError')
        hasPError_temp = 1;
    else
        hasPError_temp = 0;
    end

    if iJoint == 1

        paramStruct.kinError.xLookup = xLookup_temp;
        paramStruct.kinError.yLookup = yLookup_temp;
        paramStruct.kinError.hasPError = hasPError_temp;
        paramStruct.kinError.c_pError = c_pError_temp;
        %paramStruct.kinError.backlash = mdlStruct.param.backlash;

    else

        paramStruct.kinError.xLookup(end+1,:) = xLookup_temp;
        paramStruct.kinError.yLookup(end+1,:) = yLookup_temp;
        paramStruct.kinError.hasPError(end+1,:) = hasPError_temp;
        paramStruct.kinError.c_pError(end+1,:) = c_pError_temp;
        %paramStruct.kinError.backlash(end+1,:) = mdlStruct.param.backlash;

    end

end


%% Static Friction

for iJoint = 1:3

    mdlStruct_omegaT = load([MODEL_FOLDER '/staticFriction/model_staticFriction_omegaT_axis' num2str(iJoint) '.mat']);
    mdlStruct_load = load([MODEL_FOLDER '/staticFriction/model_staticFriction_load_axis' num2str(iJoint) '.mat']);

    if iJoint == 1

        paramStruct.staticFriction.c_c_pos = mdlStruct_omegaT.param.c_c_pos;
        paramStruct.staticFriction.c_c_neg = mdlStruct_omegaT.param.c_c_neg;
        paramStruct.staticFriction.c_load = mdlStruct_load.param.c_load;
        paramStruct.staticFriction.c_omega_pos = mdlStruct_omegaT.param.c_omega_pos;
        paramStruct.staticFriction.c_omega_neg = mdlStruct_omegaT.param.c_omega_neg;
        paramStruct.staticFriction.c_T_pos = mdlStruct_omegaT.param.c_T_pos;
        paramStruct.staticFriction.c_T_neg = mdlStruct_omegaT.param.c_T_neg;

    else
        
        paramStruct.staticFriction.c_c_pos(end+1,:) = mdlStruct_omegaT.param.c_c_pos;
        paramStruct.staticFriction.c_c_neg(end+1,:) = mdlStruct_omegaT.param.c_c_neg;
        paramStruct.staticFriction.c_load(end+1,:) = mdlStruct_load.param.c_load;
        paramStruct.staticFriction.c_omega_pos(end+1,:) = mdlStruct_omegaT.param.c_omega_pos;
        paramStruct.staticFriction.c_omega_neg(end+1,:) = mdlStruct_omegaT.param.c_omega_neg;
        paramStruct.staticFriction.c_T_pos(end+1,:) = mdlStruct_omegaT.param.c_T_pos;
        paramStruct.staticFriction.c_T_neg(end+1,:) = mdlStruct_omegaT.param.c_T_neg;

    end

end


%% Dynamic Friction

if FRICTION_MODEL == 0 || FRICTION_MODEL == 2 % Add LuGre

    for iJoint = 1:3

        mdlStruct = load([MODEL_FOLDER '/dynamicFriction/model_lugreFriction_axis' num2str(iJoint) '.mat']);

        if iJoint == 1

            paramStruct.lugreObserver.sigma_0 = mdlStruct.param.sigma_0;
            %paramStruct.lugreObserver.sigma_1 = mdlStruct.param.sigma_1;

        else

            paramStruct.lugreObserver.sigma_0(end+1,:) = mdlStruct.param.sigma_0;
            %paramStruct.lugreObserver.sigma_1(end+1,:) = mdlStruct.param.sigma_1;

        end

    end

elseif FRICTION_MODEL == 0 || FRICTION_MODEL == 3 % Add GMS

    for iJoint = 1:3

        mdlStruct = load([MODEL_FOLDER '/dynamicFriction/model_gmsFriction_axis' num2str(iJoint) '.mat']);

        if iJoint == 1

            paramStruct.gmsObserver.C = mdlStruct.param.C;
            paramStruct.gmsObserver.sigma_0 = mdlStruct.param.sigma_0;
            paramStruct.gmsObserver.sigma_1 = mdlStruct.param.sigma_1;

        else

            paramStruct.gmsObserver.C(end+1,:) = mdlStruct.param.C;
            paramStruct.gmsObserver.sigma_0(end+1,:) = mdlStruct.param.sigma_0;
            paramStruct.gmsObserver.sigma_1(end+1,:) = mdlStruct.param.sigma_1;

        end

    end

end

if ADD_CONTROLLER

    if FLEXIBLE_MODEL

        paramStruct.controller.k_theta = ones(3,1);
        paramStruct.controller.k_omega = ones(3,1);
        paramStruct.controller.k_domega = ones(3,1);
        paramStruct.controller.k_ddomega = ones(3,1);

    else

        paramStruct.controller.k_theta = ones(3,1);
        paramStruct.controller.k_omega = ones(3,1);

    end

    paramStruct.controller.cFilter = 1;

end


%% Save parameter struct

if FOR_BASE_PARAM
    save([MODEL_FOLDER '/model_forBaseParam.mat'],'-struct','paramStruct');
else
    save([MODEL_FOLDER '/model_complete.mat'],'-struct','paramStruct');
end
