%% Identification of LuGre and GMS friction model
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Define global variables
global FRICTION_MODEL
global NUM_ELEMENTS_GMS
global paramStruct
global expData_sim
global h


% Identification settings
AXIS = 1;                                                       % Robot axis

INPUT_FOLDERS = { ...                                           % Measurment data folder
    'measurments/12_05_dynamicFriction_axis1' ...
    'measurments/11_16_dynamicFriction_axis2' ...
    'measurments/12_01_dynamicFriction_axis3' ...
    };
INPUT_FOLDER_MODEL = 'identification/models/staticFriction';    % Folder with static friction model
OUTPUT_FOLDER = 'identification/models/dynamicFriction';        % Output folder for model
SAVE_IDENTIFICATION_DATA = 1;                                   % Add identification data to output
SAVE_PLOT_DATA = 1;                                             % Add plotting data to output

SAMPLE_FREQ = 500;                                              % Sampling frequency of the measurment
TIME_AVERAGE = 0.05;                                            % Length of averaging window

FRICTION_MODEL = 1;                                             % Choosen dynamic friction model
NUM_ELEMENTS_GMS = 4;                                           % Number of elements of GMS friction model


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load measurment data and simulation parameter


% Step length numerical simulation
h = 1 / SAMPLE_FREQ;


% Load kinematic parameters
kinStruct = loadRobotKinematic(0);
i_g = kinStruct.i_g(AXIS);


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
tauMotor = expData.torque(:,AXIS);% .* i_g; % Motor torque

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

%% Identify dynamic friction model


if FRICTION_MODEL == 0 % LuGre

    % Define lower and upper bounds for optimization
    sigma_0_lb = 0;
    %sigma_1_lb = 0;
    lb = [sigma_0_lb];% sigma_1_lb];

    sigma_0_ub = Inf;
    %sigma_1_ub = Inf;
    ub = [sigma_0_ub];% sigma_1_ub];

    % Find optimal model parameters by particle swarm optimization
    options = optimoptions('particleswarm','Display','iter','SwarmSize',100);
    param = particleswarm(@objFun,length(lb),lb,ub,options)

elseif FRICTION_MODEL == 1 % GMS

    % Define lower and upper bounds for optimization
    C_lb = 0;
    sigma_0_lb = 0 .* ones(NUM_ELEMENTS_GMS,1);
    %sigma_1_lb = 0 .* ones(NUM_ELEMENTS_GMS,1);
    alpha_lb = 0 .* ones(NUM_ELEMENTS_GMS,1);
    lb = [C_lb; sigma_0_lb; alpha_lb]';

    C_ub = Inf;
    sigma_0_ub = Inf .* ones(NUM_ELEMENTS_GMS,1);
    %sigma_1_ub = 10 .* ones(NUM_ELEMENTS_GMS,1);
    alpha_ub = 1 .* ones(NUM_ELEMENTS_GMS,1);
    ub = [C_ub; sigma_0_ub; alpha_ub]';

    % Define parameter constraints
    Aeq = [zeros(1,length(C_lb)+length(sigma_0_lb)) ones(1,length(alpha_lb))];
    beq = 1;

    % Find optimal model parameters by genetic algorithm
    options = optimoptions('ga','Display','iter');
    param = ga(@objFun,length(lb),[],[],Aeq,beq,lb,ub,[],options)

else

    error('choosen model does not exist')

end

%% Calculate model prediction


% Write model parameters to parameter struct
if FRICTION_MODEL == 0 % LuGre

    paramStruct.lugreObserver.sigma_0 = param(1);
    %paramStruct.lugreObserver.sigma_1 = param(2);

elseif FRICTION_MODEL == 1 % GMS

    paramStruct.gmsObserver.n = NUM_ELEMENTS_GMS;
    paramStruct.gmsObserver.C = param(1);
    paramStruct.gmsObserver.sigma_0 = param(2:1+NUM_ELEMENTS_GMS);
    %paramStruct.gmsObserver.sigma_1 = param(2+NUM_ELEMENTS_GMS:1+2*NUM_ELEMENTS_GMS);
    paramStruct.gmsObserver.alpha = param(2+NUM_ELEMENTS_GMS:1+2*NUM_ELEMENTS_GMS);

end


% Load experiment data
omegaMotor = expData_sim.omegaMotor; % Motor velocity
T = expData_sim.TMotor; % Motor temperature

nSamples = length(omegaMotor); % Number of samples


% Initalize output vectors of simulation
tauF_mdl = zeros(nSamples,1);
if FRICTION_MODEL == 0 % LuGre
    z = zeros(nSamples,1);
elseif FRICTION_MODEL == 1 || FRICTION_MODEL == 2 % GMS
    z = zeros(nSamples,NUM_ELEMENTS_GMS);
    Q = zeros(nSamples,NUM_ELEMENTS_GMS);
end
tauF_mdlStatic = zeros(nSamples,1);


% Initialize memory values
omegaMotor_pk = 0;
tauFVW_pk = 0;

if FRICTION_MODEL == 0 % LuGre
    z_pk = 0;
    dz_pk = 0;
elseif FRICTION_MODEL == 1 || FRICTION_MODEL == 2 % GMS
    z_pk = zeros(1,NUM_ELEMENTS_GMS);
    dz_pk = zeros(1,NUM_ELEMENTS_GMS);
    Q_pk = zeros(1,NUM_ELEMENTS_GMS);
end


% Simulate dynamic friction model for the entire measurement
for iSample = 1:nSamples

    % Load current signal sample
    omegaMotor_k = omegaMotor(iSample);
    T_k = T(iSample);

    % Calculate output of dynamic friction model
    if FRICTION_MODEL == 0 % LuGre
        [tauF_mdl(iSample), tauFVW_k, z(iSample), dz_k] = ...
            lugreFriction_model_midpoint(paramStruct, h, omegaMotor_k, omegaMotor_pk, tauFVW_pk, 0, T_k, z_pk, dz_pk);
    elseif FRICTION_MODEL == 1 % GMS
        [tauF_mdl(iSample), tauFVW_k, z(iSample,:), dz_k, Q(iSample,:)] = ...
            gmsFriction_model_midpoint(paramStruct, h, omegaMotor_k, omegaMotor_pk, tauFVW_pk, 0, T_k, z_pk, dz_pk, Q_pk);
    end

    % Calculate output of static friction model
    tauF_mdlStatic(iSample) = staticFriction_model(paramStruct, omegaMotor_k, 0, T_k);

    % Update memory values
    omegaMotor_pk = omegaMotor_k;
    tauFVW_pk = tauFVW_k;
    z_pk = z(iSample,:);
    dz_pk = dz_k;
    if FRICTION_MODEL == 1 || FRICTION_MODEL == 2 % GMS
        Q_pk = Q(iSample,:);
    end

end


% Compare measurment data with model output
[mae, mape] = calcMAE(tauMotor_f.*i_g,tauF_mdl.*i_g)
[mse, mspe] = calcMSE(tauMotor_f.*i_g,tauF_mdl.*i_g)
maxAE = max(abs(tauMotor_f - tauF_mdl).*i_g)

[mae_static, mape_static] = calcMAE(tauMotor_f.*i_g,tauF_mdlStatic.*i_g)
[mse_static, mspe_static] = calcMSE(tauMotor_f.*i_g,tauF_mdlStatic.*i_g)
maxAE_static = max(abs(tauMotor_f.*i_g - tauF_mdlStatic.*i_g))


% Plot model
figure
hold on
plot(expData_sim.timeVec,tauMotor_f.*i_g)
plot(expData_sim.timeVec,tauF_mdl.*i_g,'--')
plot(expData_sim.timeVec,tauF_mdlStatic.*i_g,'--')
hold off
legend('meas','dynamicMdl','staticMdl')
xlabel('time / s')
ylabel('tau / Nm')

figure
plot(expData_sim.timeVec,z)

%% Save model


mdlStruct = struct;

if FRICTION_MODEL == 0 % LuGre

    mdlStruct.param.sigma_0 = param(1);
    %mdlStruct.param.sigma_1 = param(2);

elseif FRICTION_MODEL == 1 % GMS

    mdlStruct.param.n = NUM_ELEMENTS_GMS;
    mdlStruct.param.C = param(1);
    mdlStruct.param.sigma_0 = param(2:1+NUM_ELEMENTS_GMS);
    %mdlStruct.param.sigma_1 = param(2+NUM_ELEMENTS_GMS:1+2*NUM_ELEMENTS_GMS);
    mdlStruct.param.alpha = param(2+NUM_ELEMENTS_GMS:1+2*NUM_ELEMENTS_GMS);

end

if SAVE_IDENTIFICATION_DATA

    mdlStruct.data.omegaMotor = expData_sim.omegaMotor;
    mdlStruct.data.TMotor = expData_sim.TMotor;
    mdlStruct.data.tauMotor = expData_sim.tauMotor;

    mdlStruct.metrics.mae = mae;
    mdlStruct.metrics.mape = mape;
    mdlStruct.metrics.msae = mse;
    mdlStruct.metrics.msape = mspe;
    mdlStruct.metrics.maxAE = maxAE;

    mdlStruct.metrics.mae_static = mae_static;
    mdlStruct.metrics.mape_static = mape_static;
    mdlStruct.metrics.msae_static = mse_static;
    mdlStruct.metrics.msape_static = mspe_static;
    mdlStruct.metrics.maxAE_static = maxAE_static;

end

if SAVE_PLOT_DATA

    mdlStruct.data.timeVec = expData_sim.timeVec;
    mdlStruct.data.omegaMotor = expData_sim.omegaMotor;

    mdlStruct.data.tauF_mdl = tauF_mdl;
    mdlStruct.data.tauF_mdlStatic = tauF_mdlStatic;

end

if FRICTION_MODEL == 0 % LuGre

    save([OUTPUT_FOLDER '/model_lugreFriction_axis' num2str(AXIS) '.mat'],'-struct','mdlStruct');

elseif FRICTION_MODEL == 1 % GMS

    save([OUTPUT_FOLDER '/model_gmsFriction_axis' num2str(AXIS) '.mat'],'-struct','mdlStruct');

elseif FRICTION_MODEL == 2 % GMS  alternative

    save([OUTPUT_FOLDER '/model_gmsFrictionAlt_axis' num2str(AXIS) '.mat'],'-struct','mdlStruct');

end

%%
function objFunValue = objFun(paramArray)
% Function handed over to optimisation algorithms, that runs a numerical
% simulation of the dynamic friction model and returns the according
% value of the objective function

    % Define global variables
    global FRICTION_MODEL
    global NUM_ELEMENTS_GMS
    global paramStruct
    global expData_sim
    global h


    if FRICTION_MODEL == 0 % LuGre

        % Write current model parameters to paramStruct
        paramStruct.lugreObserver.sigma_0 = paramArray(1);
        %paramStruct.lugreObserver.sigma_1 = paramArray(2);

        % Run numeric simualtion and calculate objective function value
        % (SE)
        objFunValue = objFun_lugreFriction_mex(paramStruct, expData_sim, h);

    elseif FRICTION_MODEL == 1 % GMS

        % Write current model parameters to parameter struct
        paramStruct.gmsObserver.n = NUM_ELEMENTS_GMS;
        paramStruct.gmsObserver.C = paramArray(1);
        paramStruct.gmsObserver.sigma_0 = paramArray(2:1+NUM_ELEMENTS_GMS);
        %paramStruct.gmsObserver.sigma_1 = paramArray(2+NUM_ELEMENTS_GMS:1+2*NUM_ELEMENTS_GMS);
        paramStruct.gmsObserver.alpha = paramArray(2+NUM_ELEMENTS_GMS:1+2*NUM_ELEMENTS_GMS);
        
        % Run numeric simualtion and calculate objective function value
        % (SE)
        objFunValue = objFun_gmsFriction_mex(paramStruct, expData_sim, h);

    end

end