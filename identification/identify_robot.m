%% Identification of base parameters
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Identification settings 
ROBOTER_STRUCTURE = 1;                              % Robot structure (0 = 500mm arms; 1 = 200mm arms)

INPUT_FOLDER = 'measurments/12_08_robot';           % Measurment data folder
INPUT_FOLDER_MODEL = 'identification/models';       % Folder with identified model parameters
INPUT_FOLER_ROBOT_MODEL = 'model';                  % Folder of the robots symbolic model
OUTPUT_FOLDER = 'identification/models/robot';      % Output folder for model 
SAVE_IDENTIFICATION_DATA = 1;                       % Add identification data to output
SAVE_PLOT_DATA = 1;                                 % Add plotting data to output
GENERATE_Y_FUN = 1;                                 % Generate function script for regressor matrix

SAMPLE_FREQ = 500;                                  % Sampling frequency of the measurment data
TIME_AVERAGE = 0.05;                                % Length of averaging window for filtering data


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load data


% Load already identified model parameters
paramStruct = load([INPUT_FOLDER_MODEL '/model_forBaseParam.mat']);
paramStruct.kinError.addKinError = ones(3,1);


% Load robot model
mdl_robot = load([INPUT_FOLER_ROBOT_MODEL '/mdl_flexible_joint_robot_EL.mat']);


% Load and extract measurment data
expData = load([INPUT_FOLDER, '/expData_',num2str(1),'.mat']);

timeVec = expData.t; % Time
thetaLoad = expData.thetaLoad; % Load position
thetaMotor = expData.thetaMot; % Motor position
tauMotor = expData.torque .* paramStruct.robot.i_g'; % Motor torque
TMotor = expData.temperature; % Motor temperature
thetaMotor_traj = deg2rad(expData.refValuePos); % Reference trajectory

nSamples = length(timeVec); % Number of samples

% Filter position and torque with averaging window
thetaLoad_f = movmean(thetaLoad,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_f = movmean(thetaMotor,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_f = movmean(tauMotor,TIME_AVERAGE*SAMPLE_FREQ);

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

%% Process data


%tauMotor_f = tauMotor_f - mean(tauMotor_f([1:SAMPLE_FREQ end-SAMPLE_FREQ:end],:),1);


% Cut out measurment time with no robot movement
%idxStart = find(thetaMotor_traj(:,1) ~= 0,1);
idxStart = zeros(3,1);
idxStart(1) = find(abs(omegaMotor_f(:,1)) >= 0.1,1);
idxStart(2) = find(abs(omegaMotor_f(:,2)) >= 0.1,1);
idxStart(3) = find(abs(omegaMotor_f(:,3)) >= 0.1,1);
idxStart = min(idxStart);
idxEnd = zeros(3,1);
idxEnd(1) = find(abs(omegaMotor_f(:,1)) >= 0.1,1,'last');
idxEnd(2) = find(abs(omegaMotor_f(:,2)) >= 0.1,1,'last');
idxEnd(3) = find(abs(omegaMotor_f(:,3)) >= 0.1,1,'last');
idxEnd = max(idxEnd);

timeVec = timeVec(idxStart:idxEnd);
thetaLoad_f = thetaLoad_f(idxStart:idxEnd,:);
thetaMotor_f = thetaMotor_f(idxStart:idxEnd,:);
omegaLoad_f = omegaLoad_f(idxStart:idxEnd,:);
omegaMotor_f = omegaMotor_f(idxStart:idxEnd,:);
domegaLoad_f = domegaLoad_f(idxStart:idxEnd,:);
domegaMotor_f = domegaMotor_f(idxStart:idxEnd,:);
tauMotor_f = tauMotor_f(idxStart:idxEnd,:);
TMotor = TMotor(idxStart:idxEnd,:);


% Recalculate number of samples
nSamples = length(timeVec);

%% Create regression matrix function


if GENERATE_Y_FUN

    Y = mdl_robot.regMat.Ybase_red;

    vars_sym = { ...
    mdl_robot.params.kinematic(1), mdl_robot.params.kinematic(2:4), ...
    mdl_robot.params.kinematic(5:7), mdl_robot.params.kinematic(8:10), mdl_robot.params.kinematic(11:13), ...
    ...
    mdl_robot.vars.theta, mdl_robot.vars.omega, mdl_robot.vars.domega ...
    };

    % Build function of regressor matrix
    MDL_NAME = 'YFun';
    matlabFunction(Y, ...
        'File', ['identification/functions/', MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'Y'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: YFun'], ...
        ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, theta, omega, domega', '', ''
        });

end

%% Create observation matrix from measurment data


% Extract kinematic parameters
g = paramStruct.robot.g;
i_g = paramStruct.robot.i_g;
r_P1P2 = paramStruct.robot.r_P1P2;
r_P2P3 = paramStruct.robot.r_P2P3;
r_P3P4 = paramStruct.robot.r_P3P4;


% Get size of regressor matrix and initialize observation matrix and torque
% vector
Y_i = YFun(g,i_g,r_P1P2,r_P2P3,r_P3P4,zeros(6,1),zeros(6,1),zeros(6,1));

nRow = 0.5 * size(Y_i,1); % Number of rows of Y
nCol = size(Y_i,2); % Number of columns of Y

W = zeros(nRow*nSamples,nCol); % Observation matrix of FJR
W_RR = zeros(nRow*nSamples,nCol-3); % Observation matrix of RR
tauVec = zeros(nRow*nSamples,1); % Torque vector of FJR and RR


% Initialize memory values of LuGre friction model
omegaMotor_pk = zeros(3,1);
tauFVW_pk = zeros(3,1);
z_pk = zeros(3,1);
dz_pk = zeros(3,1);

tauF = zeros(nSamples,3);

%{
W_load = zeros(nRow*nSamples,nCol+3); % Observation matrix RR
W_motor = zeros(nRow*nSamples,nCol+3); % Observation matrix RR
tauVec_load = zeros(nRow*nSamples,1); % Torque vector FJR/RR
tauVec_motor = zeros(nRow*nSamples,1); % Torque vector FJR/RR

tauFVW_temp = zeros(nSamples,3);
z_temp = zeros(nSamples,3);
dz_temp = zeros(nSamples,3);
tauE = zeros(nSamples,3);

tauFVW = zeros(3,1);
z = zeros(3,1);
dz = zeros(3,1);
%}


% Iterate through measurment samples
for iSample = 1:nSamples
    
    % Load input data of current sample
    theta_i = [thetaLoad_f(iSample,:)'; thetaMotor_f(iSample,:)']; % Position
    omega_i = [omegaLoad_f(iSample,:)'; omegaMotor_f(iSample,:)']; % Velocity
    domega_i = [domegaLoad_f(iSample,:)'; domegaMotor_f(iSample,:)']; % Acceleration
    tauMotor_i = tauMotor_f(iSample,:)'; % Motor torque
    TMotor_i = TMotor(iSample,:)'; % Motor temperature


    % Get rows of observation matrix and torque vector to write to according to current sample
    posRow = ((iSample - 1)*nRow + 1):(iSample*nRow);
    

    % Calculate friction torque twice:
    % 1) Calculate friction torque without load dependency and subract it from the motor torque
    %    to get an approximation of the load torque
    % 2) Calculate friction torque with load dependency
    tauF_i = tauMotor_i;
    for i = 1:2
        [tauF_i, tauFVW, z, dz] = ...
            lugreFriction_model_midpoint(paramStruct, 1/SAMPLE_FREQ, omega_i(4:6), omegaMotor_pk, tauFVW_pk, (tauMotor_i-tauF_i)./i_g, TMotor_i, z_pk, dz_pk);
        tauF_i = tauF_i .* i_g;
    end
    tauF(iSample,:) = tauF_i';
    

    % Write to torque vector
    tauVec(posRow) = tauMotor_i - tauF_i;


    % Write to observation matrix of FJR
    Y_i = YFun(g,i_g,r_P1P2,r_P2P3,r_P3P4,deg2rad(theta_i),deg2rad(omega_i),deg2rad(domega_i));
    W(posRow,:) = Y_i(1:3,:) + Y_i(4:6,:);

    % Write to observation matrix of RR
    Y_RR_i = YFun(g,i_g,r_P1P2,r_P2P3,r_P3P4,deg2rad([theta_i(4:6); theta_i(4:6)]),deg2rad([omega_i(4:6); omega_i(4:6)]),deg2rad([domega_i(4:6); domega_i(4:6)]));
    Y_RR_i(4:6,:) = []; % Only consider link equations
    Y_RR_i(:,[2 10 18]) = []; % Delete entries of motor inertia
    W_RR(posRow,:) = Y_RR_i;


    % Save memory values of LuGre friction model
    omegaMotor_pk = omega_i(4:6);
    tauFVW_pk = tauFVW;
    z_pk = z;
    dz_pk = dz;

    %{
    Yk_i = [ ...
        theta_i(4)-theta_i(1)-kinError_i(1) 0 0; ...
        0 theta_i(5)-theta_i(2)-kinError_i(2) 0; ...
        0 0 theta_i(6)-theta_i(3)-kinError_i(3) ...
        ];

    W_load(posRow,:) = [Y_i(1:3,:) -Yk_i];
    tauVec_load(posRow) = zeros(3,1);
    W_motor(posRow,:) = [Y_i(4:6,:) Yk_i];
    tauVec_motor(posRow) = tauMotor_i - curTauF;

    tauFVW_temp(iSample,:) = tauFVW';
    z_temp(iSample,:) = z';
    dz_temp(iSample,:) = dz';
    tauF(iSample,:) = curTauF';
    %}
end


% Calculate condition number of observation matrices
conW = cond(W) % FJR
conW_RR = cond(W_RR) % RR


% Calculate base parameters with cosindering tauE
if SAVE_PLOT_DATA

    W_tauE = zeros(2*nRow*nSamples,nCol); % Observation matrix of FJR
    tauVec_tauE = zeros(2*nRow*nSamples,1); % Torque vector of FJR

    W_tauE_l = zeros(nRow*nSamples,nCol); % Observation matrix of FJR
    W_tauE_m = zeros(nRow*nSamples,nCol); % Observation matrix of FJR
    tauVec_tauE_l = zeros(nRow*nSamples,1); % Torque vector of FJR
    tauVec_tauE_m = zeros(nRow*nSamples,1); % Torque vector of FJR

    tauE = zeros(nSamples,3);

    for iSample = 1:nSamples
    
        % Load input data of current sample
        theta_i = [thetaLoad_f(iSample,:)'; thetaMotor_f(iSample,:)']; % Position
        omega_i = [omegaLoad_f(iSample,:)'; omegaMotor_f(iSample,:)']; % Velocity
        domega_i = [domegaLoad_f(iSample,:)'; domegaMotor_f(iSample,:)']; % Acceleration
        tauMotor_i = tauMotor_f(iSample,:)'; % Motor torque
        TMotor_i = TMotor(iSample,:)'; % Motor temperature
        tauF_i = tauF(iSample,:)'; % Friction torque


        % Get rows of observation matrix and torque vector to write to according to current sample
        posRow = ((iSample - 1)*2*nRow + 1):(iSample*2*nRow);
        posRow_ = ((iSample - 1)*nRow + 1):(iSample*nRow);


        % Calculate joint stiffness torque
        % Calculate kinematic error and according position deflection
        kinError_i = kinError_fun(paramStruct, theta_i(4:6));
        [thetaDiffE_i, ~] = compKinError_fun(theta_i, zeros(6,1), kinError_i, zeros(3,1));
        % Calculate joint stiffness
        degK_i = stiffness_fun(paramStruct, thetaDiffE_i);
        % Calculate joint stiffness torque
        tauE_i = degK_i .* thetaDiffE_i;
        tauE(iSample,:) = tauE_i';

        % Write to torque vector
        tauVec_tauE(posRow) = [tauE_i; tauMotor_i - tauE_i - tauF_i];
        tauVec_tauE_l(posRow_) = tauE_i;
        tauVec_tauE_m(posRow_) = tauMotor_i - tauE_i - tauF_i;

        % Write to observation matrix of FJR
        Y_i = YFun(g,i_g,r_P1P2,r_P2P3,r_P3P4,deg2rad(theta_i),deg2rad(omega_i),deg2rad(domega_i));
        W_tauE(posRow,:) = Y_i;
        W_tauE_l(posRow_,:) = Y_i(1:3,:);
        W_tauE_m(posRow_,:) = Y_i(4:6,:);

    end

    W_tauE_l(:,[2 10 18]) = [];
    W_tauE_m = W_tauE_m(:,[2 10 18]);


    % Calculate condition number of observation matrices
    conW_tauE = cond(W_tauE) % FJR
    conW_tauE_l = cond(W_tauE_l) % FJR
    conW_tauE_m = cond(W_tauE_m) % FJR

end


%% Identify  Base Parameters


% Calculate base parameters by linear least square method
param = ((W' * W) \ W') * tauVec % FJR
param_RR = ((W_RR' * W_RR) \ W_RR') * tauVec % RR


% Get base parameters of CAD model
kinStruct = loadRobotKinematic(ROBOTER_STRUCTURE);
paramStruct_CAD = loadRobotDynamicCAD(ROBOTER_STRUCTURE,kinStruct,1);
param_CAD = paramStruct_CAD.baseParam


% Calculate model predition
tauVec_mdl = W * param; % FJR
tauVec_mdl_RR = W_RR * param_RR; % RR
tauVec_mdl_CAD = W * param_CAD; % CAD


% Seperate model output for the 3 robot joints
tauMotor_mdl = [tauVec_mdl(1:3:3*nSamples), tauVec_mdl(2:3:3*nSamples), tauVec_mdl(3:3:3*nSamples)];
tauMotor_mdl_RR = [tauVec_mdl_RR(1:3:3*nSamples), tauVec_mdl_RR(2:3:3*nSamples), tauVec_mdl_RR(3:3:3*nSamples)];
tauMotor_mdl_CAD = [tauVec_mdl_CAD(1:3:3*nSamples), tauVec_mdl_CAD(2:3:3*nSamples), tauVec_mdl_CAD(3:3:3*nSamples)];


if SAVE_PLOT_DATA

    % Calculate base parameters by linear least square method
    param_tauE = ((W_tauE' * W_tauE) \ W_tauE') * tauVec_tauE % FJR


    % Calculate model predition
    tauVec_mdl_tauE = W_tauE * param_tauE; % FJR


    % Seperate model output for the 3 robot joints
    tauMotor_mdl_tauE = [tauVec_mdl_tauE(4:6:6*nSamples), tauVec_mdl_tauE(5:6:6*nSamples), tauVec_mdl_tauE(6:6:6*nSamples)];

end

% Compare measurment data with model output
[mae, mape] = calcMAE(tauMotor_f(:),tauMotor_mdl(:)+tauF(:))
[mse, mspe] = calcMSE(tauMotor_f(:),tauMotor_mdl(:)+tauF(:))
maxAE = max(abs(tauMotor_f(:) - (tauMotor_mdl(:)+tauF(:))))

[mae_RR, mape_RR] = calcMAE(tauMotor_f(:),tauMotor_mdl_RR(:)+tauF(:))
[mse_RR, mspe_RR] = calcMSE(tauMotor_f(:),tauMotor_mdl_RR(:)+tauF(:))
maxAE_RR = max(abs(tauMotor_f(:) - (tauMotor_mdl_RR(:)+tauF(:))))

[mae_CAD, mape_CAD] = calcMAE(tauMotor_f(:),tauMotor_mdl_CAD(:)+tauF(:))
[mse_CAD, mspe_CAD] = calcMSE(tauMotor_f(:),tauMotor_mdl_CAD(:)+tauF(:))
maxAE_CAD = max(abs(tauMotor_f(:) - (tauMotor_mdl_CAD(:)+tauF(:))))


% Plot measurement and model data
figure
hold on
plot(timeVec,tauMotor_f(:,1),'-')
plot(timeVec,tauMotor_mdl(:,1) + tauF(:,1),'--')
plot(timeVec,tauMotor_mdl_RR(:,1) + tauF(:,1),'-.')
plot(timeVec,tauMotor_mdl_CAD(:,1) + tauF(:,1),':')
hold off
legend('meas','FJR','RR','CAD')
xlabel('time')
ylabel('tauMotor')
title('Axis 1')

figure
hold on
plot(timeVec,tauMotor_f(:,2),'-')
plot(timeVec,tauMotor_mdl(:,2) + tauF(:,2),'--')
plot(timeVec,tauMotor_mdl_RR(:,2) + tauF(:,2),'-.')
plot(timeVec,tauMotor_mdl_CAD(:,2) + tauF(:,2),':')
hold off
legend('meas','FJR','RR','CAD')
xlabel('time')
ylabel('tauMotor')
title('Axis 2')

figure
hold on
plot(timeVec,tauMotor_f(:,3),'-')
plot(timeVec,tauMotor_mdl(:,3) + tauF(:,3),'--')
plot(timeVec,tauMotor_mdl_RR(:,3) + tauF(:,3),'-.')
plot(timeVec,tauMotor_mdl_CAD(:,3) + tauF(:,3),':')
hold off
legend('meas','FJR','RR','CAD')
xlabel('time')
ylabel('tauMotor')
title('Axis 3')

%%

% Calculate acceleration from base parameters with cosinderation of tauE
if SAVE_PLOT_DATA

    domega_mdl_tauE = zeros(nSamples,6);

    for iSample = 1:nSamples
    
        % Load input data of current sample
        theta_i = [thetaLoad_f(iSample,:)'; thetaMotor_f(iSample,:)']; % Position
        omega_i = [omegaLoad_f(iSample,:)'; omegaMotor_f(iSample,:)']; % Velocity
        domega_i = [domegaLoad_f(iSample,:)'; domegaMotor_f(iSample,:)']; % Acceleration
        tauMotor_i = tauMotor_f(iSample,:)'; % Motor torque
        TMotor_i = TMotor(iSample,:)'; % Motor temperature
        tauF_i = tauF(iSample,:)'; % Friction torque
        tauE_i = tauE(iSample,:)'; % Joint torque


        % Calculate acceleration of FJR model
        domega_mdl_tauE(iSample,:) = ...
            rad2deg(directDynamics_reduced_fun(g, i_g, r_P1P2, r_P2P3, r_P3P4, param_tauE, deg2rad(theta_i), deg2rad(omega_i), tauE_i, tauF_i, tauMotor_i))';

    end

end

%% Save model


mdlStruct = struct;

mdlStruct.param.baseParam = param;
mdlStruct.param.baseParam_RR = [param_RR(1); 0; param_RR(2:8); 0; param_RR(9:15); 0];
mdlStruct.param.baseParam_CAD = param_CAD;

if SAVE_IDENTIFICATION_DATA

    mdlStruct.data.conW = conW;
    mdlStruct.data.conW_RR = conW_RR;

    mdlStruct.data.time = timeVec;

    mdlStruct.data.thetaMotor = thetaMotor_f;
    mdlStruct.data.thetaLoad = thetaLoad_f;
    mdlStruct.data.omegaMotor = omegaMotor_f;
    mdlStruct.data.omegaLoad = omegaLoad_f;
    mdlStruct.data.domegaMotor = domegaMotor_f;
    mdlStruct.data.domegaLoad = domegaLoad_f;
    mdlStruct.data.tauMotor = tauMotor_f;

    mdlStruct.metrics.mae = mae;
    mdlStruct.metrics.mape = mape;
    mdlStruct.metrics.msae = mse;
    mdlStruct.metrics.msape = mspe;
    mdlStruct.metrics.maxAE = maxAE;

    mdlStruct.metrics.mae_RR = mae_RR;
    mdlStruct.metrics.mape_RR = mape_RR;
    mdlStruct.metrics.msae_RR = mse_RR;
    mdlStruct.metrics.msape_RR = mspe_RR;
    mdlStruct.metrics.maxAE_RR = maxAE_RR;

    mdlStruct.metrics.mae_CAD = mae_CAD;
    mdlStruct.metrics.mape_CAD = mape_CAD;
    mdlStruct.metrics.msae_CAD = mse_CAD;
    mdlStruct.metrics.msape_CAD = mspe_CAD;
    mdlStruct.metrics.maxAE_CAD = maxAE_CAD;

end

if SAVE_PLOT_DATA

    mdlStruct.data.tauMotor_mdl = tauMotor_mdl;
    mdlStruct.data.tauMotor_mdl_RR = tauMotor_mdl_RR;
    mdlStruct.data.tauMotor_mdl_CAD = tauMotor_mdl_CAD;

    mdlStruct.data.tauF_mdl = tauF;
    mdlStruct.data.tauE_mdl = tauE;

    mdlStruct.data.conW_tauE = conW_tauE;
    mdlStruct.data.tauMotor_mdl_tauE = tauMotor_mdl_tauE;
    mdlStruct.data.domega_mdl_tauE = domega_mdl_tauE;

end

save([OUTPUT_FOLDER '/model_robot.mat'],'-struct','mdlStruct');