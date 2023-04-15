%% Identification of joint stiffness
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Identification settings 
AXIS = 1;                                                   % Robot axis

INPUT_FOLDERS = { ...                                       % Measurment data folders
    'measurments/12_07_stiffness_axis1' ...
    'measurments/12_07_stiffness_axis2' ...
    'measurments/12_07_stiffness_axis3' ...
    };
INPUT_FOLDER_MODEL = 'identification/models/kinError';      % Folder with kinematic error model
OUTPUT_FOLDER = 'identification/models/stiffness';          % Output folder for model 
SAVE_IDENTIFICATION_DATA = 1;                               % Add identification data to output

SAMPLE_FREQ = 500;                                          % Sampling frequency of the measurment
TIME_AVERAGE = 0.05;                                        % Length of averaging window for filtering data

DELTA_THETA = [0.225 0.225 0.36];                           % Difference in motor positon between 2 configurations
TIME_DYNAMIC_START = 0.1;                                   % Time considered for dynamic effects at start of constant motor position


% Datasheet parameters
TAU_DATASHEET = [ ...
    14 48 100; ...
    7 25 100; ...
    2 6.9 100; ...
    ];
K_DATASHEET = [ ...
    31000 50000 57000; ...
    16000 25000 29000; ...
    4700 6100 7100 ...
    ];


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load data


% Get input folder and position difference between steps
dirIn = INPUT_FOLDERS{AXIS};
deltaTheta = DELTA_THETA(AXIS);


% Load kinematic parameters
kinStruct = loadRobotKinematic(0);
i_g = kinStruct.i_g(AXIS);


% Load and extract measurment data
expData = load([dirIn, '/expData.mat']);

timeVec = expData.t; % Time
thetaLoad = expData.thetaLoad(:,AXIS); % Load position
thetaMotor = expData.thetaMot(:,AXIS); % Motor position
tauMotor = expData.torque(:,AXIS) .* i_g; % Motor torque (load-side)
thetaMotor_traj = expData.refValuePos(:,AXIS); % Reference trajectory

nSamples = length(timeVec); % Number of samples

% Filter position and torque with averaging window
thetaLoad_f = movmean(thetaLoad,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_f = movmean(thetaMotor,TIME_AVERAGE*SAMPLE_FREQ);
tauMotor_f = movmean(tauMotor,TIME_AVERAGE*SAMPLE_FREQ);

% Calculate position deflection
thetaDiff_f = thetaMotor_f - thetaLoad_f;

%% Process data


% Find midpoint of measurment
idxMid = round(0.5 * nSamples);


% Find start and end position of the reference trajectory
theta_start = thetaMotor_traj(1);
theta_end = thetaMotor_traj(idxMid);
theta_end = theta_end - mod(theta_end-theta_start,deltaTheta);

theta_steps = (theta_start:deltaTheta:theta_end)'; % Vector of all tested motor positions
nTheta = length(theta_steps); % number of tested motor positions


% Find start and end indices of the different steady state motor positions in the
% reference trajectory
idxCut = cell(1,2);
idxCut{1} = zeros(nTheta,2);
idxCut{2} = zeros(nTheta,2);

for iTheta = 1:nTheta

    idxCut{1}(iTheta,1) = find(abs(thetaMotor_traj - theta_steps(iTheta)) <= 1e-3,1);
    idxCut{1}(iTheta,2) = (idxCut{1}(iTheta,1)-1) + find(abs(thetaMotor_traj(idxCut{1}(iTheta,1):end) - theta_steps(iTheta)) > 1e-3,1);

    idxCut{2}(iTheta,2) = find(abs(thetaMotor_traj - theta_steps(iTheta)) <= 1e-3,1,'last');
    idxCut{2}(iTheta,1) = find(abs(thetaMotor_traj(1:idxCut{2}(iTheta,2)) - theta_steps(iTheta)) > 1e-3,1,'last');

end

% Add start time to account for dynamic effects
idxCut{1}(:,1) = idxCut{1}(:,1) + round(TIME_DYNAMIC_START*SAMPLE_FREQ);
idxCut{2}(:,1) = idxCut{2}(:,1) + round(TIME_DYNAMIC_START*SAMPLE_FREQ);

% Delet first and last entry
idxCut{1}(1,:) = [];
idxCut{1}(end,:) = [];
idxCut{2}(1,:) = [];
idxCut{2}(end,:) = [];


% Plot the first 5 cutting indices to check if everthing worked as intended
figure
hold on
plot(thetaMotor)
xline(idxCut{1}(1:5,1))
xline(idxCut{1}(1:5,2))
hold off
xlim([0 idxCut{1}(5,2)+1])
xlabel('idx')
ylabel('thetaMotor')


% Extract all samples with the same motor position according to
% the cutting indices calculated earlier
nTheta_new = size(idxCut{1},1);
thetaMotorCut = cell(nTheta_new,2);
thetaMotorCut{1} = [];
thetaMotorCut{2} = [];
thetaLoadCut = cell(nTheta_new,2);
thetaLoadCut{1} = [];
thetaLoadCut{2} = [];
thetaDiffCut = cell(nTheta_new,2);
thetaDiffCut{1} = [];
thetaDiffCut{2} = [];
tauMotorCut = cell(nTheta_new,2);
tauMotorCut{1} = [];
tauMotorCut{2} = [];

thetaMotorMean = zeros(nTheta_new,2);
thetaLoadMean = zeros(nTheta_new,2);
thetaDiffMean = zeros(nTheta_new,2);
tauMotorMean = zeros(nTheta_new,2);

for iTimeCut = 1:nTheta_new
    for iDirection = 1:2

        % Cut out signal parts
        thetaMotorCut{iTimeCut,iDirection} = thetaMotor(idxCut{iDirection}(iTimeCut,1):idxCut{iDirection}(iTimeCut,2));
        thetaLoadCut{iTimeCut,iDirection} = thetaLoad(idxCut{iDirection}(iTimeCut,1):idxCut{iDirection}(iTimeCut,2));
        thetaDiffCut{iTimeCut,iDirection} = thetaDiff_f(idxCut{iDirection}(iTimeCut,1):idxCut{iDirection}(iTimeCut,2));
        tauMotorCut{iTimeCut,iDirection} = tauMotor_f(idxCut{iDirection}(iTimeCut,1):idxCut{iDirection}(iTimeCut,2));
    
        % Calculate mean values of cut signal parts
        thetaMotorMean(iTimeCut,iDirection) = mean(thetaMotorCut{iTimeCut,iDirection});
        thetaLoadMean(iTimeCut,iDirection) = mean(thetaLoadCut{iTimeCut,iDirection});
        thetaDiffMean(iTimeCut,iDirection) = mean(thetaDiffCut{iTimeCut,iDirection});
        tauMotorMean(iTimeCut,iDirection) = mean(tauMotorCut{iTimeCut,iDirection});

    end
end


% Interpolate positive and negative movement data so that they overlap
% completely
xThetaMotor = theta_steps(5:end-4);
yThetaLoad = [ ...
    interp1(thetaMotorMean(:,1),thetaLoadMean(:,1),xThetaMotor,'linear','extrap') ...
    interp1(thetaMotorMean(:,2),thetaLoadMean(:,2),xThetaMotor,'linear','extrap') ...
    ];
yThetaDiff = [ ...
    interp1(thetaMotorMean(:,1),thetaDiffMean(:,1),xThetaMotor,'linear','extrap') ...
    interp1(thetaMotorMean(:,2),thetaDiffMean(:,2),xThetaMotor,'linear','extrap') ...
    ];
yTauMotor = [ ...
    interp1(thetaMotorMean(:,1),tauMotorMean(:,1),xThetaMotor,'linear','extrap') ...
    interp1(thetaMotorMean(:,2),tauMotorMean(:,2),xThetaMotor,'linear','extrap') ...
    ];


% Calculate torque backlash
backlashTau = 0.5 .* (yTauMotor(:,1) - yTauMotor(:,2));

%% Compensate Kinematic Error


% Load kinematic error model
mdl_kinError = load([INPUT_FOLDER_MODEL '/model_kinError_axis' num2str(AXIS) '.mat']);


% Calculate kinematic error compensation
if isfield(mdl_kinError.param,'c_pError') % Model has periodic kinematic error

    % Non-periodic error
    npErrorComp = interp1(mdl_kinError.param.xLookup,mdl_kinError.param.yLookup,xThetaMotor,'linear','extrap');

    % Periodic error
    if AXIS == 1
        pErrorComp = kinError_periodic_axis1_fun(mdl_kinError.param.c_pError',xThetaMotor);
    else
        pErrorComp = 0;
    end

    % Total kinematic error
    kinErrorComp = npErrorComp + pErrorComp;
else
    
    kinErrorComp = interp1(mdl_kinError.param.xLookup,mdl_kinError.param.yLookup,xThetaMotor,'linear','extrap');

end


% Compensate kinematic error
thetaDiff_woKinError = yThetaDiff - kinErrorComp;

%% Identify Joint Stiffness


% Average motor torque and position deflection over positive and negative direction
% movement (otherwise there is a torque offset between th different
% direction movements that is probably caused by the used controller
% I-part)
tauMotorVec = 0.5 .* (yTauMotor(:,1) + yTauMotor(:,2));
thetaDiffVec_woOffset = 0.5 .* (thetaDiff_woKinError(:,1) + thetaDiff_woKinError(:,2));


% Calculate Offset of zero deflection and subract it
thetaOffset = mean(thetaDiffVec_woOffset(abs(tauMotorVec) <= 0.1.*max(abs(tauMotorVec))))
thetaDiffVec = thetaDiffVec_woOffset - thetaOffset;


% Calculate backlash in positive and negative direction movement
backlashTheta = mean(thetaDiff_woKinError(:,1)-thetaDiff_woKinError(:,2))
backlashTau = mean(yTauMotor(:,1)-yTauMotor(:,2))


% Calculate hysteresis curves
tauMotor_hysteresis = yTauMotor + 0.5.*backlashTau.*[-1, 1];
thetaDiff_hysteresis = thetaDiff_woKinError + 0.5.*backlashTheta.*[-1, 1] - thetaOffset;


% Create symbolic model of joint stiffness torque
syms tauMotor_ 'real';
syms thetaDiff_ 'real';
syms c_k_ [2,1] 'real';

% Symbolic model
tauE_ = sign(thetaDiff_) .* c_k_(1) .* abs(thetaDiff_).^c_k_(2);

% Create function of joint stiffness torque
tauE_fun = matlabFunction(tauE_, 'Vars',{c_k_, thetaDiff_});


% Find parameters that minimize MSE between model and measurment data
error = tauMotor_ - tauE_; % Error
derror = jacobian(error,c_k_); % Jacobian of error

objFun = matlabFunction(error, derror, 'Vars',{c_k_, thetaDiff_, tauMotor_});
objective = @(param) objFun(param,thetaDiffVec,tauMotorVec); % Objective function for optimization

options = optimoptions('lsqnonlin','Display','iter','SpecifyObjectiveGradient',true);
param = lsqnonlin(objective,ones(length(c_k_),1),[],[],options); % Use nonlinear least square for optimization


% Calculate model prediction
tauE_mdl = tauE_fun(param,thetaDiffVec);


% Compare measurment data with model output
[mae, mape] = calcMAE(tauMotorVec,tauE_mdl)
[mse, mspe] = calcMSE(tauMotorVec,tauE_mdl)
maxAE = max(abs(tauMotorVec - tauE_mdl))


% Calculate torque according to deflection offset
tauOffset = tauE_fun(param,thetaOffset);


% Plot measurement and model data
thetaDiffVec_mdl = (min(thetaDiffVec):0.001:max(thetaDiffVec))';

% Joint stiffness curve
figure
hold on
plot(tauMotorVec,thetaDiffVec,'.')
plot(tauE_fun(param,thetaDiffVec_mdl),thetaDiffVec_mdl,'-')
plot(tauMotor_hysteresis(:,1),thetaDiff_hysteresis(:,1),'k.')
plot(tauMotor_hysteresis(:,2),thetaDiff_hysteresis(:,2),'g.')
hold off
xlabel('tauStiffness')
ylabel('thetaDiff')
legend('measurment','model','Location','southeast')

% Joint stiffness curve
figure
hold on
plot(thetaDiffVec,tauMotorVec,'.')
plot(thetaDiffVec_mdl,tauE_fun(param,thetaDiffVec_mdl),'-')
plot(thetaDiff_hysteresis(:,1),tauMotor_hysteresis(:,1),'k.')
plot(thetaDiff_hysteresis(:,2),tauMotor_hysteresis(:,2),'g.')
hold off
xlabel('thetaDiff')
ylabel('tauStiffness')
legend('measurment','model','Location','southeast')

%% Compare to datasheet parameters


% Get datasheet values
tau_d = TAU_DATASHEET(AXIS,:);
k_d = K_DATASHEET(AXIS,:);


% Convert datasheet values to degree
degK_d = k_d .* (pi/180);


% Calculate deflection accroding to datasheet values
thetaDiff_d = zeros(1,3);
thetaDiff_d(1) = tau_d(1) / degK_d(1);
thetaDiff_d(2) = thetaDiff_d(1) + ((tau_d(2)-tau_d(1)) / degK_d(2));
thetaDiff_d(3) = thetaDiff_d(2) + ((tau_d(3)-tau_d(2)) / degK_d(3));


% Maximum deflection in measurment data
maxThetaDiff = max(abs(thetaDiffVec));


% Create vector for new deflection points for stiffnes curve
idxConsider = thetaDiff_d > maxThetaDiff;
k_thetaDiff_lim = [maxThetaDiff, thetaDiff_d(idxConsider), Inf.*ones(1,nnz(~idxConsider))];

%% Save data


mdlStruct = struct;

mdlStruct.param.c_k = param';
mdlStruct.param.k_thetaDiff_lim = k_thetaDiff_lim;

if SAVE_IDENTIFICATION_DATA

    mdlStruct.data.tauMotor = tauMotorVec;
    mdlStruct.data.thetaDiff = thetaDiffVec;

    mdlStruct.data.tauMotor_raw = yTauMotor;
    mdlStruct.data.thetaDiff_raw = thetaDiff_woKinError;

    mdlStruct.data.tauMotor_hysteresis = tauMotor_hysteresis;
    mdlStruct.data.thetaDiff_hysteresis = thetaDiff_hysteresis;

    mdlStruct.metrics.mae = mae;
    mdlStruct.metrics.mape = mape;
    mdlStruct.metrics.msae = mse;
    mdlStruct.metrics.msape = mspe;
    mdlStruct.metrics.maxAE = maxAE;

end

save([OUTPUT_FOLDER, '/model_stiffness_axis', num2str(AXIS), '.mat'],'-struct','mdlStruct');