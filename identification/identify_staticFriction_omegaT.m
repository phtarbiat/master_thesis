%% Identification of velocity and temperature dependent joint friction
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Identification settings 
AXIS = 1;                                                                       % Robot axis
ADD_STRIBECK_EFFECT = 0;                                                        % Add stribeck effect
SPLIT_OMEGA_DIRECTION = 1;                                                      % Have different parameters for positive and negative velocity

INPUT_FOLDERS = { ...                                                           % Measurment data folders
    'measurments/12_12_staticFriction_omegaT_axis1' ...
    'measurments/11_18_staticFriction_omegaT_axis2' ...
    'measurments/12_02_staticFriction_omegaT_axis3' ...
    };
INPUT_FOLDER_TRAJ = 'trajectory/traj/staticFriction_omegaT';   % Experiment trajectory folder
OUTPUT_FOLDER = 'identification/models/staticFriction';                         % Output folder for model 
SAVE_IDENTIFICATION_DATA = 1;                                                   % Add identification data to output
SAVE_PLOT_DATA = 1;                                                             % Add plotting data to output

SAMPLE_FREQ = 500;                                                              % Sampling frequency of the measurment
SAMPLE_FREQ_TRAJ = 100;                                                         % Sampling frequency of the trajectory
TIME_AVERAGE = 0.05;                                                            % Length of averaging window for filtering data

NUM_OMEGA = 20;                                                                 % Number of velocities in measurment
NUM_MEASURMENTS = 60;                                                           % Number of measurments

TIME_DYNAMIC_START = 1;                                                         % Time considered for dynamic effect at start of constant velocity
TIME_DYNAMIC_END = 0.025;                                                       % Time considered for dynamic effect at end of constant velocity


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load and Process data


% Get input folder
dirIn = INPUT_FOLDERS{AXIS};


% Load kinematic parameters
kinStruct = loadRobotKinematic(0);
i_g = kinStruct.i_g(AXIS);


% Load the desired trajectory of the measurment
traj = load([INPUT_FOLDER_TRAJ '/traj_staticFriction_omegaT_axis' num2str(AXIS) '.mat']);
omegaMotor_traj = traj.omega(:,AXIS);


% Find start and end indices of the different velocities in the desired
% trajectory of the experiment
idxCut = cell(1,2);
idxCut{1} = zeros(NUM_OMEGA,2); % Positive velcoity
idxCut{2} = zeros(NUM_OMEGA,2); % Negative velocity

for iOmega = 1:NUM_OMEGA

    if iOmega == 1
        idxCut{1}(iOmega,1) = find(omegaMotor_traj > 0,1);
    else
        idxCut{1}(iOmega,1) = (idxCut{2}(iOmega-1,2)-1) + find(omegaMotor_traj(idxCut{2}(iOmega-1,2):end) > 0,1);
    end

    idxCut{1}(iOmega,2) = (idxCut{1}(iOmega,1)-1) + find(omegaMotor_traj(idxCut{1}(iOmega,1):end) == 0,1);
    
    idxCut{2}(iOmega,1) = (idxCut{1}(iOmega,2)-1) + find(omegaMotor_traj(idxCut{1}(iOmega,2):end) < 0,1);
    idxCut{2}(iOmega,2) = (idxCut{2}(iOmega,1)-1) + find(omegaMotor_traj(idxCut{2}(iOmega,1):end) == 0,1);

end

% Scale indices according to the signals sampling frequency
idxCut{1} = idxCut{1} * (SAMPLE_FREQ/SAMPLE_FREQ_TRAJ);
idxCut{2} = idxCut{2} * (SAMPLE_FREQ/SAMPLE_FREQ_TRAJ);

% Scale waiting time for dynamic effects at start according to velocity
scalingFun = (1 / NUM_OMEGA^2) * (1:NUM_OMEGA)'.^2;
time_dynamic = 0.1 + (TIME_DYNAMIC_START .* scalingFun);

% Add start time to account for dynamics in velocity
idxCut{1}(:,1) = idxCut{1}(:,1) + round(time_dynamic*SAMPLE_FREQ);
idxCut{2}(:,1) = idxCut{2}(:,1) + round(time_dynamic*SAMPLE_FREQ);

% Subtract end time
idxCut{1}(:,2) = idxCut{1}(:,2) - round(TIME_DYNAMIC_END*SAMPLE_FREQ);
idxCut{2}(:,2) = idxCut{2}(:,2) - round(TIME_DYNAMIC_END*SAMPLE_FREQ);


% Initialize Variables that hold all measurments data
T = []; % Temperature

% Signal parts with the same constant velocity
omegaMotorCut = cell(NUM_MEASURMENTS,1); % Motor velocity
tauMotorCut = cell(NUM_MEASURMENTS,1); % Motor torque
TMotorCut = cell(NUM_MEASURMENTS,1); % Temperature

% Average signal values for every constant velocity
omegaMotorMean = cell(2,1); % Motor velocity
omegaMotorMean{1} = zeros(NUM_OMEGA,NUM_MEASURMENTS); % Positive velocity
omegaMotorMean{2} = zeros(NUM_OMEGA,NUM_MEASURMENTS); % Negative velocity
tauMotorMean = cell(2,1); % Motor torque
tauMotorMean{1} = zeros(NUM_OMEGA,NUM_MEASURMENTS); % Positive velocity
tauMotorMean{2} = zeros(NUM_OMEGA,NUM_MEASURMENTS); % Negative velocity
TMotorMean = cell(2,1); % Temperature
TMotorMean{1} = zeros(NUM_OMEGA,NUM_MEASURMENTS); % Positive velocity
TMotorMean{2} = zeros(NUM_OMEGA,NUM_MEASURMENTS); % Negative velocity


% Collect data for all measurments
for iMeas = 1:NUM_MEASURMENTS

    % Load and extract measurment data
    expData = load([dirIn, '/expData_',num2str(iMeas),'.mat']);
    
    timeVec = expData.t; % Time
    thetaMotor = expData.thetaMot(:,AXIS); % Motor position
    tauMotor = expData.torque(:,AXIS);% .* i_g; % Motor torque
    TMotor = expData.temperature(:,AXIS); % Motor temperature
    
    nSamples = length(timeVec); % Number of samples

    % Filter position and torque with averaging window
    thetaMotor_f = movmean(thetaMotor,TIME_AVERAGE*SAMPLE_FREQ);
    tauMotor_f = movmean(tauMotor,TIME_AVERAGE*SAMPLE_FREQ);

    % Calculate motor velocity by central differentiation
    deltaTime = 1 / SAMPLE_FREQ;
    omegaMotor_f = zeros(nSamples,1);
    omegaMotor_f(2:end-1) = (thetaMotor_f(3:end) - thetaMotor_f(1:end-2)) ./ (2 * deltaTime);
    

    % Add temperature of current measurment to the data of all measurments
    T = [T; TMotor];


    % Extract all samples with the same constant velocity according to
    % the cutting indices calculated earlier
    omegaMotorCut{iMeas} = cell(NUM_OMEGA,2);
    tauMotorCut{iMeas} = cell(NUM_OMEGA,2);
    TMotorCut{iMeas} = cell(NUM_OMEGA,2);
    
    for iOmega = 1:NUM_OMEGA
        for iDirection = 1:2
        
            omegaMotorCut{iMeas}{iOmega,iDirection} = omegaMotor_f(idxCut{iDirection}(iOmega,1):idxCut{iDirection}(iOmega,2));
            tauMotorCut{iMeas}{iOmega,iDirection} = tauMotor_f(idxCut{iDirection}(iOmega,1):idxCut{iDirection}(iOmega,2));
            TMotorCut{iMeas}{iOmega,iDirection} = TMotor(idxCut{iDirection}(iOmega,1):idxCut{iDirection}(iOmega,2));

        end  
    end


    % Calculate average signal values for every constant velocity
    for iOmega = 1:NUM_OMEGA
        for iDirection = 1:2
            omegaMotorMean{iDirection}(iOmega,iMeas) = mean(omegaMotorCut{iMeas}{iOmega,iDirection});
            tauMotorMean{iDirection}(iOmega,iMeas) = mean(tauMotorCut{iMeas}{iOmega,iDirection});
            TMotorMean{iDirection}(iOmega,iMeas) = mean(TMotorCut{iMeas}{iOmega,iDirection});
        end
    end


    % Plot velocity for first measurment for reference of the cutting
    % indices
    if iMeas == 1
        figure
        hold on
        plot(timeVec,omegaMotor_f)
        xline(timeVec(idxCut{1}(:,1)),'k--')
        xline(timeVec(idxCut{1}(:,2)),'k--')
        xline(timeVec(idxCut{2}(:,1)),'k--')
        xline(timeVec(idxCut{2}(:,2)),'k--')
        hold off
        grid on
    end

    % Add data of first measurment to a struct to save it later
    if SAVE_PLOT_DATA
        if iMeas == 1
            meas1.time = timeVec;
            meas1.omegaMotor = omegaMotor_f;
            meas1.tauMotor = tauMotor_f;
            meas1.idxCut = idxCut;
        end
    end

end

%% Plot data


% Plot every 5th measurments firction curve in 2D
figure
hold on
for iMeas = 1:5:NUM_MEASURMENTS
    plot(omegaMotorMean{1},tauMotorMean{1},'--')
    plot(omegaMotorMean{2},tauMotorMean{2},'--')
end
hold off
xlabel('omegaMotor')
ylabel('tauF')
grid on


% Plot temperature over all measurments
figure
plot(T)
xlabel('time')
ylabel('T')
grid on


% Plot every firction curve in 3D over velocity and temperature
figure
surf( ...
    [flip(omegaMotorMean{2},1); zeros(1,size(omegaMotorMean{1},2)); omegaMotorMean{1}], ...
    [flip(TMotorMean{2},1); 0.5.*(TMotorMean{1}(1,:)+TMotorMean{2}(1,:)); TMotorMean{1}], ...
    [flip(tauMotorMean{2},1); zeros(1,size(omegaMotorMean{1},2)); tauMotorMean{1}], ...
    'FaceColor','interp')
xlabel('omegaMotor')
ylabel('T')
zlabel('tauF')
view(30,-30)

%% Identify velocity and temperature dependent Friction


% Create symbolic model of the friction torque depending on velocity and
% temperature
syms tauMotor_ 'real'
syms omegaMotor_ 'real';
syms T_ 'real';
syms c_c_ 'real';
syms c_omega_ [3,1] 'real';
syms c_T_ [2,1] 'real';

paramVec = [c_c_; c_omega_; c_T_]; % parameters of the friction model

% Coloumb friction
tauF_c_ = c_c_;

% Viscous friction
tauF_omega_ = (c_omega_(1) - c_T_(1).*T_) .* (1 - exp(-abs(omegaMotor_)./(c_omega_(2) + c_T_(2).*T_)) + c_omega_(3).*abs(omegaMotor_).^0.5);

% Add stribeck effect if needed
if ADD_STRIBECK_EFFECT
    syms c_s_ [3,1] 'real';
    paramVec = [paramVec; c_s_];

    % Stribeck effect
    tauF_s_ = (c_s_(1) - c_c_) .* exp(-(abs(omegaMotor_)./c_s_(2)).^c_s_(3));
else
    tauF_s_ = 0;
end

% Combine all friction effects
tauF_ = sign(omegaMotor_) .* (tauF_c_ + tauF_s_ + tauF_omega_);


% Create function of friction torque
tauF_fun = matlabFunction(tauF_, 'Vars',{paramVec, omegaMotor_, T_});


% Find parameters that minimize MSE between model and measurment data
error = tauMotor_ - tauF_; % Error
derror = jacobian(error,paramVec); % Jacobian of error

objFun = matlabFunction(error, derror, 'Vars',{paramVec, omegaMotor_, T_ tauMotor_});

options = optimoptions('lsqnonlin','SpecifyObjectiveGradient',true,'Display','iter');

param = cell(2,1); % Parameter cell array

% Identify positive velocity friction model
objective_pos = @(param) objFun(param,omegaMotorMean{1}(:),TMotorMean{1}(:),tauMotorMean{1}(:));
param{1} = lsqnonlin(objective_pos,ones(length(paramVec),1),[],[],options);

% Identify negative velocity friction model
objective_neg = @(param) objFun(param,omegaMotorMean{2}(:),TMotorMean{2}(:),tauMotorMean{2}(:));
param{2} = lsqnonlin(objective_neg,ones(length(paramVec),1),[],[],options);


% Calculate model prediction
tauF_mdl = cell(2,1);
tauF_mdl{1} = tauF_fun(param{1},omegaMotorMean{1}(:),TMotorMean{1}(:));
tauF_mdl{2} = tauF_fun(param{2},omegaMotorMean{2}(:),TMotorMean{2}(:));


% Compare measurment data with model output
[mae_pos, mape_pos] = calcMAE(tauMotorMean{1}(:),tauF_mdl{1})
[mse_pos, mspe_pos] = calcMSE(tauMotorMean{1}(:),tauF_mdl{1})

[mae_neg, mape_neg] = calcMAE(tauMotorMean{2}(:),tauF_mdl{2})
[mse_neg, mspe_neg] = calcMSE(tauMotorMean{2}(:),tauF_mdl{2})

[mae, mape] = calcMAE([tauMotorMean{1}(:); tauMotorMean{2}(:)],[tauF_mdl{1}; tauF_mdl{2}])
[mse, mspe] = calcMSE([tauMotorMean{1}(:); tauMotorMean{2}(:)],[tauF_mdl{1}; tauF_mdl{2}])
maxAE = max(abs([tauMotorMean{1}(:); tauMotorMean{2}(:)] - [tauF_mdl{1}; tauF_mdl{2}]))

absError = cell(2,1);
absError{1} = abs(tauMotorMean{1} - tauF_fun(param{1},omegaMotorMean{1},TMotorMean{1}));
absError{2} = abs(tauMotorMean{2} - tauF_fun(param{2},omegaMotorMean{2},TMotorMean{2}));


tauF_mdl{1} = zeros(size(omegaMotorMean{1}));
tauF_mdl{2} = zeros(size(omegaMotorMean{1}));
for iMeas = 1:NUM_MEASURMENTS
    tauF_mdl{1}(:,iMeas) = tauF_fun(param{1},omegaMotorMean{1}(:,iMeas),TMotorMean{1}(:,iMeas));
    tauF_mdl{2}(:,iMeas) = tauF_fun(param{2},omegaMotorMean{2}(:,iMeas),TMotorMean{2}(:,iMeas));
end


% Plot measurement and model data
% Plot friction curve of model prediction and measurment data for every 5th
% measurment
figure
hold on
for iMeas = 1:2:NUM_MEASURMENTS
    plot(omegaMotorMean{1}(:,iMeas),tauF_fun(param{1},omegaMotorMean{1}(:,iMeas),TMotorMean{1}(:,iMeas)),'-')
    plot(omegaMotorMean{1}(:,iMeas),tauMotorMean{1}(:,iMeas),'x')
    
    plot(omegaMotorMean{2}(:,iMeas),tauF_fun(param{2},omegaMotorMean{2}(:,iMeas),TMotorMean{2}(:,iMeas)),'-')
    plot(omegaMotorMean{2}(:,iMeas),tauMotorMean{2}(:,iMeas),'x')
end
hold off
xlabel('omegaMotor')
ylabel('tauF')
grid on

% Plot absolute error over the velocity and temperature
figure
hold on
for iMeas = 1:NUM_MEASURMENTS
    plot3(omegaMotorMean{1}(:,iMeas), TMotorMean{1}(:,iMeas), absError{1}(:,iMeas),'.');
    plot3(omegaMotorMean{2}(:,iMeas), TMotorMean{2}(:,iMeas), absError{2}(:,iMeas),'.');
end
surf([-100 100],[20 40],mae.*ones(2,2))
xlabel('omegaMotor')
ylabel('T')
zlabel('error')
view(0,0)

%% Save model


mdlStruct = struct;

mdlStruct.param.c_c_pos = [param{1}(1) 0];
mdlStruct.param.c_c_neg = [param{2}(1) 0];

if ADD_STRIBECK_EFFECT
    mdlStruct.param.c_stri_pos = param{1}(7:9)';
    mdlStruct.param.c_stri_neg = param{2}(7:9)';
else
    mdlStruct.param.c_stri_pos = zeros(1,3);
    mdlStruct.param.c_stri_neg = zeros(1,3);
end

mdlStruct.param.c_omega_pos = param{1}(2:4)';
mdlStruct.param.c_omega_neg = param{2}(2:4)';

mdlStruct.param.c_T_pos = param{1}(5:6)';
mdlStruct.param.c_T_neg = param{2}(5:6)';

if SAVE_IDENTIFICATION_DATA

    mdlStruct.data.omegaMotor = omegaMotorMean;
    mdlStruct.data.TMotor = TMotorMean;
    mdlStruct.data.tauMotor = tauMotorMean;

    mdlStruct.metrics.mae = mae;
    mdlStruct.metrics.mape = mape;
    mdlStruct.metrics.mse = mse;
    mdlStruct.metrics.mspe = mspe;
    mdlStruct.metrics.maxAE = maxAE;

end

if SAVE_PLOT_DATA

    mdlStruct.data.timeVec = 1/SAMPLE_FREQ:1/SAMPLE_FREQ:length(T)/SAMPLE_FREQ;
    mdlStruct.data.T = T;

    mdlStruct.data.meas1 = meas1;

    mdlStruct.data.absError = absError;

    mdlStruct.data.tauF_mdl = tauF_mdl;

end

save([OUTPUT_FOLDER, '/model_staticFriction_omegaT_axis', num2str(AXIS), '.mat'],'-struct','mdlStruct');