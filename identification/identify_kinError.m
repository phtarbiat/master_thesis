%% Identification of kinematic error
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
ADD_PERIODIC_ERROR = 0;                                     % Add periodic kinematic error identification
                                      
INPUT_FOLDERS = { ...                                       % Measurment data folders
    'measurments/12_07_kinError_axis1' ...                     
    'measurments/12_07_kinError_axis2' ...
    'measurments/12_07_kinError_axis3' ...
    };
OUTPUT_FOLDER = 'identification/models/kinError';           % Output folder for model 
SAVE_IDENTIFICATION_DATA = 1;                               % Add identification data to output
SAVE_PLOT_DATA = 1;                                         % Add plotting data to output
GENERATE_FFS_FUN = 1;                                       % Generate function script for periodic kinematic error

SAMPLE_FREQ = 500;                                          % Sampling frequency of the measurment data
TIME_AVERAGE = 0.05;                                        % Length of averaging window for filtering data

OMEGA = [0.225 0.225 0.36];                                 % Constant velocity of measurment
THETA_DELTA_LOOKUP = [0.5 0.25];                            % Lookup table incrementation
COEFFS_FOURIER_SERIES = { ...                               % Considered frequencies for periodic kinematic error
    [2 3 4] ...                                             % (in multiple of motor rotation frequency)
    0 ...
    0 ...
    };


% Create output folder
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end

%% Load data


% Get input folder, constant velocity of the measurment and flag for adding
% periodic kinematic error
dirIn = INPUT_FOLDERS{AXIS};
omega = OMEGA(AXIS);
if ADD_PERIODIC_ERROR
    thetaStep_lookup = THETA_DELTA_LOOKUP(1);
else
    thetaStep_lookup = THETA_DELTA_LOOKUP(2);
end


% Load robot work space constraints
conStruct = loadRobotConstraints();
theta_max = conStruct.theta_max(AXIS);


% Load and extract measurment data
expData = load([dirIn, '/expData.mat']);

timeVec = expData.t; % Time
thetaLoad = expData.thetaLoad(:,AXIS); % Load position
thetaMotor = expData.thetaMot(:,AXIS); % Motor position
thetaMotor_traj = expData.refValuePos(:,AXIS); % Reference trajectory

nSamples = length(timeVec); % Number of samples

% Filter motor and load position with averaging window
thetaLoad_f = movmean(thetaLoad,TIME_AVERAGE*SAMPLE_FREQ);
thetaMotor_f = movmean(thetaMotor,TIME_AVERAGE*SAMPLE_FREQ);

% Calculate position deflection
thetaDiff_f = thetaMotor_f - thetaLoad_f;

%% Process data


% Find midpoint of measurment
idxMid = round(0.5 * nSamples);


% Determine start and end points of considered data for positive and 
% negative velocity movement
idxStart = [ ...
    find(thetaMotor_traj ~= thetaMotor_traj(1),1); ...
    find(thetaMotor_traj == thetaMotor_traj(idxMid),1,'last') + 1 ...
    ];
idxEnd =[ ...
    find(thetaMotor_traj == thetaMotor_traj(idxMid),1) - 1; ...
    find(thetaMotor_traj ~= thetaMotor_traj(1),1,'last') ...
    ];


% Extract motor position and deflection according to start and end points
thetaMotorCut = { ...
    thetaMotor_f(idxStart(1):idxEnd(1)) ...
    flip(thetaMotor_f(idxStart(2):idxEnd(2))) ...
    };
thetaDiffCut = { ...
    thetaDiff_f(idxStart(1):idxEnd(1)) ...
    flip(thetaDiff_f(idxStart(2):idxEnd(2))) ...
    };


% Average over non-unique entries
[thetaMotorCut{1},~,idx] = unique(thetaMotorCut{1});
thetaDiffCut{1} = accumarray(idx(:),thetaDiffCut{1},[],@mean);
[thetaMotorCut{2},~,idx] = unique(thetaMotorCut{2});
thetaDiffCut{2} = accumarray(idx(:),thetaDiffCut{2},[],@mean);


% Interpolate positive and negative movement data so that they overlap
% completely
xThetaMotor = (-theta_max+0.1:omega/SAMPLE_FREQ:theta_max-0.1)';
yThetaDiff = { ...
    interp1(thetaMotorCut{1},thetaDiffCut{1},xThetaMotor,'linear','extrap') ...
    interp1(thetaMotorCut{2},thetaDiffCut{2},xThetaMotor,'linear','extrap') ...
    };


% Calculate backlash
backlash = 0.5 .* (yThetaDiff{1} - yThetaDiff{2});


% Calculate kinematic error
kinError = 0.5 .* (yThetaDiff{1} + yThetaDiff{2});

%% Identify backlash


backlashComp = mean(backlash);

figure
hold on
plot(xThetaMotor,backlash)
yline(backlashComp)
hold off

%% Identify kinematic error


if ADD_PERIODIC_ERROR % Add periodic kinematic error
    
    % Get fourier series parameters
    coeffsFourier = COEFFS_FOURIER_SERIES{AXIS};
    nFourier = length(coeffsFourier);


    % Load robot kinematic parameters and work space constraints
    kinStruct = loadRobotKinematic(0);
    i_g = kinStruct.i_g(AXIS);


    % Extract periodic from total kinematic error by extracting harmonics of
    % mutplies of the motor rotation from the kinematic error
    L = length(kinError);
    fVec = 10*SAMPLE_FREQ*(0:(L/2))/L; % frequency vectors of fft scaled by factor 10
    kinError_fftData = fft(kinError); % Fast fourier transform of kinematic error

    % Cut harmonics out of the kinematic errors fft
    pError_fBounds = [coeffsFourier' coeffsFourier'] + [-0.1 0.1]; % Frequencies to cut out
    pError_fftData = zeros(length(kinError_fftData),1);
    for iF = 1:size(pError_fBounds,1)
    
            idxStart = find(fVec >= pError_fBounds(iF,1),1);
            idxEnd = find(fVec > pError_fBounds(iF,2),1) - 1;
    
            pError_fftData(idxStart:idxEnd) = kinError_fftData(idxStart:idxEnd);
    
    end

    % Reconstruct a signal out of the cut out frequencies
    pError = ifft(pError_fftData,'symmetric');


    % Create symbolic finite fourier series (ffs) to approximate the
    % periodic kinematic error
    syms y 'real';
    syms x 'real';
    syms cCos [nFourier,1] 'real';
    syms cSin [nFourier,1] 'real';
    
    paramVec = [cCos; cSin]; % parameters of the ffs
    omega_0 = deg2rad(i_g); % base frequency of the periodic error
    
    fourierSeries = 0;
    for iFourier = 1:nFourier
        fourierSeries = fourierSeries ...
            + cCos(iFourier) .* cos(omega_0.*coeffsFourier(iFourier).*x) ...
            + cSin(iFourier) .* sin(omega_0.*coeffsFourier(iFourier).*x);
    end
    
    % Create function of ffs and save it to a file
    if GENERATE_FFS_FUN
        FUN_NAME = ['kinError_periodic_axis' num2str(AXIS) '_fun'];
        fun_hdError = matlabFunction(fourierSeries, ...
            'File', ['model/functions/robot/include/' FUN_NAME], ...
            'Vars', {paramVec, x}, ...
            'Outputs', {'hdError'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', FUN_NAME], ...
            ' Inputs: coeffs, theta', '', ''
            });
    else
        fun_hdError = matlabFunction(fourierSeries,'Vars',{paramVec, x});
    end


    % Find parameters that minimize MSE between model and measurment data
    error = y - fourierSeries; % Error
    derror = jacobian(error,paramVec); % Jacobian of error
    
    objFun = matlabFunction(error, derror, 'Vars',{paramVec, x, y});
    objective = @(param) objFun(param,xThetaMotor,pError); % Objective function for optimization
    
    options = optimoptions('lsqnonlin','Display','iter','SpecifyObjectiveGradient',true);
    param = lsqnonlin(objective,zeros(length(paramVec),1),[],[],options); % Use nonlinear least square for optimization


    % Calculate model prediction
    pError_mdl = fun_hdError(param,xThetaMotor);

    
    % Compare measurment data with model output
    [mae_pError, mape_pError] = calcMAE(pError,pError_mdl)
    [mse_pError, mspe_pError] = calcMSE(pError,pError_mdl)


    % Plot measurement and model data
    figure
    hold on
    plot(xThetaMotor,pError,'--')
    plot(xThetaMotor,pError_mdl)
    hold off
    title('periodic Error')
    legend('measurment','model')
    xlabel('thetaMotor')
    ylabel('hdError')


    % Calculate non-period kinematic error
    npError = kinError - pError;

else
    % Calculate non-period kinematic error
    npError = kinError;
end


% Create look-up table for non-period kinematic error
xLookup_npError = (-theta_max:thetaStep_lookup:theta_max)';
yLookup_npError = interp1(xThetaMotor,npError,xLookup_npError,'linear','extrap');

npError_mdl = interp1(xLookup_npError,yLookup_npError,xThetaMotor,'linear');

% Generate output, MAE and MSE of complete model
if ADD_PERIODIC_ERROR

    % Compare measurment data with model output
    [mae_npError, mape_npError] = calcMAE(npError,npError_mdl)
    [mse_npError, mspe_npError] = calcMSE(npError,npError_mdl)
    
    % Plot measurement and model data
    figure
    hold on
    plot(xThetaMotor,npError,'-')
    plot(xThetaMotor,npError_mdl,'--')
    hold off
    xlim([-theta_max+1 theta_max-1])
    title('non-periodic Error')
    legend('measurment','model')

    kinError_mdl = npError_mdl + pError_mdl;

else
    kinError_mdl = npError_mdl;
end

% Calculate MAE and MSE of model
[mae, mape] = calcMAE(kinError,kinError_mdl)
[mse, mspe] = calcMSE(kinError,kinError_mdl)
maxAE = max(abs(kinError - kinError_mdl))


% Plot measurement and model data
figure
hold on
plot(xThetaMotor,kinError,'-')
plot(xThetaMotor,kinError_mdl,'--')
hold off
xlim([-theta_max+1 theta_max-1])
title('kinematic Error')
legend('measurment','model')

%% Save data


mdlStruct = struct;

mdlStruct.param.xLookup = xLookup_npError';
mdlStruct.param.yLookup = yLookup_npError';

if ADD_PERIODIC_ERROR
    mdlStruct.param.nFourier = COEFFS_FOURIER_SERIES;
    mdlStruct.param.omega_0 = omega_0;
    mdlStruct.param.c_pError = param';
end

if SAVE_IDENTIFICATION_DATA

    mdlStruct.data.thetaMotor = xThetaMotor;
    mdlStruct.data.thetaDiff = yThetaDiff;

    
    mdlStruct.metrics.mae = mae;
    mdlStruct.metrics.mape = mape;
    mdlStruct.metrics.msae = mse;
    mdlStruct.metrics.msape = mspe;
    mdlStruct.metrics.maxAE = maxAE;
    

end

if SAVE_PLOT_DATA

    mdlStruct.data.backlash = backlash;
    mdlStruct.data.kinError = kinError;

end

save([OUTPUT_FOLDER, '/model_kinError_axis', num2str(AXIS), '.mat'],'-struct','mdlStruct');