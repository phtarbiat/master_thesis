%% Creation of trajectory for identifying the joint stiffness
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


AXIS = 1;                                           % Robot axis
SAMPLE_FREQ = 100;                                  % Sampling frequency
THETA_OTHER_AXIS = [ ...                            % Constant position of other axes
    0 78 0; ...
    0 0 0; ...
    0 0 0 ...
    ];
RANGE_THETA = [0 1; -1 1; -1 1];                    % Range of position of moved axis
DELTA_THETA = [0.225; 0.225; 0.36];                 % Position change per step
TIME_OMEGA_RISE = 5 * DELTA_THETA;                  % Time for change of position
TIME_THETA_STEP = 1;                                % Time for one position (including TIME_OMEGA_RISE)
OUTPUT_FOLDER = ...                                 % Output folder for trajectory
    'trajectory/traj/stiffness';


% Add necesarry path and create output folder for trajectory
addpath(pathdef_local)
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end


% Load Robot constraints
conStruct = loadRobotConstraints();
theta_max = conStruct.theta_max(AXIS); % Maximum and minimum allowed theta


% Sample time
tSample = 1 / SAMPLE_FREQ;

%% Build trajectory


% Get position range, position step length and velocity time
rangeTheta = theta_max .* RANGE_THETA(AXIS,:);
deltaTheta = DELTA_THETA(AXIS);
omegaRise = TIME_OMEGA_RISE(AXIS)

% Get time vector
timeRise = deltaTheta / omegaRise;
timeVec_rise = (tSample:tSample:timeRise)';
timeVec_static = (tSample:tSample:TIME_THETA_STEP-timeVec_rise(end))'; % Time vector for rise


% Get start and end position according to step size and robot constraints
if rangeTheta(1) >= 0
    theta_start = floor(rangeTheta(1)/deltaTheta)*deltaTheta;
else
    theta_start = ceil(rangeTheta(1)/deltaTheta)*deltaTheta;
end
if rangeTheta(2) >= 0
    theta_end = floor(rangeTheta(2)/deltaTheta)*deltaTheta;
else
    theta_end = ceil(rangeTheta(2)/deltaTheta)*deltaTheta;
end
theta_steps = (theta_start:deltaTheta:theta_end)'; % Rise trajectory


% Combine complete trajectory
theta = [];
for iStep = 1:length(theta_steps)
    if iStep == 1
        theta = [ ...
            theta; ...
            theta_start + 0.*timeVec_rise; ...
            theta_steps(iStep) + 0.*timeVec_static ...
            ];
    else
        theta = [ ...
            theta; ...
            theta(end) + omegaRise.*timeVec_rise; ...
            theta_steps(iStep) + 0.*timeVec_static ...
            ];
    end
end

theta = [ ...
    theta; ...
    flip(theta) ...
    ];

%% Save trajectory


% Number of sample points
nSamples = length(theta);

% Write trajectory to struct
% Time vector
traj.time = (tSample:tSample:tSample*nSamples)';

% Position vector
if AXIS == 1
    traj.theta = [ ...
        theta, ...
        ones(nSamples,1) .* THETA_OTHER_AXIS(AXIS,2), ...
        ones(nSamples,1) .* THETA_OTHER_AXIS(AXIS,3) ...
        ];
elseif AXIS == 2
    traj.theta = [ ...
        ones(nSamples,1) .* THETA_OTHER_AXIS(AXIS,1), ...
        theta, ...
        ones(nSamples,1) .* THETA_OTHER_AXIS(AXIS,3) ...
        ];
elseif AXIS == 3
    traj.theta = [ ...
        ones(nSamples,1) .* THETA_OTHER_AXIS(AXIS,1), ...
        ones(nSamples,1) .* THETA_OTHER_AXIS(AXIS,2), ...
        theta ...
        ];
else
    error('Check value of AXIS')
end

% Compute omega by finite difference
traj.omega = [ ...
    [0; diff(traj.theta(:,1))], ...
    [0; diff(traj.theta(:,2))], ...
    [0; diff(traj.theta(:,3))] ...
    ] .* SAMPLE_FREQ;


% Save experiment trajectory
save([OUTPUT_FOLDER, '/traj_stiffness_axis', num2str(AXIS), '.mat'], '-struct', 'traj');

%% Plot trajectory


% Theta
plot(traj.time,traj.theta)
legend('\theta_1','\theta_2','\theta_3')

% Omega
plot(traj.time,traj.omega)
legend('\omega_1','\omega_2','\omega_3')