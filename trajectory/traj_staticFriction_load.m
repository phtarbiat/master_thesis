%% Creation of trajectory for identifying the load dependent joint friction
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Trajectory settings 
AXIS = 1;                                                   % Robot axis
SAMPLE_FREQ = 100;                                          % Sampling frequency
THETA_OTHER_AXIS = [ ...                                    % Constant position of other axes
    0 78 0; ...
    0 0 0; ...
    0 0 0 ...
    ];
RANGE_THETA = [0 1; -1 1; -1 1];                            % Range of position of moved axis
OMEGA = [1; 1; 1];                                          % Velocity of trajectory
TIME_WAIT_START = 0.25;                                     % Waiting time at start
TIME_WAIT_BETWEEN = 0.25;                                   % Waiting time between change of movement direction
OUTPUT_FOLDER = ...                                         % Output folder for trajectory
    'trajectory/traj/staticFriction_load'; 


% Create output folder for trajectory
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end


% Load Robot constraints
conStruct = loadRobotConstraints();
theta_max = conStruct.theta_max(AXIS) - 1; % Maximum and minimum allowed theta


% Sample time
tSample = 1 / SAMPLE_FREQ;

%% Build trajectory


% Get trajectory velocity and position range
omega = OMEGA(AXIS);
rangeTheta = theta_max .* RANGE_THETA(AXIS,:);


% Get time vector
time_rise = (rangeTheta(2)-rangeTheta(1)) / omega; % Rise time
timeVec_rise = (tSample:tSample:time_rise)'; % Time vector for rise


% Get part of trajectory
theta_rise = rangeTheta(1) + omega .* timeVec_rise; % Rise trajectory


% Combine complete trajectory
theta = [ ...
    rangeTheta(1).*ones(TIME_WAIT_START*SAMPLE_FREQ,1); ...
    theta_rise; ...
    theta_rise(end).*ones(TIME_WAIT_BETWEEN*SAMPLE_FREQ,1); ...
    flip(theta_rise); ...
    rangeTheta(1).*ones(TIME_WAIT_START*SAMPLE_FREQ,1); ...
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
save([OUTPUT_FOLDER, '/traj_staticFriction_load_axis', num2str(AXIS), '.mat'], '-struct', 'traj');

%% Plot trajectory


% Theta
plot(traj.time,traj.theta)
legend('\theta_1','\theta_2','\theta_3')

% Omega
plot(traj.time,traj.omega)
legend('\omega_1','\omega_2','\omega_3')