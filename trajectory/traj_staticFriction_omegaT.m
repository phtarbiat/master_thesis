%% Creation of trajectory for identifying the velocity and temperature dependent joint friction
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


AXIS = 1;                                                       % Robot axis
SAMPLE_FREQ = 100;                                              % Sampling frequency
THETA_OTHER_AXIS = [ ...                                        % Constant position of other axes
    0 0 0; ...
    0 0 0; ...
    0 0 0 ...
    ];
NUM_OMEGA = 20;                                                 % Number of tested velocities
OMEGA_MAX = 100;                                                % Maximum velocity
TIME_OMEGA = 1.5;                                               % Time per velocity
TIME_ACCELERATION = 0.25;                                       % Waiting time for acceleration
TIME_WAIT_BETWEEN = 1;                                          % Waiting time between velocity direction change
TIME_WAIT_END = 1;                                              % Waiting time at end of velocity
TIME_TRAJ = 180;                                                % Total trajectory time
OUTPUT_FOLDER = ...                                             % Output folder for trajectory
    'trajectory/traj/staticFriction_omegaT';


% Create output folder for trajectory
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end


% Sample time
tSample = 1 / SAMPLE_FREQ; % Sample time

%% Build friction identification trajectory


% Scale spacing between tested velocities (do more tests at low velocities)
scalingFun = (1:NUM_OMEGA).^2' ./ NUM_OMEGA^2;
omegaTested = scalingFun * OMEGA_MAX


% Get time vector
time_1omega = 2 * (TIME_OMEGA + TIME_ACCELERATION) % total time for one omega
theta_0 = - OMEGA_MAX * 0.5 * TIME_OMEGA
timeVec_omegarise = (tSample:tSample:TIME_OMEGA)'; % Time vector for one omega


% Trajectory for all tested omegas
theta_omegaTest = [];
for iOmega = 1:NUM_OMEGA
    
    theta_omegaRise = theta_0 + omegaTested(iOmega) .* timeVec_omegarise;
    theta_omegaTest = [ ...
        theta_omegaTest; ...
        theta_0 .* ones(TIME_ACCELERATION*SAMPLE_FREQ,1); ...
        theta_omegaRise; ...
        theta_omegaRise(end) .* ones(TIME_ACCELERATION*SAMPLE_FREQ,1); ...
        flip(theta_omegaRise); ...
        ];

end

%% Build heat-up trajectory


% Get variables for heat-up
time_omegaTest = length(theta_omegaTest) / SAMPLE_FREQ;
time_heatup = TIME_TRAJ - time_omegaTest - TIME_WAIT_BETWEEN - TIME_WAIT_END;
time_hHalf = time_heatup / 4;
nCylcle_hQuarter = floor(time_hHalf / (2*TIME_OMEGA))


% Trajector for one heat-up quarter
theta_hQuarter = [];
for iCycle = 1:nCylcle_hQuarter

    theta_hQuarter = [ ...
        theta_hQuarter; ...
        theta_omegaRise; ...
        flip(theta_omegaRise); ...
        ];

end


% Wainting time of one heat-up quarter
%theta_hQuarterWait = theta_0 .* ones(length(theta_hQuarter),1);




%% Combine trajectories


% Get waiting time vectors
time_waitBetween = theta_0 .* ones(TIME_WAIT_BETWEEN*SAMPLE_FREQ,1);
time_waitEnd = theta_0 .* ones(TIME_WAIT_END*SAMPLE_FREQ,1);


% Combine trajectories
theta = [...
    theta_omegaTest; ...
    time_waitBetween; ...
    theta_hQuarter; ...
    theta_hQuarter; ...
    theta_hQuarter; ...
    theta_hQuarter; ...
    time_waitEnd ...
    ];

%% Save trajectories


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
save([OUTPUT_FOLDER, '/traj_staticFriction_omegaT_axis', num2str(AXIS), '.mat'], '-struct', 'traj');

%% Plot Trajectory


% Theta
plot(traj.time,traj.theta)
legend('\theta_1','\theta_2','\theta_3')

% Omega
plot(traj.time,traj.omega)
legend('\omega_1','\omega_2','\omega_3')