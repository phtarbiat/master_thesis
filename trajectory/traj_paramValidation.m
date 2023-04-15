%% Creation of trajectory for the complete model validation
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Trajectory settings
THETA_POINTS = [ ...                               % Configurations of trajectory
    0 0 0; ...
    45 -45 -80; ...
    -45 45 80;
    -60 -80 30;
    -135 45 -90;
    0 0 0 ...
    ];
SAMPLE_FREQ = 100;                                  % Sampling frequency
TIME_WAIT = 1;                                      % Waiting time between configurations
TIME_MOVE = 4;                                      % Time for movement
OUTPUT_FOLDER = ...                                 % Output folder for trajectory
    'trajectory/traj/validation';


% Create output folder for trajectory
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end


% Load Robot constraints
conStruct = loadRobotConstraints();
omegaMax = conStruct.omega_max;
domegaMax = conStruct.domega_max;


% Sample time
tSample = 1 / SAMPLE_FREQ;

%% Build trajectory


% Calculate trajecotry of 5th order polynoms
theta = zeros(TIME_WAIT*SAMPLE_FREQ,3);
omega = zeros(TIME_WAIT*SAMPLE_FREQ,3);
domega = zeros(TIME_WAIT*SAMPLE_FREQ,3);

for iPP = 1:size(THETA_POINTS,1)-1

    points = [THETA_POINTS(iPP,:)', THETA_POINTS(iPP+1,:)'];

    [~,curTheta,curOmega,curDOmega,~,~,~] = generate5PolyTrajectory(points, TIME_MOVE, 0, tSample, omegaMax, domegaMax);

    theta = [ ...
        theta; ...
        curTheta; ...
        curTheta(end,:).*ones(TIME_WAIT*SAMPLE_FREQ,3) ...
        ];

    omega = [ ...
        omega; ...
        curOmega; ...
        zeros(TIME_WAIT*SAMPLE_FREQ,3) ...
        ];

    domega = [ ...
        domega; ...
        curDOmega; ...
        zeros(TIME_WAIT*SAMPLE_FREQ,3) ...
        ];

end


% Get time vector
timeVec = (tSample:tSample:length(theta)*tSample)';

%% Save trajectory


% Write trajectory to struct
% Time vector 
traj.time = timeVec;

% Position, velocity, acceleration vector
traj.theta = theta;
traj.omega = omega;
traj.domega = domega;


% Save experiment trajectory
save([OUTPUT_FOLDER, '/traj_validation.mat'], '-struct', 'traj');

%% Plot trajectory


% Theta
figure
plot(timeVec,theta)

% Omega
figure
plot(timeVec,omega)

% DOmega
figure
plot(timeVec,domega)

% TCP position
paramStruct = loadRobotKinematic(1);
zPos = (paramStruct.r_P2P3(1) * cosd(theta(:,2))) ...
+ (paramStruct.r_P3P4(1) * cosd(theta(:,2)+theta(:,3)));
xPos_ = (paramStruct.r_P2P3(1) * sind(theta(:,2))) ...
+ (paramStruct.r_P3P4(1) * sind(theta(:,2)+theta(:,3)));
xPos = xPos_ .* cosd(theta(:,1));
yPos = xPos_ .* (-sind(theta(:,1)));
tcpPos = [xPos, yPos, zPos];

figure
hold on
plot3(tcpPos(1,1),tcpPos(1,2),tcpPos(1,3),'rx')
plot3(tcpPos(:,1),tcpPos(:,2),tcpPos(:,3),'b-')
surf(linspace(-0.25,0.25,100),linspace(-0.2,0.2,100),-0.0865*ones(100,100))
surf(linspace(-0.5,0.5,100),linspace(-0.5,0.5,100),-0.5*ones(100,100))
hold off