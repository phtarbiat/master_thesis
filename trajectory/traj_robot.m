%% Creation of trajectory for identifying the base parameters of the rigid body model
% The script finds a finit fourier series as the excitation trajectory of the 
% robot that minimizes the condition number of the observation matrix of the rigid 
% body model. Procedure follows [Swevers1997].
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich
%
% Source:
% [Swevers1997]:    Swevers, J.; Ganseman, C.; Schutter, J. de; van
%                   Brussel, H. | Generation of Periodic Trajectories 
%                   for Optimal Robot Excitation | 1997


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Define global variables
global paramStruct
global timeVec
global nonlconFun_fourierSeries


% Trajectory settings 
ROBOT_STRUCTURE = 1;                            % Roboter structure
NUM_FOURIER_TERMS = 5;                          % Number of fourier series terms
TIME_PERIOD = 10;                               % Time of period of fourier series
SAMPLE_FREQ = 500;                              % Sampling frequency measurment
SAMPLE_FREQ_TRAJ = 100;                         % Sampling frequency trajectory
TIME_WAIT = 1;                                  % Waiting time at start and end                     
SOLVER = 2;                                     % Solver for finding optimal trajectory
OUTPUT_FOLDER = ...                             % Output folder for trajectory
    'trajectory/traj/robot';


% Create output folder for trajectory
if ~exist(OUTPUT_FOLDER,'dir')
    mkdir(OUTPUT_FOLDER);
end


% Load kinematic parameters and robot constraints
paramStruct = loadRobotKinematic(ROBOT_STRUCTURE);
conStruct = loadRobotConstraints();


% Get time vector for measurment
tSample = 1 / SAMPLE_FREQ;
tEnd = TIME_PERIOD + 2*TIME_WAIT;
timeVec = (tSample:tSample:TIME_PERIOD)';


% Base frequency of fourier series
omega_fourier = (2 * pi) / TIME_PERIOD;

%% Define constraints of the Robot


% Symbolic variables of fourier series
syms a [NUM_FOURIER_TERMS,3] 'real';
syms b [NUM_FOURIER_TERMS,3] 'real';
syms t 'real';


% Indices of the fourier series coefficients
idxA = 1:(NUM_FOURIER_TERMS*3);
idxB = ((NUM_FOURIER_TERMS*3)+1):(NUM_FOURIER_TERMS*6);


% Load robot constraints
theta_0 = zeros(3,1);
omega_0 = zeros(3,1);
domega_0 = zeros(3,1);

theta_max = conStruct.theta_max;
omega_max = conStruct.omega_max;
domega_max = conStruct.domega_max;
tcp_zmin = conStruct.tcp_zmin;


% Define constraint struct
constraints = struct;

% Inital position constraints
constraints.theta_0.beq = deg2rad(theta_0);
Aeq = zeros(3,idxB(end));
for iJoint = 1:3
    for iN = 1:NUM_FOURIER_TERMS

        Aeq(iJoint,idxB(((iJoint-1)*NUM_FOURIER_TERMS)+iN)) = 1/(iN*omega_fourier);

    end
end
constraints.theta_0.Aeq = Aeq;

% Inital omega constraints
constraints.omega_0.beq = deg2rad(omega_0);
Aeq = zeros(3,idxB(end));
for iJoint = 1:3

    Aeq(iJoint,idxA(((iJoint-1)*NUM_FOURIER_TERMS)+1:((iJoint-1)*NUM_FOURIER_TERMS)+NUM_FOURIER_TERMS)) = 1;

end
constraints.omega_0.Aeq = Aeq;

% Inital domega constraints
constraints.domega_0.beq = deg2rad(domega_0);
Aeq = zeros(3,idxB(end));
for iJoint = 1:3
    for iN = 1:NUM_FOURIER_TERMS

        Aeq(iJoint,idxB(((iJoint-1)*NUM_FOURIER_TERMS)+iN)) = iN*omega_fourier;

    end
end
constraints.domega_0.Aeq = Aeq;

% Maximum theta constraints
c = sym(zeros(3,1));
for iJoint = 1:3
    for iN = 1:NUM_FOURIER_TERMS

        c(iJoint) = c(iJoint) ...
            + (1 / (omega_fourier * iN)) ...
            * sqrt((a(iN,iJoint).^2) + (b(iN,iJoint).^2));

    end
end
constraints.theta_max.c = c - deg2rad(theta_max);
thetaAmplitude = c;

% Maximum omega constraints
c = sym(zeros(3,1));
for iJoint = 1:3
    for iN = 1:NUM_FOURIER_TERMS

        c(iJoint) = c(iJoint) ...
            + sqrt((a(iN,iJoint).^2) + (b(iN,iJoint).^2));

    end
end
constraints.omega_max.c = c - deg2rad(omega_max);

% Maximum domega constraints
c = sym(zeros(3,1));
for iJoint = 1:3
    for iN = 1:NUM_FOURIER_TERMS

        c(iJoint) = c(iJoint) ...
            + (omega_fourier * iN) ...
            * sqrt((a(iN,iJoint).^2) + (b(iN,iJoint).^2));

    end
end
constraints.domega_max.c = c - deg2rad(domega_max);

% TCP z-position constraint
c = (paramStruct.r_P2P3(1) * cos(thetaAmplitude(2))) ... % Z-position link 2
    + (paramStruct.r_P3P4(1) * cos(thetaAmplitude(2) + thetaAmplitude(3))); % Z-position link 3
constraints.tcp_zmin.c = tcp_zmin - c;


% Combine constraints
Aeq = [ ...
    constraints.theta_0.Aeq; ...
    constraints.omega_0.Aeq; ...
    constraints.domega_0.Aeq ...
    ];
beq = [ ...
    constraints.theta_0.beq; ...
    constraints.omega_0.beq; ...
    constraints.domega_0.beq ...
    ];
c = [ ...
    constraints.theta_max.c; ...
    constraints.omega_max.c; ...
    constraints.domega_max.c; ...
    constraints.tcp_zmin.c ...
    ];


% Save nonlinear constraints as function
nonlconFun_fourierSeries = matlabFunction(c,'Vars', {[a(:)' b(:)']});

%% Find Finite Fourier Series that minimizes the condition number of the observation matrix W


if SOLVER == 1
    % Patternsearch
    x0 = ones(1,30);
    options = optimoptions('patternsearch','Display','iter','ConstraintTolerance',0);%,'MaxFunctionEvaluations',1e10);
    param = patternsearch(@objFun,ones(1,NUM_FOURIER_TERMS*3*2),[],[],Aeq,beq,[],[],@nonlcon,options)
    conW = calcCondNum(param)
else
    % Genetic Algorithm
    options = optimoptions('ga','Display','iter');
    param = ga(@objFun,30,[],[],Aeq,beq,[],[],@nonlcon,options)
    conW = objFun(param)
end

%% Build trajectory from coefficients


% Create symbolic Finite Fourier Series
thetaTraj = sym(zeros(3,1));
omegaTraj = sym(zeros(3,1));
domegaTraj = sym(zeros(3,1));

for iJoint = 1:3
    for iN = 1:NUM_FOURIER_TERMS
        thetaTraj(iJoint) = thetaTraj(iJoint) ...
            + ((a(iN,iJoint) / (omega_fourier * iN)) ...
            * sin(omega_fourier * iN * t)) ...
            - ((b(iN,iJoint) / (omega_fourier * iN)) ...
            * cos(omega_fourier * iN * t));
        
        omegaTraj(iJoint) = omegaTraj(iJoint) ...
            + (a(iN,iJoint) * cos(omega_fourier * iN * t)) ...
            + (b(iN,iJoint) * sin(omega_fourier * iN * t));

        domegaTraj(iJoint) = omegaTraj(iJoint) ...
            - (a(iN,iJoint) * omega_fourier * iN * sin(omega_fourier * iN * t)) ...
            + (b(iN,iJoint) * omega_fourier * iN * cos(omega_fourier * iN * t));
    end
end

thetaSym = thetaTraj';
omegaSym = omegaTraj';
domegaSym = domegaTraj';


% Subtitute coefficients in trajectories
oldSubsVec = [a(:); b(:)];

for iParam = 1:length(param)
    thetaSym = subs(thetaSym,oldSubsVec(iParam),param(iParam));
    omegaSym = subs(omegaSym,oldSubsVec(iParam),param(iParam));
    domegaSym = subs(domegaSym,oldSubsVec(iParam),param(iParam));
end


% Get final trajectories out of symbolic trajectories
thetaFun = matlabFunction(thetaSym);
theta_rad = arrayfun(thetaFun,timeVec,'UniformOutput',false);
theta_rad  = cell2mat(theta_rad );
omegaFun = matlabFunction(omegaSym);
omega_rad  = arrayfun(omegaFun,timeVec,'UniformOutput',false);
omega_rad  = cell2mat(omega_rad);
domegaFun = matlabFunction(domegaSym);
domega_rad  = arrayfun(domegaFun,timeVec,'UniformOutput',false);
domega_rad  = cell2mat(domega_rad);


% Convert trajectories to degree
theta = rad2deg(theta_rad);
omega = rad2deg(omega_rad);
domega = rad2deg(domega_rad);


% Add waiting time at start and end
theta = [ ...
    zeros((TIME_WAIT*SAMPLE_FREQ),3); ...
    theta; ...
    zeros((TIME_WAIT*SAMPLE_FREQ),3) ...
    ];
omega = [ ...
    zeros((TIME_WAIT*SAMPLE_FREQ),3); ...
    omega; ...
    zeros((TIME_WAIT*SAMPLE_FREQ),3) ...
    ];
domega = [ ...
    zeros((TIME_WAIT*SAMPLE_FREQ),3); ...
    domega; ...
    zeros((TIME_WAIT*SAMPLE_FREQ),3) ...
    ];

% Redefine time vector with waiting time
timeVec = (tSample:tSample:tEnd)';

%% Save trajectory


% Write trajectory to struct
% Time vector
traj.time = downsample(timeVec,SAMPLE_FREQ/SAMPLE_FREQ_TRAJ);

% Position, velocity, acceleration vector
traj.theta = downsample(theta,SAMPLE_FREQ/SAMPLE_FREQ_TRAJ);
traj.omega = downsample(omega,SAMPLE_FREQ/SAMPLE_FREQ_TRAJ);
traj.domega = downsample(domega,SAMPLE_FREQ/SAMPLE_FREQ_TRAJ);

% coefficients and condition number
traj.param = param;
traj.conW = conW;


% Save experiment trajectory
save([OUTPUT_FOLDER, '/traj_robot.mat'], '-struct', 'traj');

%% Plot trajectory


% Theta
plot(timeVec,theta)
hold on
yline(theta_max,'--')
yline(-theta_max,'--')
hold off
legend('1','2','3')
title('theta')

% Omega
plot(timeVec,omega)
hold on
yline(omega_max,'--')
yline(-omega_max,'--')
hold off
title('omega')

% DOmega
plot(timeVec,domega)
hold on
yline(domega_max,'--')
yline(-domega_max,'--')
hold off
title('domega')

% TCP position
zPos = (paramStruct.r_P2P3(1) * cosd(theta(:,2))) ...
+ (paramStruct.r_P3P4(1) * cosd(theta(:,2)+theta(:,3)));
xPos_ = (paramStruct.r_P2P3(1) * sind(theta(:,2))) ...
+ (paramStruct.r_P3P4(1) * sind(theta(:,2)+theta(:,3)));
xPos = xPos_ .* cosd(theta(:,1));
yPos = xPos_ .* (-sind(theta(:,1)));
tcpPos = [xPos, yPos, zPos];

plot3(tcpPos(:,1),tcpPos(:,2),tcpPos(:,3),'-')
hold on
surf(linspace(-0.25,0.25,100),linspace(-0.2,0.2,100),-0.0865*ones(100,100))
surf(linspace(-0.5,0.5,100),linspace(-0.5,0.5,100),-0.5*ones(100,100))
hold off

%% Functions

function objFunValue = objFun(ab)

    global paramStruct
    global timeVec

    objFunValue = objFun_robotTraj_mex(paramStruct,ab,timeVec);

end

function [c,ceq] = nonlcon(ab)

    global nonlconFun_fourierSeries

    c = nonlconFun_fourierSeries(ab);
    ceq = [];
    
end