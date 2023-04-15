%% Structured Modeling of the 3-DOF Flexible Joint Robot using base parameters
% * Joint rotation axes (in local frames): Z Z Z
% * Modeling with joint angles / minimal coordinates and angular velocities
% * Lagrangian Form
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich
%
% Sources:
% [Gautier1990]:    Gautier, M. | Numerical calculation of the base inertial
%                   parameters of robots | 1990
%
% [Khalil2004]:     Khalil, W.; Dombre, E. | Modeling, identification & control
%                   of robots | 2004
%
% [MLS94]:          Murray, R.M.; Li, Z.; Sastry, S. | A mathematical
%                   introduction to robotic manipulation
%
% [Zollo2015]:      Zollo, Loredana; Lopez, Edoardo; Spedaliere, Luca; 
%                   Garcia Aracil, Nicolas; Guglielmelli, Eugenio |
%                   Identification of Dynamic Parameters for Robots with
%                   Elastic Joints | 2015


% Preparation
clearvars
%close all
addpath(pathdef_local)

%% Model Generation Settings
% The script generates a symbolic model in form of a struct with the standardized 
% fields, which is saved as a mat file.


curPath = pwd;

% Name and path of the saved models
MDL_NAME = 'mdl_flexible_joint_robot_EL';
MDL_PATH = '../robot_modeling_flexible/model/';

% Simulink settings
GENERATE_FUNCTIONS = true;
GENERATE_CONTROLLER = true;

%% Variables


% General variables
%syms t real
syms g 'real'; % gravitation constant

% Kinematic parameters
syms i_g [3,1] 'positive'; % joint gear ratio

% Variables as purely symbolic variables
% (motor variables given for after the transmission)
syms theta_l [3,1] 'real';
syms theta_m [3,1] 'real'; % Motor position after transmission
theta = [theta_l; theta_m];

syms omega_l [3,1] 'real';
syms omega_m [3,1] 'real'; % Motor velocity after transmission
omega = [omega_l; omega_m];

syms domega_l [3,1] 'real';
syms domega_m [3,1] 'real';
domega = [domega_l; domega_m];


% kinematic error of the joints
syms kinError [3,1] 'real';

% Vectors from pivot point to pivot point (in local frames)
r_P0P1 = zeros(3,1); % in frame 0
syms r_P1P2 [3,1] 'real'; % in frame 1
syms r_P2P3 [3,1] 'real'; % in frame 2
syms r_P3P4 [3,1] 'real'; % in frame 3


% Dynamic Parameters

syms m [3,1] 'positive'; % masses of bodies (link i and motor i+1)
syms k [3,1] 'positive'; % joint elasticity

% Product of mass and vector from pivot point to center of mass of the body (link i + motor i+1) (in local frames)
syms mr_P1S1 [3,1] 'real'; % in frame 1
syms mr_P2S2 [3,1] 'real'; % in frame 2
syms mr_P3S3 [3,1] 'real'; % in frame 3

% Inertia tensors of the bodies (symmetric) (in local frames)
% https://de.mathworks.com/matlabcentral/answers/385581-how-to-create-a-symbolic-matrix-which-is-symmetric
syms J_1 [3,3] 'positive';
syms J_2 [3,3] 'positive';
syms J_3 [3,3] 'positive';

J_1 = tril(J_1,0) + tril(J_1,-1).';
J_2 = tril(J_2,0) + tril(J_2,-1).';
J_3 = tril(J_3,0) + tril(J_3,-1).';

% Inertia of the rotor in rotational direction (in local frames)
syms J_m1 'positive';
syms J_m2 'positive';
syms J_m3 'positive';

%% Kinematics / Rotation matrices
%% Rotation axes Z Z Z


% Rotation matrices from joint angles
R_0l1 = rotsymz(theta_l(1));
R_l1l2 = rotsymx(0.5*sym(pi)) * rotsymz(theta_l(2) + 0.5*sym(pi));
R_l2l3 = rotsymz(theta_l(3));


% Homogenous transformation matrices
g_0l1 = [R_0l1, r_P0P1 ; 0 0 0 1];
g_l1l2 = [R_l1l2, r_P1P2 ; 0 0 0 1];
g_l2l3 = [R_l2l3, r_P2P3 ; 0 0 0 1];

g_0l2 = g_0l1 * g_l1l2;
g_0l3 = g_0l2 * g_l2l3;


% Save in struct form
q_i = {theta_l(1) theta_l(2) theta_l(3)};
r_PiPj_vec = {r_P1P2, r_P2P3, r_P3P4};
mr_PiSi_vec = {mr_P1S1 mr_P2S2 mr_P3S3};
R_pi_vec = {R_0l1, R_l1l2, R_l2l3};
g_0li = {g_0l1 g_0l2 g_0l3};
m_i = {m(1) m(2) m(3)};
J_i = {J_1 J_2 J_3};
J_mi = {J_m1 J_m2 J_m3};

%% Mass Matrix
% Calculation after [MLS94, pp. 115] and [Zollo2015, pp. 3-4]


J_Si_b = cell(3,1);
J_Ti_b = cell(3,1);
J_Ri_b = cell(3,1);
M_gen_i = cell(3,1);
M_i = cell(3,1);
S_i = cell(3,1);

for iBody = 1:3

    % Calculate Jacobians with local function (body fixed only)
    [~,J_Si_b{iBody}] = manipJacobianFromSE3_jointAngles(g_0li{iBody}, theta_l);
    J_Ti_b{iBody} = J_Si_b{iBody}(1:3,1:3);
    J_Ri_b{iBody} = J_Si_b{iBody}(4:6,1:3);

    % Generalized inertia matrices
    M_gen_i{iBody} = blkdiag(m_i{iBody}*eye(3),J_i{iBody});
    
    % Calculate mass matrix of links for every body
    M_i{iBody} = J_Si_b{iBody}' * M_gen_i{iBody} * J_Si_b{iBody};
    M_i{iBody} = M_i{iBody} ...
        + J_Ri_b{iBody}' * skew(mr_PiSi_vec{iBody}) * J_Ti_b{iBody} ...
        - J_Ti_b{iBody}' * skew(mr_PiSi_vec{iBody}) * J_Ri_b{iBody};

    % Calculate coupling terms between links and motors
    if iBody == 1
        S_i{iBody} = zeros(3,1);
    else
        S_i{iBody} = i_g(iBody) * J_mi{iBody} * J_Ri_b{iBody-1}' ...
            * (R_pi_vec{iBody} * [0 ; 0 ; 1]);
    end
end

% Mass Matrix of links
M = M_i{1} + M_i{2} + M_i{3};
% Mass Matrix of motors
B = diag([J_m1*i_g(1)^2 J_m2*i_g(2)^2 J_m3*i_g(3)^2]);
% Mass Matrix of couplings between links and motors
S = [S_i{1} S_i{2} S_i{3}];

% Combine Mass Matrices
M_com = [M S ; S' B];
M_red = blkdiag(M,B);

% Kinetic Energy
T_com = 0.5 * omega' * M_com * omega;
T_red = 0.5 * omega' * M_red * omega;

%% Coriolis Matrix
% See [MLS94]


% Derivatives of the total mass matrix
M_com_dq = massMatrixDerivatives(M_com, theta);

% Christoffel symbols as anonymous function
Gamma = @(i,j,k) 1/2 * ( ...
    squeeze(M_com_dq(i,j,k)) + ...
    squeeze(M_com_dq(i,k,j)) - ...
    squeeze(M_com_dq(j,k,i)) ...
    );

% Assemble Coriolis Matrix
syms C_com [6,6];
for iRow = 1:6
    for iCol = 1:6
        C_com(iRow, iCol) = ...
            Gamma(iRow, iCol, 1)*omega_l(1) + ...
            Gamma(iRow, iCol, 2)*omega_l(2) + ...
            Gamma(iRow, iCol, 3)*omega_l(3) + ...
            Gamma(iRow, iCol, 4)*omega_m(1) + ...
            Gamma(iRow, iCol, 5)*omega_m(2) + ...
            Gamma(iRow, iCol, 6)*omega_m(3);
    end
end

C_com = simplify(C_com);
C = C_com(1:3,1:3);
C_red = blkdiag(C,zeros(3,3));

%% Gravitational and Joint Stiffness vector


% Calculate product of center of gravity and mass in inertia frame
mr_S1 = m(1)*g_0l1(1:3,4) + g_0l1(1:3,1:3)*mr_P1S1;
mr_S2 = m(2)*g_0l2(1:3,4) + g_0l2(1:3,1:3)*mr_P2S2;
mr_S3 = m(3)*g_0l3(1:3,4) + g_0l3(1:3,1:3)*mr_P3S3;

% Potential Energy
U_g = ...
    + g * unit(3,3)' * mr_S1 ...
    + g * unit(3,3)' * mr_S2 ...
    + g * unit(3,3)' * mr_S3;

K = diag(k);
U_k = 0.5 * (theta_m - theta_l)' * K * (theta_m - theta_l);

U = U_g + U_k;

% Vector of gravity torque
gVec = simplify(jacobian(U_g, theta_l)');

% Vector of spring torque
kVec = K * ((theta_m - theta_l) - kinError);

% input forces
u = sym('u', [3,1], 'real');

%% Calculate Regressor Matrices


% All inertia parameters
paramVec = [...
    J_1(1,1); J_1(1,2); J_1(1,3); J_1(2,2); J_1(2,3); J_1(3,3); ...
    mr_P1S1; m(1); J_m1; ...
    J_2(1,1); J_2(1,2); J_2(1,3); J_2(2,2); J_2(2,3); J_2(3,3); ...
    mr_P2S2; m(2); J_m2; ...
    J_3(1,1); J_3(1,2); J_3(1,3); J_3(2,2); J_3(2,3); J_3(3,3); ...
    mr_P3S3; m(3); J_m3; ...
    ];

% Define symbolic variables of the regression matrices
syms YRobot_com [6,length(paramVec)];
syms YRobot_red [6,length(paramVec)];
syms YMdomega_com [6,length(paramVec)];
syms YMdomega_red [6,length(paramVec)];
syms YComega_com [6,length(paramVec)];
syms YComega_red [6,length(paramVec)];
syms YGVec [6,length(paramVec)];

syms tempVar [6,1];
syms tempOmega [6,1];
syms tempDOmega [6,1];

regMat.Robot_com = YRobot_com;
regMat.Robot_red = YRobot_red;
regMat.Mdomega_com = YMdomega_com;
regMat.Mdomega_red = YMdomega_red;
regMat.Comega_com = YComega_com;
regMat.Comega_red = YComega_red;
regMat.gVec = YGVec;

% Output vecotr
tauVec.Robot_com = [zeros(3,1) ; u];
tauVec.Robot_red = [zeros(3,1) ; u];
tauVec.Mdomega_com = [zeros(3,1) ; u];
tauVec.Mdomega_red = [zeros(3,1) ; u];
tauVec.Comega_com = [zeros(3,1) ; u];
tauVec.Comega_red = [zeros(3,1) ; u];
tauVec.gVec = [zeros(3,1) ; u];

% Robot ODE and ODE terms
odeVec.Robot_com = M_com * domega + C_com * omega + [gVec; zeros(3,1)] + [-kVec; kVec] + tempVar;
odeVec.Robot_red = M_red * domega + C_red * omega + [gVec; zeros(3,1)] + [-kVec; kVec] + tempVar;
odeVec.Mdomega_com = M_com * tempDOmega + tempVar;
odeVec.Mdomega_red = M_red * tempDOmega + tempVar;
odeVec.Comega_com = C_com * tempOmega + tempVar;
odeVec.Comega_red = C_red * tempOmega + tempVar;
odeVec.gVec = [gVec; zeros(3,1)] + tempVar;

odeNames = {'Robot_com' 'Robot_red' 'Mdomega_com' 'Mdomega_red' 'Comega_com' 'Comega_red' 'gVec'};

% Build regressor matrices by extracting the coefficients of the inertia
% parameters from the robot ode
for iOde = 1:length(odeNames)

    for iRow = 1:6
    
        % Load curent row
        curRow = odeVec.(odeNames{iOde})(iRow);
    
        for iParam = 1:length(paramVec)
            
            % Get coefficients from inertia parameter
            curCoeff = coeffs(curRow,paramVec(iParam));
    
            if length(curCoeff) > 2
                error('The differential equation can not be linearized with the chosen parameter vector')
            elseif length(curCoeff) > 1
                % Save linear coefficient in regressor matrix
                regMat.(odeNames{iOde})(iRow,iParam) = curCoeff(2);
            else
                regMat.(odeNames{iOde})(iRow,iParam) = 0;
            end
    
            % Delete extracted term from curent row
            curRow = curCoeff(1);
    
        end
    
        % Add residual to output vector
        tauVec.(odeNames{iOde})(iRow) = tauVec.(odeNames{iOde})(iRow) - curRow;
    
    end

    tauVec.(odeNames{iOde}) = tauVec.(odeNames{iOde}) + tempVar;

end


% Identify unobservable parameters (0 coulumns) and eliminated them
idxUnidentifiable = all(regMat.Robot_com == 0);

paramVec_red = paramVec;
paramVec_red(idxUnidentifiable) = [];

for iOde = 1:length(odeNames)
    regMat.(odeNames{iOde})(:,idxUnidentifiable) = [];
end

%% Generate random Trajectories for finding base parameters


% number of trajectories created
nTraj = 10;

% Finite Fourier Series parameters
nFourier = 5;
freqFourier = 0.1;
omegaFourier = 2 * pi * freqFourier;
tSample = 0.1;
tPeriod = 1 / freqFourier;

time = (0:tSample:tPeriod-tSample)';

% Robot constraints
dqMax = deg2rad([108; 132; 330]);

% Inital position
theta_0 = [0; 0; 0];

% Create trajectories
thetaTraj = cell(nTraj,1);
omegaTraj = cell(nTraj,1);
domegaTraj = cell(nTraj,1);

for iTraj = 1:nTraj

    curTheta = zeros(length(time),3);
    curOmega = zeros(length(time),3);
    curDOmega = zeros(length(time),3);
    
    for iJoint = 1:3

        % Choose random coefficients for the trajectory and scale them so
        % that the initial conditions are theta(0)=q_0 and omega(0)=0
        ab2R = randi([0 100],2*nFourier,1);
        ab2 = (ab2R ./ sum(ab2R));
        ab = sqrt(ab2) * (dqMax(iJoint));
        a = ab(1:nFourier);
        b = ab(nFourier+1:2*nFourier);
        
        bQ = b ./ (1:nFourier)';
        bQ = bQ - (sum(bQ)/nFourier);
        b = bQ .* (1:nFourier)';
        
        a = a - (sum(a)/nFourier);
        

        curTheta(1,iJoint) = theta_0(1);
        
        for iFourier = 1:nFourier
        
            c = omegaFourier * iFourier;
        
            curTheta(:,iJoint) = curTheta(:,iJoint) ...
                + (a(iFourier) / c) * sin(c * time) ...
                + (b(iFourier) / c) * cos(c * time);
        
            curOmega(:,iJoint) = curOmega(:,iJoint) ...
                + a(iFourier) * cos(c * time) ...
                - b(iFourier) * sin(c * time);
        
            curDOmega(:,iJoint) = curDOmega(:,iJoint) ...
                - a(iFourier) * c * sin(c * time) ...
                - b(iFourier) * c * cos(c * time);
        
        end
    end

    thetaTraj{iTraj} = curTheta;
    omegaTraj{iTraj} = curOmega;
    domegaTraj{iTraj} = curDOmega;

end

%% Fill Robot Regression Matrix with trajectory data

% Load kinematic robot parameters
paramStruct = loadRobotKinematic(1);

% Substitute kinematic parameters in regression matrix
oldSubsVec = [ ...
    g; i_g; ...
    r_P1P2; r_P2P3; r_P3P4 ...
    ];
newSubsVec = [ ...
    paramStruct.g; paramStruct.i_g; ...
    paramStruct.r_P1P2; paramStruct.r_P2P3; paramStruct.r_P3P4 ...
    ];

Ysubs = regMat.Robot_com;
for iParam = 1:length(oldSubsVec)
    Ysubs = subs(Ysubs,oldSubsVec(iParam),newSubsVec(iParam));
end

% Define fucntion_handle of the regressor matrix
paramVecFun = [ ...
    theta_l1; theta_l2; theta_l3; ...
    omega_l1; omega_l2; omega_l3; ...
    domega_l1; domega_l2; domega_l3; ...
    domega_m1; domega_m2; domega_m3 ...
    ];
YFun = matlabFunction(Ysubs,'Vars',paramVecFun);

% Create observation matrix W from trajectories
WTraj = cell(nTraj,1);

for iTraj = 1:nTraj

    WTraj{iTraj} = [];

    curTheta = thetaTraj{iTraj};
    curOmega = omegaTraj{iTraj};
    curDOmega = domegaTraj{iTraj};

    for iTime = 1:length(time)
        
        curW = YFun( ...
            curTheta(iTime,1), curTheta(iTime,2), curTheta(iTime,3), ...
            curOmega(iTime,1),curOmega(iTime,2),curOmega(iTime,3), ...
            curDOmega(iTime,1),curDOmega(iTime,2),curDOmega(iTime,3), ...
            curDOmega(iTime,1),curDOmega(iTime,2),curDOmega(iTime,3) ...
            );

        WTraj{iTraj} = [WTraj{iTraj}; curW];

    end
     
end

% Combine observation matrices of all trajectories
W = [];
for iTraj = 1:nTraj

    W = [W; WTraj{iTraj}];

end

%% Compute Base Parameter set

% The used procedure for numerically identifying the base parameters of the
% robot is based on [Gautier1990, pp. 2-3]
% Get QR-decomposition from W
[Q,R] = qr(W);

% Find number of base parameters
r = diag(R);
idxX1 = find(abs(r) >= 1);
idxX2 = find(~(abs(r) >= 1));
c = length(paramVec_red);
b = length(idxX1);

% Calculate perbutation matrix
P_T = zeros(c,c);

for iRow = 1:c

    if iRow <= b
        P_T(iRow,idxX1(iRow)) = 1;
    else
        P_T(iRow,idxX2(iRow-b)) = 1;
    end
end

P = P_T';

% Calculate pase parameters and corresbonding regressor matrix
WP = W * P;
[~,R_P] = qr(WP);

R1 = R_P(1:c,1:b);
R2 = R_P(1:c,b+1:end);

T = R1 \ R2;
T(abs(T)<1e-10) = 0;

X1 = paramVec_red(idxX1);
X2 = paramVec_red(idxX2);
paramVecBase = X1 + T * X2 % base parameter

% Numerical calculation of the regressor matrix following [Khalil2004,
% pp. 419, Eq. A5.7]
regMatBase = struct;
for iOde = 1:length(odeNames)
    regMatP = regMat.(odeNames{iOde}) * P;
    regMatBase.(odeNames{iOde}) = regMatP(:,1:b); % base parameters regressor matrix
end

%% Calculate ODE terms (M(q), C(q)) in dependency of the base Parameters


syms baseParam [length(paramVecBase),1];
syms M_comBase [6,6];
syms M_redBase [6,6];
syms C_comBase [6,6];
syms C_redBase [6,6];

mdlBase.M_com = M_comBase;
mdlBase.M_red = M_redBase;
mdlBase.C_com = M_comBase;
mdlBase.C_red = M_redBase;

mdlNames = {'_com' '_red'};

for iMdl = 1:length(mdlNames)

    tempC = regMatBase.(strcat('Comega',mdlNames{iMdl})) * baseParam + tempVar;
    tempM = regMatBase.(strcat('Mdomega',mdlNames{iMdl})) * baseParam + tempVar;

    for iRow = 1:6
    
        curRowC = tempC(iRow);
        curRowM = tempM(iRow);
    
        for iVar = 1:length(tempOmega)
    
            % Get coefficients
            curCoeffC = coeffs(curRowC,tempOmega(iVar));
            curCoeffM = coeffs(curRowM,tempDOmega(iVar));
   
            if length(curCoeffC) > 2
                error('The argument can not be linearized with the chosen linearization vector')
            elseif length(curCoeffC) > 1
                % Save linear coefficient in regressor matrix
                mdlBase.(strcat('C',mdlNames{iMdl}))(iRow,iVar) = curCoeffC(2);
            else
                mdlBase.(strcat('C',mdlNames{iMdl}))(iRow,iVar) = 0;
            end
            if length(curCoeffM) > 2
                error('The argument can not be linearized with the chosen linearization vector')
            elseif length(curCoeffM) > 1
                % Save linear coefficient in regressor matrix
                mdlBase.(strcat('M',mdlNames{iMdl}))(iRow,iVar) = curCoeffM(2);
            else
                mdlBase.(strcat('M',mdlNames{iMdl}))(iRow,iVar) = 0;
            end
    
            curRowC = curCoeffC(1);
            curRowM = curCoeffM(1);
    
        end
        
    end

end


mdlBase.M = mdlBase.M_com(1:3,1:3);
mdlBase.B = mdlBase.M_com(4:6,4:6);
mdlBase.S = mdlBase.M_com(4:6,1:3);
mdlBase.C = mdlBase.C_com(1:3,1:3);

mdlBase.gVec = regMatBase.gVec(1:3,:)*baseParam;
mdlBase.kVec = kVec;

%% Calculate Derivatives of System Terms

syms ddomega_l [3,1] 'real';
syms ddomega_m [3,1] 'real';
ddomega = [ddomega_l; ddomega_m];

% Calculate derivatives of M, C and gVec
% Calculate first derivatives of ode terms
mdlBase.dM = sym(zeros(3,3));
mdlBase.dC = sym(zeros(3,3));
mdlBase.dgVec = sym(zeros(3,1));


for iQ = 1:3

    mdlBase.dM = mdlBase.dM ...
        + (diff(mdlBase.M,theta_l(iQ)) * omega_l(iQ));
    mdlBase.dC = mdlBase.dC ...
        + (diff(mdlBase.C,theta_l(iQ)) * omega_l(iQ)) ...
        + (diff(mdlBase.C,omega_l(iQ)) * domega_l(iQ));
    mdlBase.dgVec = mdlBase.dgVec ...
        + (diff(mdlBase.gVec,theta_l(iQ)) * omega_l(iQ));

end

% Calculate second derivatives of ode terms
mdlBase.ddM = sym(zeros(3,3));
mdlBase.ddC = sym(zeros(3,3));
mdlBase.ddgVec = sym(zeros(3,1));

for iQ = 1:3

    mdlBase.ddM = mdlBase.ddM ...
        + (diff(mdlBase.dM,theta_l(iQ)) * omega_l(iQ)) ...
        + (diff(mdlBase.dM,omega_l(iQ)) * domega_l(iQ));
    mdlBase.ddC = mdlBase.ddC ...
        + (diff(mdlBase.dC,theta_l(iQ)) * omega_l(iQ)) ...
        + (diff(mdlBase.dC,omega_l(iQ)) * domega_l(iQ)) ...
        + (diff(mdlBase.dC,domega_l(iQ)) * ddomega_l(iQ));
    mdlBase.ddgVec = mdlBase.ddgVec ...
        + (diff(mdlBase.dgVec,theta_l(iQ)) * omega_l(iQ)) ...
        + (diff(mdlBase.dgVec,omega_l(iQ)) * domega_l(iQ));

end

%% Calculate terms for Robot Model (and Controllers)


syms tauF [3,1] 'real'; % Friction torque
syms tauE [3,1] 'real'; % Joint torque


% Compute inverse inertia matrix for complete and reduced model
invB = sym(zeros(3,3));
invB(mdlBase.B ~= 0) = 1 ./ mdlBase.B(mdlBase.B ~= 0);
invM = inv(mdlBase.M);
invM_com = [inv(mdlBase.M - mdlBase.S*invB*mdlBase.S') zeros(3) ; zeros(3) inv(mdlBase.B - mdlBase.S'*invM*mdlBase.S)] ...
    * [eye(3) -mdlBase.S*invB; -mdlBase.S'*invM eye(3)];
invM_red = [invM zeros(3); zeros(3) invB];


% Direct Dynamics with motor friction
domegaDD_com = invM_com * ...
    (-mdlBase.C_com*omega - [mdlBase.gVec; tauF + tauE] + [tauE; u]);
domegaDD_red = invM_red * ...
    (-mdlBase.C_red*omega - [mdlBase.gVec; tauF + tauE] + [tauE; u]);

% Inverse Dynamics with motor friction
tauMotor_com = mdlBase.M_com(4:6,:)*domega + mdlBase.C_com(4:6,:)*omega + tauE + tauF;
tauMotor_red = mdlBase.M_red(4:6,:)*domega + mdlBase.C_red(4:6,:)*omega + tauE + tauF;


% Rigid body direct Dynamics with friction
domegaDD_rigid = invM * (-mdlBase.C*omega_l - mdlBase.gVec - tauF + u);

% Rigid body inverse Dynamics with friction
tauMotor_rigid = mdlBase.M*domega_l + mdlBase.C*omega_l + mdlBase.gVec + tauF;


% Controller
if GENERATE_CONTROLLER

    % Gravity compensation
    gravComp = mdlBase.gVec;


    % Controller for reduced model
    % Inverse Motor Dynamics for reduced system distrubance observer
    tauMotor_disObserver_red = mdlBase.B*domega_m + tauE;

    % Desired Motor acceleration
    domegaMotor_red_des = -invB * (mdlBase.M*domega_l + mdlBase.C*omega_l + mdlBase.gVec);
    thetaMotor_red_des = theta_l + inv(K)*(mdlBase.M*domega_l + mdlBase.C*omega_l + mdlBase.gVec);


    % Feedback Linearization control law for reduced system
    syms v [3,1] 'real';
    syms dtauE [3,1] 'real';

    n = mdlBase.C*omega_l + mdlBase.gVec;
    dn = mdlBase.dC * omega_l + mdlBase.C * domega_l + mdlBase.dgVec;
    ddn = mdlBase.ddC * omega_l + mdlBase.C * ddomega_l + 2 * mdlBase.dC * domega_l + mdlBase.ddgVec;

    domegaLoadDD_red = invM * (tauE - n);
    ddomegaLoadDD_red = invM * (dtauE - mdlBase.dM*domega_l - dn);

    alpha = mdlBase.ddM*domega_l + 2*mdlBase.dM*ddomega_l + ddn;
    u_1 = mdlBase.B*inv(K) * (mdlBase.M*v + alpha);
    u_2 = (mdlBase.M + mdlBase.B)*domega_l + mdlBase.C*omega_l;
    
    
    % Controller for rigid model
    % Inverse Motor Dynamics for rigid system distrubance observer
    tauMotor_disObserver_rigid = mdlBase.M*domega_l + mdlBase.C*omega_l + mdlBase.gVec;

    % Feedback Linearization control law for rigid system
    u_rigid = mdlBase.M*v + mdlBase.C*omega_l;

end

%% Save to model struct


mdl = struct;

% Metadata / Model information
mdl.metadata.isGlobal   = false;
mdl.metadata.isDiscrete = false;
mdl.metadata.name       = MDL_NAME;

% System Variables
mdl.vars.theta = theta;
mdl.vars.omega = omega;
mdl.vars.domega = domega;
mdl.vars.ddomega = ddomega;

% Model Parameter
mdl.params.baseParam = paramVecBase;
mdl.params.dynamic_og = paramVec;
mdl.params.k = k;
mdl.params.kinematic = [g; i_g; r_P1P2; r_P2P3; r_P3P4];

% Model Symstem Terms
mdl.sysTerms.M_com = mdlBase.M_com;
mdl.sysTerms.M_red = mdlBase.M_red;
mdl.sysTerms.M = mdlBase.M;
mdl.sysTerms.dM = mdlBase.dM;
mdl.sysTerms.ddM = mdlBase.ddM;
mdl.sysTerms.B = mdlBase.B;
mdl.sysTerms.S = mdlBase.S;
mdl.sysTerms.C_com = mdlBase.C_com;
mdl.sysTerms.C_red = mdlBase.C_red;
mdl.sysTerms.C = mdlBase.C;
mdl.sysTerms.dC = mdlBase.dC;
mdl.sysTerms.ddC = mdlBase.ddC;
mdl.sysTerms.gVec = mdlBase.gVec;
mdl.sysTerms.kVec = mdlBase.kVec;

mdl.sysTerms_og.M_com = M_com;
mdl.sysTerms_og.M_red = M_red;
mdl.sysTerms_og.M = M;
mdl.sysTerms_og.B = B;
mdl.sysTerms_og.S = S;
mdl.sysTerms_og.C_com = C_com;
mdl.sysTerms_og.C_red = C_red;
mdl.sysTerms_og.C = C;
mdl.sysTerms_og.gVec = gVec;
mdl.sysTerms_og.kVec = kVec;

% Regression Matrix
mdl.regMat.Ybase_com = regMatBase.Robot_com;
mdl.regMat.Ybase_red = regMatBase.Robot_red;
mdl.regMat.Y_com = regMat.Robot_com;
mdl.regMat.Y_red = regMat.Robot_red;

% EOM
mdl.eom.domegaDD_com = domegaDD_com;
mdl.eom.domegaDD_red = domegaDD_red;
mdl.eom.domegaDD_rigid = domegaDD_rigid;
mdl.eom.tauMotor_com = tauMotor_com;
mdl.eom.tauMotor_red = tauMotor_red;
mdl.eom.tauMotor_rigid = tauMotor_rigid;

if GENERATE_CONTROLLER
    mdl.controller.tauMotor_disObserver_red = tauMotor_disObserver_red;
    mdl.controller.tauMotor_disObserver_rigid = tauMotor_disObserver_rigid;
    mdl.controller.domegaLoadDD_red = domegaLoadDD_red;
    mdl.controller.ddomegaLoadDD_red = ddomegaLoadDD_red;
    mdl.controller.u_1 = u_1;
    mdl.controller.u_2 = u_2;
    mdl.controller.u_rigid = u_rigid;
    mdl.controller.u_g = gravComp;
end

%% Save model to file


save([MDL_PATH, MDL_NAME], '-struct', 'mdl');

%% Create MATLAB Functions


% Create Matlab Function
if GENERATE_FUNCTIONS

    disp('Saving Simulink Blocks...');  %#ok<UNRCH> 


    % Flexible Joint Model
    % Direct Dynamics
    vars_sym = { ...
        g, i_g, r_P1P2, r_P2P3, r_P3P4, ...
        baseParam, ...
        ...
        theta, omega, ...
        tauE, tauF, u ...
        };
    % Complete Direct Dynamics
    MDL_NAME = 'directDynamics_complete_fun';
    matlabFunction(mdl.eom.domegaDD_com, ...
        'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'domega'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: ', MDL_NAME], ...
        ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, tauE, tauF, u', '', ''
        });
    % Reduced Direct Dynamics
    MDL_NAME = 'directDynamics_reduced_fun';
    matlabFunction(mdl.eom.domegaDD_red, ...
        'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'domega'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: ', MDL_NAME], ...
        ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, tauE, tauF, u', '', ''
        });

    % Inverse Dynamics
    vars_sym = { ...
        g, i_g, r_P1P2, r_P2P3, r_P3P4, ...
        baseParam, ...
        ...
        theta, omega, domega ...
        tauE, tauF, u  ...
        };
    % Complete Inverse Dynamics
    MDL_NAME = 'inverseDynamics_complete_fun';
    matlabFunction(mdl.eom.tauMotor_com, ...
        'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'tauMotor'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: ', MDL_NAME], ...
        ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, tauE, tauF, u', '', ''
        });
    % Reduced Invrse Dynamics
    MDL_NAME = 'inverseDynamics_reduced_fun';
    matlabFunction(mdl.eom.tauMotor_red, ...
        'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'tauMotor'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: ', MDL_NAME], ...
        ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, tauE, tauF, u', '', ''
        });


    % Rigid Body Model
    % Rigid Body Direct Dynamics
    vars_sym = { ...
        g, r_P1P2, r_P2P3, r_P3P4, ...
        baseParam, ...
        ...
        theta_l, omega_l, ...
        tauF, u ...
        };
    MDL_NAME = 'directDynamics_rigid_fun';
    matlabFunction(mdl.eom.domegaDD_rigid, ...
        'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'domega'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: ', MDL_NAME], ...
        ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, tauF, u', '', ''
        });

    % Rigid Body Inverse Dynamics
    vars_sym = { ...
        g, r_P1P2, r_P2P3, r_P3P4, ...
        baseParam, ...
        ...
        theta_l, omega_l, domega_l, ...
        tauF ...
        };
    MDL_NAME = 'inverseDynamics_rigid_fun';
    matlabFunction(mdl.eom.tauMotor_rigid, ...
        'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
        'Vars', vars_sym, ...
        'Outputs', {'tauMotor'}, ...
        'Optimize', true, ...
        'Comments', {
        [' Code generated by file ''', mfilename, '''.'], ...
        [' Model: ', MDL_NAME], ...
        ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, domega, tauF', '', ''
        });


    % Functions for Controller
    if GENERATE_CONTROLLER

        % Reduced Inverse Motor Dynamics of the Disturbance Observer
        vars_sym = { ...
            i_g, ...
            baseParam, ...
            ...
            domega_m, ...
            tauE ...
            };
        MDL_NAME = 'inverseDynamicsMotor_disObserver_reduced_fun';
        matlabFunction(mdl.controller.tauMotor_disObserver_red, ...
            'File', [MDL_PATH 'functions/control/include/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'tauMotor'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: i_g, baseParam, domegaMotor, tauE', '', ''
            });


        % Desired motor acceleration for reduced model
        MDL_NAME = 'domegaMotorDes_reduced_fun';
        vars_sym = { ...
            g, i_g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l, omega_l, domega_l ...
            };
        matlabFunction(domegaMotor_red_des, ...
            'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'domegaMotor_des'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad_des, omegaLoad_des, domegaLoad_des', '', ''
            });

        % Desired motor acceleration for reduced model
        MDL_NAME = 'thetaMotorDes_reduced_fun';
        vars_sym = { ...
            g, i_g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, k, ...
            ...
            theta_l, omega_l, domega_l ...
            };
        matlabFunction(thetaMotor_red_des, ...
            'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'thetaMotor_des'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, k, thetaLoad_des, omegaLoad_des, domegaLoad_des', '', ''
            });

        % Reduced Direct Dynamics of Load
        MDL_NAME = 'directDynamicsLoad_reduced_fun';
        vars_sym = { ...
            g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l, omega_l, ...
            tauE
            };
        matlabFunction(mdl.controller.domegaLoadDD_red, ...
            'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'domegaLoad'}, ...
            'Optimize', false, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad, omegaLoad, tauE', '', ''
            });


        % Reduced Derivative of Direct Dynamics
        MDL_NAME = 'ddirectDynamicsLoad_reduced_fun';
        vars_sym = { ...
            g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l, omega_l, domega_l, ...
            dtauE ...
            };
        matlabFunction(mdl.controller.ddomegaLoadDD_red, ...
            'File', [MDL_PATH 'functions/robot/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'ddomegaLoad'}, ...
            'Optimize', false, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad, omegaLoad, domegaLoad, dtauE', '', ''
            });


        % Feedback Linearization
        MDL_NAME = 'feedbackLinearization1_fun';
        vars_sym = { ...
            g, i_g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, k, ...
            ...
            theta_l, omega_l, domega_l, ddomega_l, ...
            v ...
            };
        matlabFunction(u_1, ...
            'File', [MDL_PATH 'functions/control/include/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'u'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, k, thetaLoad, omegaLoad, domegaLoad, ddomegaLoad, v', '', ''
            });

        MDL_NAME = 'feedbackLinearization2_fun';
        vars_sym = { ...
            g, i_g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l, omega_l, domega_l ...
            };
        matlabFunction(u_2, ...
            'File', [MDL_PATH 'functions/control/include/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'u'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, i_g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad, omegaLoad, domegaLoad', '', ''
            });


        % Gravity Compensation
        MDL_NAME = 'gravityCompensation_fun';
        vars_sym = { ...
            g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l ...
            };
        matlabFunction(gravComp, ...
            'File', [MDL_PATH 'functions/control/include/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'u'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad', '', ''
            });


        % Feedback Linearization Rigid Model
        MDL_NAME = 'feedbackLinearization_rigid_fun';
        vars_sym = { ...
            g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l, omega_l, ...
            v ...
            };
        matlabFunction(u_rigid, ...
            'File', [MDL_PATH 'functions/control/include/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'u'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, v', '', ''
            });

    end

    % Rigid Inverse Dynamics of the Disturbance Observer
        vars_sym = { ...
            g, r_P1P2, r_P2P3, r_P3P4, ...
            baseParam, ...
            ...
            theta_l, omega_l, domega_l ...
            };
        MDL_NAME = 'inverseDynamicsMotor_disObserver_rigid_fun';
        matlabFunction(mdl.controller.tauMotor_disObserver_rigid, ...
            'File', [MDL_PATH 'functions/control/include/' MDL_NAME], ...
            'Vars', vars_sym, ...
            'Outputs', {'tauMotor'}, ...
            'Optimize', true, ...
            'Comments', {
            [' Code generated by file ''', mfilename, '''.'], ...
            [' Model: ', MDL_NAME], ...
            ' Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, theta, omega, domega', '', ''
            });

end