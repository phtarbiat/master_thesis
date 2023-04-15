%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'identification_trajectories/traj/stiffness';
MODEL_FOLDER = 'identification/models/stiffness';
OUTPUT_FOLDER = 'plot_data/identification/stiffness';

PLOT_WIDTH = 730;
PLOT_HEIGHT = 275;


% Create Output folder
if ~exist(OUTPUT_FOLDER,'dir') && SAVE_PLOTS
    mkdir(OUTPUT_FOLDER);
end

% Change to Latex interpreter
listFactory = fieldnames(get(groot,'factory'));
idxInterpreter = find(contains(listFactory,'Interpreter'));
for k = 1:length(idxInterpreter)
    defaultName = strrep(listFactory{idxInterpreter(k)},'factory','default');
    set(groot, defaultName,'latex');
end


% Part parameters
T = [ ...
    14 48 100; ...
    7 25 100; ...
    2 6.9 100; ...
    ];

K = [ ...
    31000 50000 57000; ...
    16000 25000 29000; ...
    4700 6100 7100 ...
    ];

K_deg = (pi/180) .* K; 
Tau = T;

ThetaDiff = zeros(3,3);
for iJoint = 1:3
    ThetaDiff(iJoint,1) = Tau(iJoint,1) / K_deg(iJoint,1);
    ThetaDiff(iJoint,2) = ThetaDiff(iJoint,1) + ((Tau(iJoint,2)-Tau(iJoint,1)) / K_deg(iJoint,2));
    ThetaDiff(iJoint,3) = ThetaDiff(iJoint,2) + ((Tau(iJoint,3)-Tau(iJoint,2)) / K_deg(iJoint,3));
end


%% Load trajectory data
trajStruct_axis1 = load([TRAJ_FOLDER '/traj_stiffness_axis1.mat']);

timeVec_traj_axis1 = trajStruct_axis1.time;
theta_traj_axis1 = trajStruct_axis1.theta(:,1);
omega_traj_axis1 = trajStruct_axis1.omega(:,1);

%% Load identification data
thetaDiffVec = (0:0.001:1)';

% Axis 1
mdlStruct_axis1 = load([MODEL_FOLDER '/model_stiffness_axis1.mat']);

tauMotor_axis1 = mdlStruct_axis1.data.tauMotor;
thetaDiff_axis1 = mdlStruct_axis1.data.thetaDiff;
tauMotor_raw_axis1 = mdlStruct_axis1.data.tauMotor_raw;
thetaDiff_raw_axis1 = mdlStruct_axis1.data.thetaDiff_raw;
tauMotor_hysteresis_axis1 = mdlStruct_axis1.data.tauMotor_hysteresis;
thetaDiff_hysteresis_axis1 = mdlStruct_axis1.data.thetaDiff_hysteresis;

c_k_axis1 = mdlStruct_axis1.param.c_k;
tauE_axis1 = sign(thetaDiffVec) .* c_k_axis1(1) .* thetaDiffVec.^c_k_axis1(2);

maxThetaDiff_axis1 = max(abs(thetaDiff_axis1));
tauE_axis1_end = sign(maxThetaDiff_axis1) .* c_k_axis1(1) .* maxThetaDiff_axis1.^c_k_axis1(2);
thetaDiffVec_axis1 = 0:0.001:maxThetaDiff_axis1;
tauEVec_axis1 = sign(thetaDiffVec_axis1) .* c_k_axis1(1) .* thetaDiffVec_axis1.^c_k_axis1(2);
linTauMotor_axis1 = [ ...
    tauEVec_axis1 ...
    tauE_axis1_end ...
    Tau(1,2) ...
    Tau(1,3) ...
    ];
linThetaDiff_axis1 = [ ...
    thetaDiffVec_axis1 ...
    maxThetaDiff_axis1 ...
    maxThetaDiff_axis1 + (Tau(1,2)-tauE_axis1_end)/K_deg(1,2) ...
    0 ...
    ];
linThetaDiff_axis1(end) = linThetaDiff_axis1(end-1) + (Tau(1,3)-Tau(1,2))/K_deg(1,3);

% Axis 2
mdlStruct_axis2 = load([MODEL_FOLDER '/model_stiffness_axis2.mat']);

tauMotor_axis2 = mdlStruct_axis2.data.tauMotor;
thetaDiff_axis2 = mdlStruct_axis2.data.thetaDiff;

c_k_axis2 = mdlStruct_axis2.param.c_k;
tauE_axis2 = sign(thetaDiffVec) .* c_k_axis2(1) .* thetaDiffVec.^c_k_axis2(2);

maxThetaDiff_axis2 = max(abs(thetaDiff_axis2));
tauE_axis2_end = sign(maxThetaDiff_axis2) .* c_k_axis2(1) .* maxThetaDiff_axis2.^c_k_axis2(2);
thetaDiffVec_axis2 = 0:0.001:maxThetaDiff_axis2;
tauEVec_axis2 = sign(thetaDiffVec_axis2) .* c_k_axis2(1) .* thetaDiffVec_axis2.^c_k_axis2(2);
linTauMotor_axis2 = [ ...
    tauEVec_axis2 ...
    tauE_axis2_end ...
    Tau(2,3) ...
    ];
linThetaDiff_axis2 = [ ...
    thetaDiffVec_axis2 ...
    maxThetaDiff_axis2 ...
    maxThetaDiff_axis2 + (Tau(2,3)-tauE_axis2_end)/K_deg(2,3) ...
    ];

% Axis 3
mdlStruct_axis3 = load([MODEL_FOLDER '/model_stiffness_axis3.mat']);

tauMotor_axis3 = mdlStruct_axis3.data.tauMotor;
thetaDiff_axis3 = mdlStruct_axis3.data.thetaDiff;

c_k_axis3 = mdlStruct_axis3.param.c_k;
tauE_axis3 = sign(thetaDiffVec) .* c_k_axis3(1) .* thetaDiffVec.^c_k_axis3(2);

maxThetaDiff_axis3 = max(abs(thetaDiff_axis3));
tauE_axis3_end = sign(maxThetaDiff_axis3) .* c_k_axis3(1) .* maxThetaDiff_axis3.^c_k_axis3(2);
thetaDiffVec_axis3 = 0:0.001:maxThetaDiff_axis3;
tauEVec_axis3 = sign(thetaDiffVec_axis3) .* c_k_axis3(1) .* thetaDiffVec_axis3.^c_k_axis3(2);
linTauMotor_axis3 = [ ...
    tauEVec_axis3 ...
    tauE_axis3_end ...
    Tau(3,3) ...
    ];
linThetaDiff_axis3 = [ ...
    thetaDiffVec_axis3 ...
    maxThetaDiff_axis3 ...
    maxThetaDiff_axis3 + (Tau(3,3)-tauE_axis3_end)/K_deg(3,3) ...
    ];

%}
%% Plot Data
% Axis 1 trajectory
f1 = figure;

subplot(2,2,1)
hold on

plot(timeVec_traj_axis1,theta_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-10 165])
ylabel('$\theta_{t}$ / $^{\circ}$')
%legend('$\theta_{1}$')
yticks([-90 -45 0 45 90 135 180])
grid on
box on

subplot(2,2,2)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-1.3 1.3])
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
yticks([-1.5 -1 -0.5 0 0.5 1 1.5])
grid on
box on

subplot(2,2,3)
hold on

plot(timeVec_traj_axis1,theta_traj_axis1)

hold off

xlim([9.8 14.4])
ylim([1.85 3.3])
xlabel('$t$ / s')
ylabel('$\theta$ / $^{\circ}$')
%legend('$\theta_{1}$')
%yticks([-90 -45 0 45 90 135 180])
grid on
box on

subplot(2,2,4)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)

hold off

xlim([9.8 14.4])
ylim([-0.2 1.3])
xlabel('$t$ / s')
ylabel('$\dot{\theta}$ / $^{\circ}$/s')
%legend('$\dot{\theta}_{1}$')
yticks([-1.5 -1 -0.5 0 0.5 1 1.5])
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_traj_axis1.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 hysteresis
f2 = figure;

subplot(2,1,1)
hold on

plot(tauMotor_raw_axis1(:,1),thetaDiff_raw_axis1(:,1),'bx')
plot(tauMotor_raw_axis1(:,2),thetaDiff_raw_axis1(:,2),'rx')

hold off

xlim([-30 30])
ylim([-0.07 0.07])
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('$\dot{\theta} > 0$','$\dot{\theta} < 0$','Location','southeast')
grid on
box on

subplot(2,1,2)
hold on

plot(tauMotor_hysteresis_axis1(:,1),thetaDiff_hysteresis_axis1(:,1),'bx')
plot(tauMotor_hysteresis_axis1(:,2),thetaDiff_hysteresis_axis1(:,2),'rx')

hold off

xlim([-30 30])
ylim([-0.07 0.07])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('$\dot{\theta} > 0$','$\dot{\theta} < 0$','Location','southeast')
grid on
box on

f2.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_hysteresis_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


f3 = figure;

hold on

plot(tauMotor_axis1,thetaDiff_axis1,'bx')

plot(tauE_axis1,thetaDiffVec,'r-','LineWidth',1.5)
plot(-tauE_axis1,-thetaDiffVec,'r-','LineWidth',1.5)

hold off

xlim([-30 30])
ylim([-0.07 0.07])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('Messdaten','Modell','Location','southeast')
grid on
box on

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_axis1.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


f4 = figure;

hold on

plot([0 Tau(1,:)],[0 ThetaDiff(1,:)],'-','Color',[0 0 1])
plot(tauE_axis1,thetaDiffVec,'-','Color',[1 0 0])
plot(linTauMotor_axis1,linThetaDiff_axis1,'--','Color',[0.9290 0.6940 0.1250])

xline(tauE_axis1_end,'k--')

hold off

xlim([0 80])
ylim([0 0.12])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('Datenblatt','Iden. Modell','kombiniertes Modell','Location','southeast')
grid on
box on

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_hd_axis1.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


f5 = figure;

hold on

plot(tauMotor_axis2,thetaDiff_axis2,'bx')

plot(tauE_axis2,thetaDiffVec,'r-','LineWidth',1.5)
plot(-tauE_axis2,-thetaDiffVec,'r-','LineWidth',1.5)

hold off

xlim([-30 30])
ylim([-0.12 0.12])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('Messdaten','Modell','Location','southeast')
grid on
box on

f5.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_axis2.pdf');
    exportgraphics(f5,fileName,'Append',true);
end


f6 = figure;

hold on

plot([0 Tau(2,:)],[0 ThetaDiff(2,:)],'-','Color',[0 0 1])
plot(tauE_axis2,thetaDiffVec,'-','Color',[1 0 0])
plot(linTauMotor_axis2,linThetaDiff_axis2,'--','Color',[0.9290 0.6940 0.1250])

xline(tauE_axis2_end,'k--')

hold off

xlim([0 80])
ylim([0 0.22])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('Datenblatt','Iden. Modell','kombiniertes Modell','Location','southeast')
grid on
box on

f6.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_hd_axis2.pdf');
    exportgraphics(f6,fileName,'Append',true);
end


f7 = figure;

hold on

plot(tauMotor_axis3,thetaDiff_axis3,'bx')

plot(tauE_axis3,thetaDiffVec,'r-','LineWidth',1.5)
plot(-tauE_axis3,-thetaDiffVec,'r-','LineWidth',1.5)

hold off

xlim([-9 9])
ylim([-0.13 0.13])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('Messdaten','Modell','Location','southeast')
grid on
box on

f7.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_axis3.pdf');
    exportgraphics(f7,fileName,'Append',true);
end


f8 = figure;

hold on

plot([0 Tau(3,:)],[0 ThetaDiff(3,:)],'-','Color',[0 0 1])
plot(tauE_axis3,thetaDiffVec,'-','Color',[1 0 0])
plot(linTauMotor_axis3,linThetaDiff_axis3,'--','Color',[0.9290 0.6940 0.1250])

xline(tauE_axis3_end,'k--')

hold off

xlim([0 25])
ylim([0 0.25])
xlabel('$\tau_{e}$ / Nm')
ylabel('$\varphi_{e}$ / $^{\circ}$')
legend('Datenblatt','Iden. Modell','kombiniertes Modell','Location','southeast')
grid on
box on

f8.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/stiffness_hd_axis3.pdf');
    exportgraphics(f8,fileName,'Append',true);
end
%}