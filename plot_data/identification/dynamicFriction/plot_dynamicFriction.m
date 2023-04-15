%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'trajectory/traj/dynamicFriction';
MODEL_FOLDER = 'identification/models/dynamicFriction';
OUTPUT_FOLDER = 'plot_data/identification/dynamicFriction';
SAMPLE_FREQ = 500;

PLOT_WIDTH = 730;
PLOT_HEIGHT = 275;

PLOT_COULOURS = {[0 0 1] [0 1 0] [1 0 0] [0.8500 0.3250 0.0980] [1 0 1] [0 0 0] [1 1 0]};
plotMeasurments = [1 2 5 10 20 60];
plotOmegas = [1 2 4 8 12 16 20];

i_g = [160; 160; 100];


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


%% Load trajectory data
trajStruct_axis1 = load([TRAJ_FOLDER '/traj_dynamicFriction_axis1.mat']);

timeVec_traj_axis1 = trajStruct_axis1.time;
theta_traj_axis1 = trajStruct_axis1.theta(:,1);
omega_traj_axis1 = trajStruct_axis1.omega(:,1);


%% Load identification data
% Axis 1
lugre_mdlStruct_axis1 = load([MODEL_FOLDER '/model_lugreFriction_axis1.mat']);
gms_mdlStruct_axis1 = load([MODEL_FOLDER '/model_gmsFriction_axis1.mat']);

timeVec_axis1 = lugre_mdlStruct_axis1.data.timeVec;
omegaMotor_axis1 = lugre_mdlStruct_axis1.data.omegaMotor;
tauMotor_axis1 = lugre_mdlStruct_axis1.data.tauMotor .* i_g(1);

static_tauF_mdl_axis1 = lugre_mdlStruct_axis1.data.tauF_mdlStatic .* i_g(1);
lugre_tauF_mdl_axis1 = lugre_mdlStruct_axis1.data.tauF_mdl .* i_g(1);
gms_tauF_mdl_axis1 = gms_mdlStruct_axis1.data.tauF_mdl .* i_g(1);

% Axis 2
lugre_mdlStruct_axis2 = load([MODEL_FOLDER '/model_lugreFriction_axis2.mat']);
gms_mdlStruct_axis2 = load([MODEL_FOLDER '/model_gmsFriction_axis2.mat']);

timeVec_axis2 = lugre_mdlStruct_axis2.data.timeVec;
tauMotor_axis2 = lugre_mdlStruct_axis2.data.tauMotor .* i_g(2);

static_tauF_mdl_axis2 = lugre_mdlStruct_axis2.data.tauF_mdlStatic .* i_g(2);
lugre_tauF_mdl_axis2 = lugre_mdlStruct_axis2.data.tauF_mdl .* i_g(2);
gms_tauF_mdl_axis2 = gms_mdlStruct_axis2.data.tauF_mdl .* i_g(2);

% Axis 3
lugre_mdlStruct_axis3 = load([MODEL_FOLDER '/model_lugreFriction_axis3.mat']);
gms_mdlStruct_axis3 = load([MODEL_FOLDER '/model_gmsFriction_axis3.mat']);

timeVec_axis3 = lugre_mdlStruct_axis3.data.timeVec;
tauMotor_axis3 = lugre_mdlStruct_axis3.data.tauMotor .* i_g(3);

static_tauF_mdl_axis3 = lugre_mdlStruct_axis3.data.tauF_mdlStatic .* i_g(3);
lugre_tauF_mdl_axis3 = lugre_mdlStruct_axis3.data.tauF_mdl .* i_g(3);
gms_tauF_mdl_axis3 = gms_mdlStruct_axis3.data.tauF_mdl .* i_g(3);


%% Plot Data
% Axis 1 trajectory
f1 = figure;

subplot(2,2,1)
hold on

plot(timeVec_traj_axis1,theta_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-180 180])
ylabel('$\theta_{t}$ / $^{\circ}$')
yticks([-180 -90 0 90 180])
grid on
box on

subplot(2,2,2)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-1.2 1.2])
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
yticks([-1.5 -1 -0.5 0 0.5 1 1.5])
grid on
box on

subplot(2,2,3)
hold on

plot(timeVec_traj_axis1,theta_traj_axis1)

hold off

xlim([10.25 10.25+2*5])
ylim([-153.5 -150])
xlabel('$t$ / s')
ylabel('$\theta_{t}$ / $^{\circ}$')
grid on
box on

subplot(2,2,4)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)

hold off

xlim([10.25 10.25+2*5])
ylim([-0.7 1.2])
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
%legend('$\dot{\theta}_{1}$')
yticks([-1.5 -1 -0.5 0 0.5 1 1.5])
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/dynamicFriction_traj_axis1.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 friction torque without model
f2 = figure;

hold on

plot(timeVec_axis1,tauMotor_axis1,'b-')

ylim([-9 9])
ylabel('$\tau_{R}$ / Nm')

yyaxis right

plot(timeVec_axis1,omegaMotor_axis1,'r-')

ylim([-1.2 1.2])
ylabel('$\dot{\theta}$ / $^{\circ}$/s')

hold off

xlim([55.30 59.30])
xlabel('$t$ / s')
grid on
box on
legend('$\tau_{R}$','$\dot{\theta}$','Location','southeast')

f2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/dynamicFriction_noMdl_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


% Axis 1 friction torque
f3 = figure;

hold on

plot(timeVec_axis1,tauMotor_axis1,'b-')

plot(timeVec_axis1,static_tauF_mdl_axis1,'--','Color',[0.6350 0.0780 0.1840])
plot(timeVec_axis1,lugre_tauF_mdl_axis1,'-','Color',[0.9290 0.6940 0.1250])
plot(timeVec_axis1,gms_tauF_mdl_axis1,'r-')

hold off

xlim([55.30 59.30])
ylim([-9 9])
xlabel('$t$ / s')
ylabel('$\tau_{R}$ / Nm')
grid on
box on
legend('Messdaten','statisches Modell','LuGre-Modell','GMS-Modell','Location','southeast')

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/dynamicFriction_axis1.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% Axis 2 friction torque
f4 = figure;

hold on

plot(timeVec_axis2,tauMotor_axis2,'b-')

plot(timeVec_axis2,static_tauF_mdl_axis2,'--','Color',[0.6350 0.0780 0.1840])
plot(timeVec_axis2,lugre_tauF_mdl_axis2,'-','Color',[0.9290 0.6940 0.1250])
plot(timeVec_axis2,gms_tauF_mdl_axis2,'r-')

hold off

xlim([107.30 111.30])
ylim([-4 5])
xlabel('$t$ / s')
ylabel('$\tau_{R}$ / Nm')
grid on
box on
legend('Messdaten','statisches Modell','LuGre-Modell','GMS-Modell','Location','southeast')

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/dynamicFriction_axis2.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


% Axis 3 friction torque
f5 = figure;

hold on

plot(timeVec_axis3,tauMotor_axis3,'b-')

plot(timeVec_axis3,static_tauF_mdl_axis3,'--','Color',[0.6350 0.0780 0.1840])
plot(timeVec_axis3,lugre_tauF_mdl_axis3,'-','Color',[0.9290 0.6940 0.1250])
plot(timeVec_axis3,gms_tauF_mdl_axis3,'r-')

hold off

xlim([31.30 35.30])
ylim([-1.2 1.4])
xlabel('$t$ / s')
ylabel('$\tau_{R}$ / Nm')
grid on
box on
legend('Messdaten','statisches Modell','LuGre-Modell','GMS-Modell','Location','southeast')

f5.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/dynamicFriction_axis3.pdf');
    exportgraphics(f5,fileName,'Append',true);
end