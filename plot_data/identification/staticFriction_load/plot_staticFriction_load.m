%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'identification_trajectories/traj/staticFriction_load';
MODEL_FOLDER = 'identification/models/staticFriction';
OUTPUT_FOLDER = 'plot_data/identification/staticFriction_load';
SAMPLE_FREQ = 500;

PLOT_WIDTH = 730;
PLOT_HEIGHT = 275;


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
trajStruct_axis1 = load([TRAJ_FOLDER '/traj_staticFriction_load_axis1.mat']);

timeVec_traj_axis1 = trajStruct_axis1.time;
theta_traj_axis1 = trajStruct_axis1.theta(:,1);
omega_traj_axis1 = trajStruct_axis1.omega(:,1);


%% Load identification data
tauLoadVec = (-0.2:0.001:0.2)';

% Axis 1
mdlStruct_axis1 = load([MODEL_FOLDER '/model_staticFriction_load_axis1.mat']);

tauLoad_axis1 = mdlStruct_axis1.data.tauLoad .* i_g(1);
tauF_axis1 = mdlStruct_axis1.data.tauF .* i_g(1);

c_load_axis1 = mdlStruct_axis1.param.c_load;
tauF_mdl_axis1 = c_load_axis1 .* tauLoadVec.^2;
tauF_mdl_axis1 = tauF_mdl_axis1 .* i_g(1);

% Axis 2
mdlStruct_axis2 = load([MODEL_FOLDER '/model_staticFriction_load_axis2.mat']);

tauLoad_axis2 = mdlStruct_axis2.data.tauLoad .* i_g(2);
tauF_axis2 = mdlStruct_axis2.data.tauF .* i_g(2);

c_load_axis2 = mdlStruct_axis2.param.c_load;
tauF_mdl_axis2 = c_load_axis2 .* tauLoadVec.^2;
tauF_mdl_axis2 = tauF_mdl_axis2 .* i_g(2);

% Axis 3
mdlStruct_axis3 = load([MODEL_FOLDER '/model_staticFriction_load_axis3.mat']);

tauLoad_axis3 = mdlStruct_axis3.data.tauLoad .* i_g(3);
tauF_axis3 = mdlStruct_axis3.data.tauF .* i_g(3);

c_load_axis3 = mdlStruct_axis3.param.c_load;
tauF_mdl_axis3 = c_load_axis3 .* tauLoadVec.^2;
tauF_mdl_axis3 = tauF_mdl_axis3 .* i_g(3);


%% Plot Data
% Axis 1 trajectory
f1 = figure;

subplot(2,1,1)
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

subplot(2,1,2)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-1.2 1.2])
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
%legend('$\dot{\theta}_{1}$')
yticks([-1.5 -1 -0.5 0 0.5 1 1.5])
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_load_traj_axis1.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 load dependent friction
f2 = figure;

hold on

plot(tauLoad_axis1,tauF_axis1,'bx','Markersize',1)

plot(tauLoadVec.*i_g(1),tauF_mdl_axis1,'r-','LineWidth',1.5)

hold off

xlim([-30 30])
ylim([-2.5 4])
xlabel('$\tau_{l}$ / Nm')
ylabel('$\tau_{R,l}$ / Nm')
grid on
box on
legend('Messdaten','Modell','Location','southeast')

f2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_load_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


% Axis 2 load dependent friction
f3 = figure;

hold on

plot(tauLoad_axis2,tauF_axis2,'bx','Markersize',1)

plot(tauLoadVec.*i_g(2),tauF_mdl_axis2,'r-','LineWidth',1.5)

hold off

xlim([-30 30])
ylim([-1 5])
xlabel('$\tau_{l}$ / Nm')
ylabel('$\tau_{R,l}$ / Nm')
grid on
box on
legend('Messdaten','Modell','Location','southeast')

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_load_axis2.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% Axis 3 load dependent friction
f4 = figure;

hold on

plot(tauLoad_axis3,tauF_axis3,'bx','Markersize',1)

plot(tauLoadVec.*i_g(3),tauF_mdl_axis3,'r-','LineWidth',1.5)

hold off

xlim([-9 9])
ylim([-0.5 1.1])
xlabel('$\tau_{l}$ / Nm')
ylabel('$\tau_{R,l}$ / Nm')
grid on
box on
legend('Messdaten','Modell','Location','southeast')

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_load_axis3.pdf');
    exportgraphics(f4,fileName,'Append',true);
end