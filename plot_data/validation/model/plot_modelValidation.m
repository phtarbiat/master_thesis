%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'identification_trajectories/traj/validation';
INPUT_FOLDER_NUMERIC = 'validate/output';
INPUT_FOLDER_SIMULINK = 'simulation_data/validation_traj';
OUTPUT_FOLDER = 'plot_data/validation/model';
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
trajStruct = load([TRAJ_FOLDER '/traj_validation.mat']);

timeVec_traj = trajStruct.time;
theta_traj = trajStruct.theta;
omega_traj = trajStruct.omega;
domega_traj = trajStruct.domega;


%% Load identification data
% Numeric validation
dataStruct_num = load([INPUT_FOLDER_NUMERIC '/data_modelValidation.mat']);

timeVec = dataStruct_num.data.time;

theta = dataStruct_num.data.theta;
omega = dataStruct_num.data.omega;
domega = dataStruct_num.data.domega;
tauMotor = dataStruct_num.data.tauMotor;

tauMotor_mdl = dataStruct_num.data.tauMotor_mdl;
tauMotor_mdl_reg = dataStruct_num.data.tauMotor_mdl_reg;
tauMotor_mdl_cad = dataStruct_num.data.tauMotor_mdl_cad;

domega_mdl = dataStruct_num.data.domega_mdl;
domega_mdl_cad = dataStruct_num.data.domega_mdl_cad;


% Simulink validation
dataStruct_sim = load([INPUT_FOLDER_SIMULINK '/rigid_model.mat']);

timeVec_sim = dataStruct_sim.simulation.timeVec;

theta_sim = rad2deg(dataStruct_sim.simulation.theta(:,:)');
omega_sim = rad2deg(dataStruct_sim.simulation.omega(:,:)');
domega_sim = rad2deg(dataStruct_sim.simulation.domega(:,:)');


dataStruct_sim_cad = load([INPUT_FOLDER_SIMULINK '/rigid_model_cad.mat']);

timeVec_sim_cad = dataStruct_sim_cad.simulation.timeVec;

theta_sim_cad = rad2deg(dataStruct_sim_cad.simulation.theta(:,:)');
omega_sim_cad = rad2deg(dataStruct_sim_cad.simulation.omega(:,:)');
domega_sim_cad = rad2deg(dataStruct_sim_cad.simulation.domega(:,:)');


%% Plot Data
% Trajectory
f1 = figure;

subplot(3,1,1)

plot(timeVec_traj,theta_traj)

xlim([timeVec_traj(1) timeVec_traj(end)])
ylim([-150 100])
ylabel('$\theta_{t}$ / $^{\circ}$')
legend('$\theta_{t,1}$','$\theta_{t,2}$','$\theta_{t,3}$','Location','eastoutside')
yticks([-150 -100 -50 0 50 100])
grid on
box on

subplot(3,1,2)

plot(timeVec_traj,omega_traj)

xlim([timeVec_traj(1) timeVec_traj(end)])
ylim([-70 85])
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
legend('$\dot{\theta}_{t,1}$','$\dot{\theta}_{t,2}$','$\dot{\theta}_{t,3}$','Location','eastoutside')
yticks([-90 -60 -30 0 30 60 90])
grid on
box on

subplot(3,1,3)

plot(timeVec_traj,domega_traj)

xlim([timeVec_traj(1) timeVec_traj(end)])
ylim([-70 70])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}_{t}$ / $^{\circ}$/s$^{2}$')
legend('$\ddot{\theta}_{t,1}$','$\ddot{\theta}_{t,2}$','$\ddot{\theta}_{t,3}$','Location','eastoutside')
yticks([-90 -60 -30 0 30 60 90])
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_traj.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 tauM measurment and model
f2 = figure;

hold on

plot(timeVec,tauMotor(:,1),'b-')
plot(timeVec,tauMotor_mdl(:,1),'r-')
plot(timeVec,tauMotor_mdl_cad(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-20 20])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','Iden. Modell','CAD Modell','Location','northeast')
grid on
box on

f2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_tauMotor_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


% Axis 2 tauM measurment and model
f3 = figure;

hold on

plot(timeVec,tauMotor(:,2),'b-')
plot(timeVec,tauMotor_mdl(:,2),'r-')
plot(timeVec,tauMotor_mdl_cad(:,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-30 30])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','Iden. Modell','CAD Modell','Location','northeast')
grid on
box on

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_tauMotor_axis2.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% Axis 3 tauM measurment and model
f4 = figure;

hold on

plot(timeVec,tauMotor(:,3),'b-')
plot(timeVec,tauMotor_mdl(:,3),'r-')
plot(timeVec,tauMotor_mdl_cad(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-8 8])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','Iden. Modell','CAD Modell','Location','northeast')
grid on
box on

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_tauMotor_axis3.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


% Axis 1 domega measurment and model
f5 = figure;

hold on

plot(timeVec,domega(:,1),'b-')
plot(timeVec,domega_mdl(:,1),'r-')
plot(timeVec,domega_mdl_cad(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-100 65])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messdaten','Iden. Modell','CAD Modell','Location','northeast')
grid on
box on

f5.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_domega_axis1.pdf');
    exportgraphics(f5,fileName,'Append',true);
end


% Axis 2 domega measurment and model
f6 = figure;

hold on

plot(timeVec,domega(:,2),'b-')
plot(timeVec,domega_mdl(:,2),'r-')
plot(timeVec,domega_mdl_cad(:,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-150 200])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messdaten','Iden. Modell','CAD Modell','Location','northeast')
grid on
box on

f6.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_domega_axis2.pdf');
    exportgraphics(f6,fileName,'Append',true);
end


% Axis 3 domega measurment and model
f7 = figure;

hold on

plot(timeVec,domega(:,3),'b-')
plot(timeVec,domega_mdl(:,3),'r-')
plot(timeVec,domega_mdl_cad(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-600 400])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messdaten','Iden. Modell','CAD Modell','Location','northeast')
grid on
box on

f7.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_domega_axis3.pdf');
    exportgraphics(f7,fileName,'Append',true);
end


% Axis 1 Simulink
f8 = figure;

subplot(2,1,1)

hold on

plot(timeVec,theta(:,1),'b-')
plot(timeVec_sim,theta_sim(:,1),'r-')
plot(timeVec_sim_cad,theta_sim_cad(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-150 70])
ylabel('$\theta$ / $^{\circ}$')
legend('Messdaten','Iden. Modell','CAD Modell','Location','southwest')%'eastoutside')
%yticks([-150 -100 -50 0 50 100])
grid on
box on

subplot(2,1,2)

hold on

plot(timeVec,omega(:,1),'b-')
plot(timeVec_sim,omega_sim(:,1),'r-')
plot(timeVec_sim_cad,omega_sim_cad(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-50 70])
ylabel('$\dot{\theta}$ / $^{\circ}$/s')
legend('Messdaten','Iden. Modell','CAD Modell','Location','southwest')%'eastoutside')
%yticks([-90 -60 -30 0 30 60 90])
grid on
box on
%{
subplot(3,1,3)

hold on

plot(timeVec,domega(:,1),'b-')
plot(timeVec_sim,domega_sim(:,1),'r-')
plot(timeVec_sim_cad,domega_sim_cad(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-200 200])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messdaten','Modell Iden','Modell CAD','Location','eastoutside')
%yticks([-90 -60 -30 0 30 60 90])
grid on
box on
%}
f8.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_simulink_axis1.pdf');
    exportgraphics(f8,fileName,'Append',true);
end


% Axis 3 Simulink
f9 = figure;

subplot(2,1,1)

hold on

plot(timeVec,theta(:,3),'b-')
plot(timeVec_sim,theta_sim(:,3),'r-')
plot(timeVec_sim_cad,theta_sim_cad(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-600 200])
ylabel('$\theta$ / $^{\circ}$')
legend('Messdaten','Iden. Modell','CAD Modell','Location','southwest')%'eastoutside')
%yticks([-150 -100 -50 0 50 100])
grid on
box on

subplot(2,1,2)

hold on

plot(timeVec,omega(:,3),'b-')
plot(timeVec_sim,omega_sim(:,3),'r-')
plot(timeVec_sim_cad,omega_sim_cad(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-400 200])
ylabel('$\dot{\theta}$ / $^{\circ}$/s')
legend('Messdaten','Iden. Modell','CAD Modell','Location','southwest')%'eastoutside')
%yticks([-90 -60 -30 0 30 60 90])
grid on
box on
%{
subplot(3,1,3)

hold on

plot(timeVec,domega(:,1),'b-')
plot(timeVec_sim,domega_sim(:,1),'r-')
plot(timeVec_sim_cad,domega_sim_cad(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-200 200])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messdaten','Modell Iden','Modell CAD','Location','eastoutside')
%yticks([-90 -60 -30 0 30 60 90])
grid on
box on
%}
f9.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/validation_simulink_axis3.pdf');
    exportgraphics(f9,fileName,'Append',true);
end
