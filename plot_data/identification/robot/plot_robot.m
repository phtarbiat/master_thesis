%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'identification_trajectories/traj/robot';
MODEL_FOLDER = 'identification/models/robot';
OUTPUT_FOLDER = 'plot_data/identification/robot';
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
trajStruct = load([TRAJ_FOLDER '/traj_robot_structure1.mat']);

timeVec_traj = trajStruct.time;
theta_traj = trajStruct.theta;
omega_traj = trajStruct.omega;
domega_traj = trajStruct.domega;


%% Load identification data

mdlStruct = load([MODEL_FOLDER '/model_robot.mat']);

timeVec = mdlStruct.data.time;

thetaMotor = mdlStruct.data.thetaMotor;
thetaLoad = mdlStruct.data.thetaLoad;

omegaMotor = mdlStruct.data.omegaMotor;
omegaLoad = mdlStruct.data.omegaLoad;

domegaMotor = mdlStruct.data.domegaMotor;
domegaLoad = mdlStruct.data.domegaLoad;

tauMotor = mdlStruct.data.tauMotor;
tauMotor_mdl = mdlStruct.data.tauMotor_mdl;
tauMotor_mdl_RR = mdlStruct.data.tauMotor_mdl_RR;
tauMotor_mdl_CAD = mdlStruct.data.tauMotor_mdl_CAD;
tauMotor_mdl_tauE = mdlStruct.data.tauMotor_mdl_tauE;

tauF_mdl = mdlStruct.data.tauF_mdl;

tauE_mdl = mdlStruct.data.tauE_mdl;
domega_mdl_tauE = mdlStruct.data.domega_mdl_tauE;

%% Plot Data
% Trajectory
f1 = figure;

subplot(3,1,1)

plot(timeVec_traj,theta_traj)

xlim([timeVec_traj(1) timeVec_traj(end)])
ylim([-100 100])
ylabel('$\theta_{t}$ / $^{\circ}$')
legend('$\theta_{t,1}$','$\theta_{t,2}$','$\theta_{t,3}$','Location','eastoutside')
yticks([-90 -45 0 45 90])
grid on
box on

subplot(3,1,2)

plot(timeVec_traj,omega_traj)

xlim([timeVec_traj(1) timeVec_traj(end)])
ylim([-100 100])
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
legend('$\dot{\theta}_{t,1}$','$\dot{\theta}_{t,2}$','$\dot{\theta}_{t,3}$','Location','eastoutside')
yticks([-90 -45 0 45 90])
grid on
box on

subplot(3,1,3)

plot(timeVec_traj,domega_traj)

xlim([timeVec_traj(1) timeVec_traj(end)])
ylim([-220 220])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}_{t}$ / $^{\circ}$/s$^{2}$')
legend('$\ddot{\theta}_{t,1}$','$\ddot{\theta}_{t,2}$','$\ddot{\theta}_{t,3}$','Location','eastoutside')
yticks([-180 -90 0 90 180])
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_traj.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% theta
f2 = figure;

hold on

plot(timeVec_traj,theta_traj)

plot(timeVec,thetaMotor,'--')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-100 100])
ylabel('$\theta$ / $^{\circ}$')
legend('$\theta_{t,1}$','$\theta_{t,2}$','$\theta_{t,3}$','$\theta_{1}$','$\theta_{2}$','$\theta_{3}$','Location','eastoutside')
yticks([-90 -45 0 45 90])
grid on
box on

f2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_theta.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


% omega
f3 = figure;

hold on

plot(timeVec_traj,omega_traj)

plot(timeVec,omegaMotor,'--')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-100 100])
ylabel('$\dot{\theta}$ / $^{\circ}$/s')
legend('$\dot{\theta}_{t,1}$','$\dot{\theta}_{t,2}$','$\dot{\theta}_{t,3}$','$\dot{\theta}_{1}$','$\dot{\theta}_{2}$','$\dot{\theta}_{3}$','Location','eastoutside')
yticks([-90 -45 0 45 90])
grid on
box on

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_omega.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% domega
f4 = figure;

hold on

plot(timeVec_traj,domega_traj)

plot(timeVec,domegaMotor,'--')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-425 425])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('$\ddot{\theta}_{t,1}$','$\ddot{\theta}_{t,2}$','$\ddot{\theta}_{t,3}$','$\ddot{\theta}_{1}$','$\ddot{\theta}_{2}$','$\ddot{\theta}_{3}$','Location','eastoutside')
yticks([-360 -270 -180 -90 0 90 180 270 360])
grid on
box on

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_domega.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


% tauM and tauF
f5 = figure;

hold on

plot(timeVec,tauMotor)
plot(timeVec,tauF_mdl,'--')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-45 45])
xlabel('$t$ / s')
ylabel('$\tau$ / Nm')
legend('$\tau_{m,1}$','$\tau_{m,2}$','$\tau_{m,3}$','$\tau_{R,1}$','$\tau_{R,2}$','$\tau_{R,3}$')
grid on
box on

f5.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_tauMotor.pdf');
    exportgraphics(f5,fileName,'Append',true);
end


% Axis 1 tauM measurment and model
f6 = figure;

hold on

plot(timeVec,tauMotor(:,1),'b-')
plot(timeVec,tauMotor_mdl_RR(:,1)+tauF_mdl(:,1),'r-')
plot(timeVec(1:2:end),tauMotor_mdl(1:2:end,1)+tauF_mdl(1:2:end,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-45 45])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','RR Modell','FJR Modell','Location','northeast')
grid on
box on

f6.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_tauMotor_identification_axis1.pdf');
    exportgraphics(f6,fileName,'Append',true);
end


% Axis 2 tauM measurment and model
f7 = figure;

hold on

plot(timeVec,tauMotor(:,2),'b-')
plot(timeVec,tauMotor_mdl_RR(:,2)+tauF_mdl(:,2),'r-')
plot(timeVec(1:2:end),tauMotor_mdl(1:2:end,2)+tauF_mdl(1:2:end,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-40 40])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','RR Modell','FJR Modell','Location','northeast')
grid on
box on

f7.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_tauMotor_identification_axis2.pdf');
    exportgraphics(f7,fileName,'Append',true);
end


% Axis 3 tauM measurment and model
f8 = figure;

hold on

plot(timeVec,tauMotor(:,3),'b-')
plot(timeVec,tauMotor_mdl_RR(:,3)+tauF_mdl(:,3),'r-')
plot(timeVec(1:2:end),tauMotor_mdl(1:2:end,3)+tauF_mdl(1:2:end,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-7 7])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
yticks([-6 -3 0 3 6])
legend('Messdaten','RR Modell','FJR Modell','Location','northeast')
grid on
box on

f8.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_tauMotor_identification_axis3.pdf');
    exportgraphics(f8,fileName,'Append',true);
end


% Axis 1 tauM measurment and tauE model
f9 = figure;

hold on

plot(timeVec,tauMotor(:,1),'b-')
plot(timeVec,tauMotor_mdl_RR(:,1)+tauF_mdl(:,1)+tauE_mdl(:,1),'r-')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-45 45])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','FJR Modell','Location','northeast')
grid on
box on

f9.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_tauMotorFJR_with_tauE_identification_axis1.pdf');
    exportgraphics(f9,fileName,'Append',true);
end


% Axis 1 domega
f10 = figure;

hold on

plot(timeVec,domegaMotor(:,1),'b-')
plot(timeVec,domega_mdl_tauE(:,1),'r-')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-2000 6000])
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messung','FJR Modell','Location','northeast')
yticks([-2000 0 2000 4000 6000])
grid on
box on

f10.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_domegaFJR_with_tauE_identification_axis1.pdf');
    exportgraphics(f10,fileName,'Append',true);
end


% Axis 2 domega
f11 = figure;

hold on

plot(timeVec,domegaMotor(:,2),'b-')
plot(timeVec,domega_mdl_tauE(:,2),'r-')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-600 600])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messung','FJR Modell','Location','northeast')
yticks([-500 -250 0 250 500])
grid on
box on

f11.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_domegaFJR_with_tauE_identification_axis2.pdf');
    exportgraphics(f11,fileName,'Append',true);
end
%{
% Axis 1 and 2 domega
f10 = figure;

subplot(2,1,1)

hold on

plot(timeVec,domegaMotor(:,1),'b-')
plot(timeVec,domega_mdl_tauE(:,1),'r-')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-2000 6000])
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messung','FJR Modell','Location','northeast')
yticks([-2000 0 2000 4000 6000])
grid on
box on

subplot(2,1,2)

hold on

plot(timeVec,domegaMotor(:,2),'b-')
plot(timeVec,domega_mdl_tauE(:,2),'r-')

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-600 600])
xlabel('$t$ / s')
ylabel('$\ddot{\theta}$ / $^{\circ}$/s$^{2}$')
legend('Messung','FJR Modell','Location','northeast')
yticks([-500 -250 0 250 500])
grid on
box on

f10.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_domegaFJR_with_tauE_identification.pdf');
    exportgraphics(f10,fileName,'Append',true);
end


% Axis 1 domega measurment and tauE model
f10 = figure;

hold on

plot(timeVec,domegaMotor(:,1),'b-')
plot(timeVec,domegaLoad(:,1),'-')
plot(timeVec,domega_mdl_tauE(:,4),'--')
plot(timeVec,domega_mdl_tauE(:,1),'--')

hold off

xlim([timeVec(1) timeVec(end)])
%ylim([-45 45])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('Messdaten','RR Modell','FJR Modell','Location','northeast')
grid on
box on

f10.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/robot_tauMotor_identification_axis1.pdf');
    exportgraphics(f10,fileName,'Append',true);
end
%}