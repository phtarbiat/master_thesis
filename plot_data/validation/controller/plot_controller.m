%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
INPUT_FOLDER = 'validate/output';
OUTPUT_FOLDER = 'plot_data/validation/controller';
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


%% Load identification data
% Numeric validation
dataStruct = load([INPUT_FOLDER '/data_controllerValidation.mat']);

timeVec = dataStruct.data.time;

%% Plot Data

f1 = figure;

%subplot(3,1,1)

hold on

plot(timeVec,dataStruct.data.omega.com(:,3),'b-')
plot(timeVec,dataStruct.data.omega.DisObs(:,3),'r')

hold off

xlim([17.35 18.65])
%ylim([-5.1 7.2])
xlabel('$t$ / s')
ylabel('$\dot{\theta}$ / $^{\circ}$/s')
legend('CT+RK+SB','CT+RK','Location','northeast')
grid on
box on

f1.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_omega.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 position error
f2 = figure;

hold on

plot(timeVec,dataStruct.data.eTheta.com(:,1),'b-')
plot(timeVec,dataStruct.data.eTheta.DisObs(:,1),'r--')
plot(timeVec,dataStruct.data.eTheta.FricComp(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-1.5*1.1 1.5])
xlabel('$t$ / s')
ylabel('$\theta_{d} - \theta$ / $^{\circ}$')
legend('CT+RK+SB','CT+RK','CT','Location','southeast')
grid on
box on

f2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eTheta_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


% Axis 2 position error
f3 = figure;

hold on

plot(timeVec,dataStruct.data.eTheta.com(:,2),'b-')
plot(timeVec,dataStruct.data.eTheta.DisObs(:,2),'r--')
plot(timeVec,dataStruct.data.eTheta.FricComp(:,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-2.4 3.2].*1.1)
xlabel('$t$ / s')
ylabel('$\theta_{d} - \theta$ / $^{\circ}$')
legend('CT+RK+SB','CT+RK','CT','Location','northeast')
grid on
box on

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eTheta_axis2.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% Axis 3 position error
f4 = figure;

hold on

plot(timeVec,dataStruct.data.eTheta.com(:,3),'b-')
plot(timeVec,dataStruct.data.eTheta.DisObs(:,3),'r--')
plot(timeVec,dataStruct.data.eTheta.FricComp(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-7.2 5.5].*1.1)
xlabel('$t$ / s')
ylabel('$\theta_{d} - \theta$ / $^{\circ}$')
legend('CT+RK+SB','CT+RK','CT','Location','southeast')
grid on
box on

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eTheta_axis3.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


% Axis 1 velocity error
f5 = figure;

hold on

plot(timeVec,dataStruct.data.eOmega.com(:,1),'b-')
plot(timeVec,dataStruct.data.eOmega.DisObs(:,1),'r--')
plot(timeVec,dataStruct.data.eOmega.FricComp(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-4.5 4.6].*1.1)
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{d} - \dot{\theta}$ / $^{\circ}$/s')
legend('CT+RK+SB','CT+RK','CT','Location','southeast')
grid on
box on

f5.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eOmega_axis1.pdf');
    exportgraphics(f5,fileName,'Append',true);
end


% Axis 2 velocity error
f6 = figure;

hold on

plot(timeVec,dataStruct.data.eOmega.com(:,2),'b-')
plot(timeVec,dataStruct.data.eOmega.DisObs(:,2),'r--')
plot(timeVec,dataStruct.data.eOmega.FricComp(:,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5.2 5].*1.1)
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{d} - \dot{\theta}$ / $^{\circ}$/s')
yticks([-8 -6 -4 -2 0 2 4 6 8])
legend('CT+RK+SB','CT+RK','CT','Location','southeast')
grid on
box on

f6.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eOmega_axis2.pdf');
    exportgraphics(f6,fileName,'Append',true);
end


% Axis 3 velocity error
f7 = figure;

hold on

plot(timeVec,dataStruct.data.eOmega.com(:,3),'b-')
plot(timeVec,dataStruct.data.eOmega.DisObs(:,3),'r--')
plot(timeVec,dataStruct.data.eOmega.FricComp(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-8.7 8.5].*1.1)
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{d} - \dot{\theta}$ / $^{\circ}$/s')
yticks([-9 -6 -3 0 3 6 9])
legend('CT+RK+SB','CT+RK','CT','Location','southeast')
grid on
box on

f7.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eOmega_axis3.pdf');
    exportgraphics(f7,fileName,'Append',true);
end


% Axis 1 position error 
f8 = figure;

hold on

plot(timeVec,dataStruct.data.eTheta.com(:,1),'b-')
plot(timeVec,dataStruct.data.eTheta.cad(:,1),'r--')
plot(timeVec,dataStruct.data.eTheta.val(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-3.2 6.1].*1.1)
xlabel('$t$ / s')
ylabel('$\theta_{d} - \theta$ / $^{\circ}$')
legend('Iden. Modell','CAD Modell','ACOPOS','Location','southeast')
grid on
box on

f8.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eTheta_cad_axis1.pdf');
    exportgraphics(f8,fileName,'Append',true);
end


% Axis 2 position error
f9 = figure;

hold on

plot(timeVec,dataStruct.data.eTheta.com(:,2),'b-')
plot(timeVec,dataStruct.data.eTheta.cad(:,2),'r--')
plot(timeVec,dataStruct.data.eTheta.val(:,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5.4 5.4].*1.1)
xlabel('$t$ / s')
ylabel('$\theta_{d} - \theta$ / $^{\circ}$')
yticks([-8 -6 -4 -2 0 2 4 6 8])
legend('Iden. Modell','CAD Modell','ACOPOS','Location','northeast')
grid on
box on

f9.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eTheta_cad_axis2.pdf');
    exportgraphics(f9,fileName,'Append',true);
end


% Axis 3 position error
f10 = figure;

hold on

plot(timeVec,dataStruct.data.eTheta.com(:,3),'b-')
plot(timeVec,dataStruct.data.eTheta.cad(:,3),'r--')
plot(timeVec,dataStruct.data.eTheta.val(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5 8].*1.1)
xlabel('$t$ / s')
ylabel('$\theta_{d} - \theta$ / $^{\circ}$')
legend('Iden. Modell','CAD Modell','ACOPOS','Location','northeast')
grid on
box on

f10.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eTheta_cad_axis3.pdf');
    exportgraphics(f10,fileName,'Append',true);
end


% Axis 1 velocity error 
f11 = figure;

hold on

plot(timeVec,dataStruct.data.eOmega.com(:,1),'b-')
plot(timeVec,dataStruct.data.eOmega.cad(:,1),'r--')
plot(timeVec,dataStruct.data.eOmega.val(:,1),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5.6 4.8].*1.1)
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{d} - \dot{\theta}$ / $^{\circ}$/s')
legend('Iden. Modell','CAD Modell','ACOPOS','Location','southwest')
grid on
box on

f11.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eOmega_cad_axis1.pdf');
    exportgraphics(f11,fileName,'Append',true);
end


% Axis 2 velocity error 
f12 = figure;

hold on

plot(timeVec,dataStruct.data.eOmega.com(:,2),'b-')
plot(timeVec,dataStruct.data.eOmega.cad(:,2),'r--')
plot(timeVec,dataStruct.data.eOmega.val(:,2),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5.5 4.9].*1.1)
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{d} - \dot{\theta}$ / $^{\circ}$/s')
legend('Iden. Modell','CAD Modell','ACOPOS','Location','northeast')
grid on
box on

f12.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eOmega_cad_axis2.pdf');
    exportgraphics(f12,fileName,'Append',true);
end


% Axis 2 velocity error 
f13 = figure;

hold on

plot(timeVec,dataStruct.data.eOmega.com(:,3),'b-')
plot(timeVec,dataStruct.data.eOmega.cad(:,3),'r--')
plot(timeVec,dataStruct.data.eOmega.val(:,3),'--','Color',[0.4660 0.6740 0.1880])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-7.7 6.4].*1.1)
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{d} - \dot{\theta}$ / $^{\circ}$/s')
yticks([-8 -6 -4 -2 0 2 4 6 8])
legend('Iden. Modell','CAD Modell','ACOPOS','Location','northeast')
grid on
box on

f13.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_eOmega_cad_axis3.pdf');
    exportgraphics(f13,fileName,'Append',true);
end


% Axis 1 u
f14 = figure;

hold on

plot(timeVec,dataStruct.data.uGes_com(:,1),'b-')
plot(timeVec,dataStruct.data.uFbackLin_com(:,1),'r--')
plot(timeVec,dataStruct.data.uFricComp_com(:,1),'--','Color',[0.4660 0.6740 0.1880])
plot(timeVec,dataStruct.data.uDisObs_com(:,1),'--','Color',[0 1 0])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-31.4 25.8].*1.1)
xlabel('$t$ / s')
ylabel('$u$ / Nm')
legend('$u_{1}$','$u_{CT,1}$','$u_{R,1}$','$u_{SB,1}$','Location','eastoutside')
grid on
box on

f14.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_u_axis1.pdf');
    exportgraphics(f14,fileName,'Append',true);
end


% Axis 2 u
f15 = figure;

hold on

plot(timeVec,dataStruct.data.uGes_com(:,2),'b-')
plot(timeVec,dataStruct.data.uFbackLin_com(:,2),'r--')
plot(timeVec,dataStruct.data.uFricComp_com(:,2),'--','Color',[0.4660 0.6740 0.1880])
plot(timeVec,dataStruct.data.uDisObs_com(:,2),'--','Color',[0 1 0])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-28.3 41.5].*1.1)
xlabel('$t$ / s')
ylabel('$u$ / Nm')
legend('$u_{2}$','$u_{CT,2}$','$u_{R,2}$','$u_{SB,2}$','Location','eastoutside')
grid on
box on

f15.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_u_axis2.pdf');
    exportgraphics(f15,fileName,'Append',true);
end


% Axis 3 u
f16 = figure;

hold on

plot(timeVec,dataStruct.data.uGes_com(:,3),'b-')
plot(timeVec,dataStruct.data.uFbackLin_com(:,3),'r--')
plot(timeVec,dataStruct.data.uFricComp_com(:,3),'--','Color',[0.4660 0.6740 0.1880])
plot(timeVec,dataStruct.data.uDisObs_com(:,3),'--','Color',[0 1 0])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5.1 6.5].*1.1)
xlabel('$t$ / s')
ylabel('$u$ / Nm')
yticks([-9 -6 -3 0 3 6 9])
legend('$u_{,3}$','$u_{CT,3}$','$u_{R,3}$','$u_{SB,3}$','Location','eastoutside')
grid on
box on

f16.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_u_axis3.pdf');
    exportgraphics(f16,fileName,'Append',true);
end

%{
% u parts
f14 = figure;

subplot(3,1,1)

hold on

plot(timeVec,dataStruct.data.uGes_com(:,1),'b-')
plot(timeVec,dataStruct.data.uFbackLin_com(:,1),'r--')
plot(timeVec,dataStruct.data.uFricComp_com(:,1),'--','Color',[0.4660 0.6740 0.1880])
plot(timeVec,dataStruct.data.uDisObs_com(:,1),'--','Color',[0.8500 0.3250 0.0980])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-31.4 25.8].*1.1)
ylabel('$u$ / Nm')
legend('$u_{1}$','$u_{CT,1}$','$u_{R,1}$','$u_{SB,1}$','Location','eastoutside')
%yticks([-150 -100 -50 0 50 100])
grid on
box on

subplot(3,1,2)

hold on

plot(timeVec,dataStruct.data.uGes_com(:,2),'b-')
plot(timeVec,dataStruct.data.uFbackLin_com(:,2),'r--')
plot(timeVec,dataStruct.data.uFricComp_com(:,2),'--','Color',[0.4660 0.6740 0.1880])
plot(timeVec,dataStruct.data.uDisObs_com(:,2),'--','Color',[0.8500 0.3250 0.0980])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-28.3 41.5].*1.1)
ylabel('$u$ / Nm')
legend('$u_{2}$','$u_{CT,2}$','$u_{R,2}$','$u_{SB,2}$','Location','eastoutside')
%yticks([-90 -60 -30 0 30 60 90])
grid on
box on

subplot(3,1,3)

hold on

plot(timeVec,dataStruct.data.uGes_com(:,3),'b-')
plot(timeVec,dataStruct.data.uFbackLin_com(:,3),'r--')
plot(timeVec,dataStruct.data.uFricComp_com(:,3),'--','Color',[0.4660 0.6740 0.1880])
plot(timeVec,dataStruct.data.uDisObs_com(:,3),'--','Color',[0.8500 0.3250 0.0980])

hold off

xlim([timeVec(1) timeVec(end)])
ylim([-5.1 7.2].*1.1)
xlabel('$t$ / s')
ylabel('$u$ / Nm')
yticks([-9 -6 -3 0 3 6 9])
legend('$u_{,3}$','$u_{CT,3}$','$u_{R,3}$','$u_{SB,3}$','Location','eastoutside')
%yticks([-90 -60 -30 0 30 60 90])
grid on
box on

f14.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/controller_u.pdf');
    exportgraphics(f14,fileName,'Append',true);
end
%}