%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'trajectory/traj/staticFriction_omegaT';
MODEL_FOLDER = 'identification/models/staticFriction';
OUTPUT_FOLDER = 'plot_data/identification/staticFriction_omegaT';
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
trajStruct_axis1 = load([TRAJ_FOLDER '/traj_staticFriction_omegaT_axis1.mat']);

timeVec_traj_axis1 = trajStruct_axis1.time;
theta_traj_axis1 = trajStruct_axis1.theta(:,1);
omega_traj_axis1 = trajStruct_axis1.omega(:,1);

tIden_end = 70.5;


%% Load identification data
% Axis 1
mdlStruct_axis1 = load([MODEL_FOLDER '/model_staticFriction_omegaT_axis1.mat']);

timeVecM1_axis1 = mdlStruct_axis1.data.meas1.time;
omegaMotorM1_axis1 = mdlStruct_axis1.data.meas1.omegaMotor;
tauMotorM1_axis1 = mdlStruct_axis1.data.meas1.tauMotor .* i_g(1);
idxCutM1_axis1 = mdlStruct_axis1.data.meas1.idxCut;

omegaMotor_axis1 = mdlStruct_axis1.data.omegaMotor;
TMotor_axis1 = mdlStruct_axis1.data.TMotor;
tauMotor_axis1 = mdlStruct_axis1.data.tauMotor;
tauMotor_axis1{1} = tauMotor_axis1{1} .* i_g(1);
tauMotor_axis1{2} = tauMotor_axis1{2} .* i_g(1);
tauF_mdl_axis1 = mdlStruct_axis1.data.tauF_mdl;
tauF_mdl_axis1{1} = tauF_mdl_axis1{1} .* i_g(1);
tauF_mdl_axis1{2} = tauF_mdl_axis1{2} .* i_g(1);

timeVecTot_axis1 = mdlStruct_axis1.data.timeVec;
T_axis1 = mdlStruct_axis1.data.T;

tM1_axis1 = timeVecM1_axis1(end);
timeVecCut_axis1 = (tM1_axis1:tM1_axis1:timeVecTot_axis1(end))';

surf_omegaMotor_axis1 = [ ...
    flip(omegaMotor_axis1{2},1); ...
    omegaMotor_axis1{1} ...
    ];
surf_TMotor_axis1 = [ ...
    flip(TMotor_axis1{2},1); ...
    TMotor_axis1{1} ...
    ];
surf_tauMotor_axis1 = [ ...
    flip(tauMotor_axis1{2},1); ...
    tauMotor_axis1{1} ...
    ];

% Axis 2
mdlStruct_axis2 = load([MODEL_FOLDER '/model_staticFriction_omegaT_axis2.mat']);

omegaMotor_axis2 = mdlStruct_axis2.data.omegaMotor;
TMotor_axis2 = mdlStruct_axis2.data.TMotor;
tauMotor_axis2 = mdlStruct_axis2.data.tauMotor;
tauMotor_axis2{1} = tauMotor_axis2{1} .* i_g(2);
tauMotor_axis2{2} = tauMotor_axis2{2} .* i_g(2);
tauF_mdl_axis2 = mdlStruct_axis2.data.tauF_mdl;
tauF_mdl_axis2{1} = tauF_mdl_axis2{1} .* i_g(2);
tauF_mdl_axis2{2} = tauF_mdl_axis2{2} .* i_g(2);

surf_omegaMotor_axis2 = [ ...
    flip(omegaMotor_axis2{2},1); ...
    omegaMotor_axis2{1} ...
    ];
surf_TMotor_axis2 = [ ...
    flip(TMotor_axis2{2},1); ...
    TMotor_axis2{1} ...
    ];
surf_tauMotor_axis2 = [ ...
    flip(tauMotor_axis2{2},1); ...
    tauMotor_axis2{1} ...
    ];

% Axis 3
mdlStruct_axis3 = load([MODEL_FOLDER '/model_staticFriction_omegaT_axis3.mat']);

omegaMotor_axis3 = mdlStruct_axis3.data.omegaMotor;
TMotor_axis3 = mdlStruct_axis3.data.TMotor;
tauMotor_axis3 = mdlStruct_axis3.data.tauMotor;
tauMotor_axis3{1} = tauMotor_axis3{1} .* i_g(3);
tauMotor_axis3{2} = tauMotor_axis3{2} .* i_g(3);
tauF_mdl_axis3 = mdlStruct_axis3.data.tauF_mdl;
tauF_mdl_axis3{1} = tauF_mdl_axis3{1} .* i_g(3);
tauF_mdl_axis3{2} = tauF_mdl_axis3{2} .* i_g(3);

surf_omegaMotor_axis3 = [ ...
    flip(omegaMotor_axis3{2},1); ...
    omegaMotor_axis3{1} ...
    ];
surf_TMotor_axis3 = [ ...
    flip(TMotor_axis3{2},1); ...
    TMotor_axis3{1} ...
    ];
surf_tauMotor_axis3 = [ ...
    flip(tauMotor_axis3{2},1); ...
    tauMotor_axis3{1} ...
    ];

%% Plot Data
% Axis 1 trajectory
f1 = figure;

subplot(2,1,1)
hold on

plot(timeVec_traj_axis1,theta_traj_axis1)
xline(tIden_end,'r--')

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-90 90])
ylabel('$\theta_{t}$ / $^{\circ}$')
%legend('$\theta_{1}$')
yticks([-90 -45 0 45 90])
grid on
box on

subplot(2,1,2)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)
xline(70.5,'r--')

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-120 120])
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
%legend('$\dot{\theta}_{1}$')
yticks([-100 -50 0 50 100])
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_traj_axis1.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 first measurment
f2 = figure;

subplot(2,1,1)
hold on

plot(timeVecM1_axis1,omegaMotorM1_axis1)

xline(timeVecM1_axis1(idxCutM1_axis1{1}(:,1)),'r--')
xline(timeVecM1_axis1(idxCutM1_axis1{1}(:,2)),'r--')
xline(timeVecM1_axis1(idxCutM1_axis1{2}(:,1)),'g--')
xline(timeVecM1_axis1(idxCutM1_axis1{2}(:,2)),'g--')

hold off

xlim([timeVec_traj_axis1(1) tIden_end])
ylim([-120 120])
ylabel('$\dot{\theta}$ / $^{\circ}$/s')
yticks([-100 -50 0 50 100])
grid on
box on

subplot(2,1,2)
hold on

plot(timeVecM1_axis1,tauMotorM1_axis1)

xline(timeVecM1_axis1(idxCutM1_axis1{1}(:,1)),'r--')
xline(timeVecM1_axis1(idxCutM1_axis1{1}(:,2)),'r--')
xline(timeVecM1_axis1(idxCutM1_axis1{2}(:,1)),'g--')
xline(timeVecM1_axis1(idxCutM1_axis1{2}(:,2)),'g--')

hold off

xlim([timeVec_traj_axis1(1) tIden_end])
%ylim([-0.42 0.42])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
%legend('$\dot{\theta}_{1}$')
%yticks([-100 -50 0 50 100])
grid on
box on

f2.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_measurment1_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


% Axis 1 temperature
f3 = figure;

hold on

plot(timeVecTot_axis1,T_axis1)

xline(timeVecCut_axis1,'r--')

hold off

xlim([timeVecTot_axis1(1) timeVecTot_axis1(end)])
ylim([15 35])
xlabel('$t$ / s')
ylabel('$T$ / $^{\circ}$C')
grid on
box on

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_T_axis1.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% Axis 1 surface plot friction
f4 = figure;

hold on

surf(surf_omegaMotor_axis1,surf_TMotor_axis1,surf_tauMotor_axis1)

hold off

xlim([-110 110])
ylim([17.5 35])
%zlim([-0.155 0.155])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$T$ / $^{\circ}$C')
zlabel('$\tau_{R,vT}$ / Nm')
grid on
%box on
view(30,-30)

f4.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_surface_axis1.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


% Axis 1 friction contourlines over velocity
f5_1 = figure;

hold on

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis1{1}(:,plotMeasurments(iMeas)),tauMotor_axis1{1}(:,plotMeasurments(iMeas)),'.-','Color',PLOT_COULOURS{iMeas},'MarkerSize',8)
    plot(omegaMotor_axis1{2}(:,plotMeasurments(iMeas)),tauMotor_axis1{2}(:,plotMeasurments(iMeas)),'.-','Color',PLOT_COULOURS{iMeas},'MarkerSize',8)
end

hold off

xlim([-110 110])
ylim([-22 22])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$\tau_{R,vT}$ / Nm')
grid on
box on
%{
legend( ...
    ['Messung ' num2str(plotMeasurments(1))],'', ...
    ['Messung ' num2str(plotMeasurments(2))],'', ...
    ['Messung ' num2str(plotMeasurments(3))],'', ...
    ['Messung ' num2str(plotMeasurments(4))],'', ...
    ['Messung ' num2str(plotMeasurments(5))],'', ...
    ['Messung ' num2str(plotMeasurments(6))],'', ...
    'Location','southeast')
%}

f5_1.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_contourOmega_axis1.pdf');
    exportgraphics(f5_1,fileName,'Append',true);
end

f5_2 = figure;

hold on

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis1{1}(:,plotMeasurments(iMeas)),tauMotor_axis1{1}(:,plotMeasurments(iMeas)),'.-','Color',PLOT_COULOURS{iMeas},'MarkerSize',8)
    plot(omegaMotor_axis1{2}(:,plotMeasurments(iMeas)),tauMotor_axis1{2}(:,plotMeasurments(iMeas)),'.-','Color',PLOT_COULOURS{iMeas},'MarkerSize',8)
end

hold off

xlim([0 5])
ylim([2.5 7.5])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$\tau_{R,vT}$ / Nm')
grid on
box on
legend( ...
    ['Messung ' num2str(plotMeasurments(1))],'', ...
    ['Messung ' num2str(plotMeasurments(2))],'', ...
    ['Messung ' num2str(plotMeasurments(3))],'', ...
    ['Messung ' num2str(plotMeasurments(4))],'', ...
    ['Messung ' num2str(plotMeasurments(5))],'', ...
    ['Messung ' num2str(plotMeasurments(6))],'', ...
    'Location','eastoutside')

f5_2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_contourOmega_zoom_axis1.pdf');
    exportgraphics(f5_2,fileName,'Append',true);
end


% Axis 2 friction contourlines over temperature
f6 = figure;

hold on

for iOmega = 1:length(plotOmegas)

    if iOmega == length(plotOmegas)
        offset = -1;
    elseif iOmega == length(plotOmegas)-1
        offset = 1;
    else
        offset = 0;
    end
    plot(TMotor_axis2{1}(plotOmegas(iOmega),:),tauMotor_axis2{1}(plotOmegas(iOmega),:),'.-','Color',PLOT_COULOURS{iOmega},'MarkerSize',8)
    plot(TMotor_axis2{2}(plotOmegas(iOmega),:),tauMotor_axis2{2}(plotOmegas(iOmega),:),'.-','Color',PLOT_COULOURS{iOmega},'MarkerSize',8)

end

hold off

%xlim([-120 120])
ylim([-22 22])
xlabel('$T$ / $^{\circ}$C')
ylabel('$\tau_{R,vT}$ / Nm')
grid on
box on
legend( ...
    '$\pm 0,25$ $^{\circ}$/s','', ...
    '$\pm 1$ $^{\circ}$/s','', ...
    '$\pm 4$ $^{\circ}$/s','', ...
    '$\pm 16$ $^{\circ}$/s','', ...
    '$\pm 36$ $^{\circ}$/s','', ...
    '$\pm 64$ $^{\circ}$/s','', ...
    '$\pm 100$ $^{\circ}$/s','', ...
    'Location','eastoutside')

f6.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_contourT_axis2.pdf');
    exportgraphics(f6,fileName,'Append',true);
end


% Axis 1 friction identification
f7 = figure;

hold on

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis1{1}(:,plotMeasurments(iMeas)),tauMotor_axis1{1}(:,plotMeasurments(iMeas)),'x','Color',PLOT_COULOURS{iMeas})
    plot(omegaMotor_axis1{2}(:,plotMeasurments(iMeas)),tauMotor_axis1{2}(:,plotMeasurments(iMeas)),'x','Color',PLOT_COULOURS{iMeas})
end

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis1{1}(:,plotMeasurments(iMeas)),tauF_mdl_axis1{1}(:,plotMeasurments(iMeas)),'-','Color',PLOT_COULOURS{iMeas})
    plot(omegaMotor_axis1{2}(:,plotMeasurments(iMeas)),tauF_mdl_axis1{2}(:,plotMeasurments(iMeas)),'-','Color',PLOT_COULOURS{iMeas})
end

hold off

xlim([-110 110])
ylim([-22 22])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$\tau_{R,vT}$ / Nm')
grid on
box on
legend('Messdaten','','','','','','','','','','','','Modell','Location','southeast')

f7.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_identification_axis1.pdf');
    exportgraphics(f7,fileName,'Append',true);
end


% Axis 2 surface plot friction
f8 = figure;

hold on

surf(surf_omegaMotor_axis2,surf_TMotor_axis2,surf_tauMotor_axis2)

hold off

xlim([-110 110])
ylim([17.5 40])
%zlim([-0.155 0.155])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$T$ / $^{\circ}$C')
zlabel('$\tau_{R,vT}$ / Nm')
grid on
%box on
view(30,-30)

f8.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_surface_axis2.pdf');
    exportgraphics(f8,fileName,'Append',true);
end


% Axis 2 friction identification
f9 = figure;

hold on

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis2{1}(:,plotMeasurments(iMeas)),tauMotor_axis2{1}(:,plotMeasurments(iMeas)),'x','Color',PLOT_COULOURS{iMeas})
    plot(omegaMotor_axis2{2}(:,plotMeasurments(iMeas)),tauMotor_axis2{2}(:,plotMeasurments(iMeas)),'x','Color',PLOT_COULOURS{iMeas})
end

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis2{1}(:,plotMeasurments(iMeas)),tauF_mdl_axis2{1}(:,plotMeasurments(iMeas)),'-','Color',PLOT_COULOURS{iMeas})
    plot(omegaMotor_axis2{2}(:,plotMeasurments(iMeas)),tauF_mdl_axis2{2}(:,plotMeasurments(iMeas)),'-','Color',PLOT_COULOURS{iMeas})
end

hold off

xlim([-110 110])
ylim([-22 22])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$\tau_{R,vT}$ / Nm')
grid on
box on
legend('Messdaten','','','','','','','','','','','','','Modell','Location','southeast')

f9.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_identification_axis2.pdf');
    exportgraphics(f9,fileName,'Append',true);
end


% Axis 3 surface plot friction
f10 = figure;

hold on

surf(surf_omegaMotor_axis3,surf_TMotor_axis3,surf_tauMotor_axis3)

hold off

xlim([-110 110])
ylim([17.5 40])
%zlim([-0.155 0.155])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$T$ / $^{\circ}$C')
zlabel('$\tau_{R,vT}$ / Nm')
grid on
%box on
view(30,-30)

f10.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_surface_axis3.pdf');
    exportgraphics(f10,fileName,'Append',true);
end


% Axis 3 friction identification
f11 = figure;

hold on

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis3{1}(:,plotMeasurments(iMeas)),tauMotor_axis3{1}(:,plotMeasurments(iMeas)),'x','Color',PLOT_COULOURS{iMeas})
    plot(omegaMotor_axis3{2}(:,plotMeasurments(iMeas)),tauMotor_axis3{2}(:,plotMeasurments(iMeas)),'x','Color',PLOT_COULOURS{iMeas})
end

for iMeas = 1:length(plotMeasurments)
    plot(omegaMotor_axis3{1}(:,plotMeasurments(iMeas)),tauF_mdl_axis3{1}(:,plotMeasurments(iMeas)),'-','Color',PLOT_COULOURS{iMeas})
    plot(omegaMotor_axis3{2}(:,plotMeasurments(iMeas)),tauF_mdl_axis3{2}(:,plotMeasurments(iMeas)),'-','Color',PLOT_COULOURS{iMeas})
end

hold off

xlim([-110 110])
ylim([-5 5])
xlabel('$\dot{\theta}$ / $^{\circ}$/s')
ylabel('$\tau_{R,vT}$ / Nm')
grid on
box on
legend('Messdaten','','','','','','','','','','','','','Modell','Location','southeast')

f11.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT*1.5];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/staticFriction_omegaT_identification_axis3.pdf');
    exportgraphics(f11,fileName,'Append',true);
end





%{
% Axis 2 friction contourlines over temperature
f20 = figure;

hold on

for iOmega = 1:length(plotOmegas)

    if iOmega == length(plotOmegas)
        offset = -1;
    elseif iOmega == length(plotOmegas)-1
        offset = 1;
    else
        offset = 0;
    end
    plot(TMotor_axis2{1}(plotOmegas(iOmega),:),tauMotor_axis2{1}(plotOmegas(iOmega),:),[PLOT_COULOURS{iOmega+offset},'.-'],'MarkerSize',8)
    plot(TMotor_axis2{2}(plotOmegas(iOmega),:),tauMotor_axis2{2}(plotOmegas(iOmega),:),[PLOT_COULOURS{iOmega+offset},'.-'],'MarkerSize',8)

end

hold off

%xlim([-120 120])
ylim([-22 22])
xlabel('$T$ / $^{\circ}$C')
ylabel('$\tau_{R}$ / Nm')
grid on
box on
legend( ...
    '$\pm 0,25$ $^{\circ}$/s','', ...
    '$\pm 1$ $^{\circ}$/s','', ...
    '$\pm 4$ $^{\circ}$/s','', ...
    '$\pm 16$ $^{\circ}$/s','', ...
    '$\pm 36$ $^{\circ}$/s','', ...
    '$\pm 64$ $^{\circ}$/s','', ...
    '$\pm 100$ $^{\circ}$/s','', ...
    'Location','eastoutside')

f20.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUPUT_FOLDER,'/staticFriction_omegaT_contourT_axis1.pdf');
    exportgraphics(f20,fileName,'Append',true);
end
%}