%% Initialization
clear;
close all;
addpath(pathdef_local);


% Plot settings
SAVE_PLOTS = 1;
TRAJ_FOLDER = 'identification_trajectories/traj/kinError';
MODEL_FOLDER = 'identification/models/kinError';
OUTPUT_FOLDER = 'plot_data/identification/kinError';
SAMPLE_FREQ = 500;

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


%% Load trajectory data
trajStruct_axis1 = load([TRAJ_FOLDER '/traj_kinError_axis1.mat']);

timeVec_traj_axis1 = trajStruct_axis1.time;
theta_traj_axis1 = trajStruct_axis1.theta(:,1);
omega_traj_axis1 = trajStruct_axis1.omega(:,1);


%% Load identification data
% Axis 1
mdlStruct_axis1 = load([MODEL_FOLDER '/model_kinError_axis1.mat']);

thetaMotor_axis1 = mdlStruct_axis1.data.thetaMotor;
thetaDiff_axis1 = mdlStruct_axis1.data.thetaDiff;
backlash_axis1 = mdlStruct_axis1.data.backlash;
kinError_axis1 = mdlStruct_axis1.data.kinError;

L = length(kinError_axis1);
fVec_axis1 = SAMPLE_FREQ*(0:(L/2))/L;
kinError_fftData = fft(kinError_axis1);
P2 = abs(kinError_fftData/L);
P1_axis1 = P2(1:L/2+1);
P1_axis1(2:end-1) = 2*P1_axis1(2:end-1);

xLookup_axis1 = mdlStruct_axis1.param.xLookup;
yLookup_axis1 = mdlStruct_axis1.param.yLookup;
npError = interp1(xLookup_axis1,yLookup_axis1,thetaMotor_axis1,'linear');
c_pError = mdlStruct_axis1.param.c_pError;
pError = kinError_periodic_axis1_fun(c_pError',thetaMotor_axis1);
kinError_mdl_axis1 = npError + pError;

% Axis 2
mdlStruct_axis2 = load([MODEL_FOLDER '/model_kinError_axis2.mat']);

thetaMotor_axis2 = mdlStruct_axis2.data.thetaMotor;
kinError_axis2 = mdlStruct_axis2.data.kinError;

L = length(kinError_axis2);
fVec_axis2 = SAMPLE_FREQ*(0:(L/2))/L;
kinError_fftData = fft(kinError_axis2);
P2 = abs(kinError_fftData/L);
P1_axis2 = P2(1:L/2+1);
P1_axis2(2:end-1) = 2*P1_axis2(2:end-1);

kinError_mdl_axis2 = interp1(mdlStruct_axis2.param.xLookup,mdlStruct_axis2.param.yLookup,thetaMotor_axis2,'linear');

% Axis 3
mdlStruct_axis3 = load([MODEL_FOLDER '/model_kinError_axis3.mat']);

thetaMotor_axis3 = mdlStruct_axis3.data.thetaMotor;
kinError_axis3 = mdlStruct_axis3.data.kinError;

kinError_mdl_axis3 = interp1(mdlStruct_axis3.param.xLookup,mdlStruct_axis3.param.yLookup,thetaMotor_axis3,'linear');

%% Plot Data
% Axis 1 trajectory
f1 = figure;

subplot(2,1,1)
hold on

plot(timeVec_traj_axis1,theta_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-180 180])
ylabel('$\theta_{t}$ / $^{\circ}$')
%legend('$\theta_{1}$')
yticks([-180 -90 0 90 180])
grid on
box on

subplot(2,1,2)
hold on

plot(timeVec_traj_axis1,omega_traj_axis1)

hold off

xlim([timeVec_traj_axis1(1) timeVec_traj_axis1(end)])
ylim([-0.3 0.3])
xlabel('$t$ / s')
ylabel('$\dot{\theta}_{t}$ / $^{\circ}$/s')
%legend('$\dot{\theta}_{1}$')
grid on
box on

f1.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_traj_axis1.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


% Axis 1 thetaDiff
f2 = figure;

hold on

plot(thetaMotor_axis1,thetaDiff_axis1{1},'b-')
plot(thetaMotor_axis1,thetaDiff_axis1{2},'r-')

hold off

xlim([thetaMotor_axis1(1) thetaMotor_axis1(end)])
ylim([-0.06 0.05])
xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi$ / $^{\circ}$')
legend('$\varphi_{+}$','$\varphi_{-}$','Location','southeast')
yticks([-0.05 -0.025 0 0.025 0.05])
grid on
box on

f2.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/thetaDiff_axis1.pdf');
    exportgraphics(f2,fileName,'Append',true);
end

% Axis 1 backlash
f3 = figure;

hold on

plot(thetaMotor_axis1,backlash_axis1,'b-')

hold off

xlim([thetaMotor_axis1(1) thetaMotor_axis1(end)])
ylim([2*1e-3 12*1e-3])
xlabel('$\theta$ / $^{\circ}$')
ylabel('$\frac{\varphi_{+} - \varphi_{-}}{2}$ / $^{\circ}$')
grid on
box on

f3.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/backlash_axis1.pdf');
    exportgraphics(f3,fileName,'Append',true);
end


% Axis 1 kinError
f4 = figure;

subplot(2,1,1)
hold on

plot(thetaMotor_axis1,kinError_axis1,'b-')

hold off

xlim([thetaMotor_axis1(1) thetaMotor_axis1(end)])
ylim([-0.06 0.05])
%xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
yticks([-0.05 -0.025 0 0.025 0.05])
grid on
box on

subplot(2,1,2)
hold on

plot(thetaMotor_axis1,kinError_axis1,'b-')

hold off

xlim([-20 20])
ylim([-0.04 0.04])
xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
yticks([-0.04 -0.02 0 0.02 0.04])
grid on
box on

f4.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_axis1.pdf');
    exportgraphics(f4,fileName,'Append',true);
end


% Axis 1 freq. spectrum
f5 = figure;

hold on

plot(fVec_axis1,P1_axis1)

hold off

xlim([0 2])
ylim([-0.3e-3 6e-3])
xlabel('Frequenz / Hz')
ylabel('Amplitude / $^{\circ}$')
grid on
box on

f5.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_freqSpec_axis1.pdf');
    exportgraphics(f5,fileName,'Append',true);
end


% Axis 1 kinError model
f6 = figure;

subplot(2,1,1)
hold on

plot(thetaMotor_axis1,kinError_axis1,'b-')
plot(thetaMotor_axis1,kinError_mdl_axis1,'r-')

hold off

xlim([thetaMotor_axis1(1) thetaMotor_axis1(end)])
ylim([-0.06 0.05])
%xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
legend('Messdaten','Modell','Location','southeast')
grid on
box on

subplot(2,1,2)
hold on

plot(thetaMotor_axis1,kinError_axis1,'b-')
plot(thetaMotor_axis1,kinError_mdl_axis1,'r-')

hold off

xlim([-20 20])
ylim([-0.02 0.035])
xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
legend('Messdaten','Modell','Location','southeast')
grid on
box on

f6.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_mdl_axis1.pdf');
    exportgraphics(f6,fileName,'Append',true);
end


% Axis 2 kinError model
f7 = figure;

subplot(2,1,1)
hold on

plot(thetaMotor_axis2,kinError_axis2,'b-')
plot(thetaMotor_axis2,kinError_mdl_axis2,'r-')

hold off

xlim([thetaMotor_axis2(1) thetaMotor_axis2(end)])
ylim([-0.093 0.035])
%xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
legend('Messdaten','Modell')
grid on
box on

subplot(2,1,2)
hold on

plot(thetaMotor_axis2,kinError_axis2,'b-')
plot(thetaMotor_axis2,kinError_mdl_axis2,'r-')

hold off

xlim([-20 20])
ylim([-0.045 0.015])
xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
legend('Messdaten','Modell')
grid on
box on

f7.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_mdl_axis2.pdf');
    exportgraphics(f7,fileName,'Append',true);
end


% Axis 2 freq. spectrum
f8 = figure;

hold on

plot(fVec_axis2,P1_axis2)

hold off

xlim([0 2])
ylim([-0.3e-3 6e-3])
xlabel('$f$ / Hz')
ylabel('Amplitude')
grid on
box on

f8.Position = [680 558 PLOT_WIDTH PLOT_HEIGHT];

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_freqSpec_axis1.pdf');
    exportgraphics(f8,fileName,'Append',true);
end


% Axis 3 kinError model
f9 = figure;

subplot(2,1,1)
hold on

plot(thetaMotor_axis3,kinError_axis3,'b-')
plot(thetaMotor_axis3,kinError_mdl_axis3,'r-')

hold off

xlim([thetaMotor_axis3(1) thetaMotor_axis3(end)])
ylim([-0.093 0.025])
%xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
legend('Messdaten','Modell')
grid on
box on

subplot(2,1,2)
hold on

plot(thetaMotor_axis3,kinError_axis3,'b-')
plot(thetaMotor_axis3,kinError_mdl_axis3,'r-')

hold off

xlim([-20 20])
ylim([-0.09 0.01])
xlabel('$\theta$ / $^{\circ}$')
ylabel('$\varphi_{kin}$ / $^{\circ}$')
legend('Messdaten','Modell')
grid on
box on

f9.Position(3) = PLOT_WIDTH;

if SAVE_PLOTS
    fileName = strcat(OUTPUT_FOLDER,'/kinError_mdl_axis3.pdf');
    exportgraphics(f9,fileName,'Append',true);
end
