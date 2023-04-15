clear
close all

% Plot settings
saveData = 1;
pathOut = 'plot_data/motor_dynamic';

plotWidth = 730;
plotHeight = 275;

% Create Output path
if ~exist(pathOut,'dir') && saveData
    mkdir(pathOut);
end

% Change to Latex interpreter
listFactory = fieldnames(get(groot,'factory'));
idxInterpreter = find(contains(listFactory,'Interpreter'));
for kMotor = 1:length(idxInterpreter)
    defaultName = strrep(listFactory{idxInterpreter(kMotor)},'factory','default');
    set(groot, defaultName,'latex');
end


expDataTheta = load('measurments/10_20_stepResponse_theta_axis1/expData.mat');
expDataTau = load('measurments/10_25_stepResonse_tau_axis1/expData.mat');


f1 = figure;

subplot(2,1,1)
hold on

plot(expDataTheta.t,expDataTheta.refValuePos(:,1),'r-')
plot(expDataTheta.t,expDataTheta.thetaMot(:,1),'b-')

hold off

xlim(25.009+[-1 1])
ylim([-0.5 1.5])
ylabel('$\theta$ / $^{\circ}$')
legend('$\theta_{ref}$','$\theta$')
xticks(25.009+[-1:1:1])
xticklabels({'$0$' '$1$' '$2$'});
grid on
box on

subplot(2,1,2)
hold on

plot(expDataTau.t,expDataTau.refValueTorque(:,1),'r-')
plot(expDataTau.t,expDataTau.torque(:,1),'b-')

hold off

xlim(25.009+[-1 1])
%ylim([-0.05 0.15])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('$\tau_{m,ref}$','$\tau_{m}$')
xticks(25.009+[-1:1:1])
xticklabels({'$0$' '$1$' '$2$'});
grid on
box on

f1.Position(3) = plotWidth;

if saveData
    fileName = strcat(pathOut,'/stepResponse_motor_axis1_theta_tau.pdf');
    exportgraphics(f1,fileName,'Append',true);
end


f2 = figure;

subplot(2,1,1)
hold on

plot(expDataTheta.t,expDataTheta.refValuePos(:,1),'r-')
plot(expDataTheta.t,expDataTheta.thetaMot(:,1),'b-')

hold off

xlim(25.009+[-1 1])
ylim([-0.5 1.5])
ylabel('$\theta$ / $^{\circ}$')
legend('$\theta_{ref}$','$\theta$')
xticks(25.009+[-1:1:1])
xticklabels({'$0$' '$0,25$' '$0,5$'});
grid on
box on

subplot(2,1,2)
hold on

plot(expDataTheta.t,expDataTheta.torque(:,1),'b-')

hold off

xlim(25.009+[-1 1])
%ylim([-0.05 0.15])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('$\tau_{m}$')
xticks(25.009+[-1:1:1])
xticklabels({'$0$' '$0,25$' '$0,5$'});
grid on
box on

f2.Position(3) = plotWidth;

if saveData
    fileName = strcat(pathOut,'/stepResponse_motor_axis1_theta.pdf');
    exportgraphics(f2,fileName,'Append',true);
end


f3 = figure;

hold on

plot(expDataTau.t,expDataTau.refValueTorque(:,1),'r-')
plot(expDataTau.t,expDataTau.torque(:,1),'b-')


hold off

xlim(25.009+[-1 1])
%ylim([-0.05 0.15])
xlabel('$t$ / s')
ylabel('$\tau_{m}$ / Nm')
legend('$\tau_{m,ref}$','$\tau_{m}$')
xticks(25.009+[-1:1:1])
xticklabels({'$0$' '$0,25$' '$0,5$'});
grid on
box on

f3.Position(3) = plotWidth;
f3.Position(4) = plotHeight;

if saveData
    fileName = strcat(pathOut,'/stepResponse_motor_axis1_tau.pdf');
    exportgraphics(f3,fileName,'Append',true);
end

timeTheta = 0.13621 - 0.0018
timeTau = 0.01587 - 0.00187

factor = (timeTau / timeTheta)*100