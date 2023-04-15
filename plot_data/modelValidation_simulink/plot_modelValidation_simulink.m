clear
close all

% Plot settings
saveData = 1;
pathOut = 'plot_data/modelValidation_simulink';

plotWidth = 730;
plotHeight = 550;

%xticksVec = [-xlimit, -0.5*xlimit, 0, 0.5*xlimit, xlimit];
%xticksString = {'' '' '$0$' '' ''};
%yticksVec = [-F_S, -F_C, 0, F_C, F_S];
%yticksString = {'$-F_S$' '$-F_C$' '$0$' '$F_C$' '$F_S$'};

% Create Output path
if ~exist(pathOut,'dir') && saveData
    mkdir(pathOut);
end

% Change to Latex interpreter
listFactory = fieldnames(get(groot,'factory'));
idxInterpreter = find(contains(listFactory,'Interpreter'));
for k = 1:length(idxInterpreter)
    defaultName = strrep(listFactory{idxInterpreter(k)},'factory','default');
    set(groot, defaultName,'latex');
end

simData_complete = load('simulation_data/model_validation/complete_model.mat');
simData_reduced = load('simulation_data/model_validation/reduced_model.mat');

timeVec_complete = simData_complete.simulation.timeVec;
theta_complete = rad2deg(simData_complete.simulation.theta);
omega_complete = rad2deg(simData_complete.simulation.omega);

timeVec_reduced = simData_reduced.simulation.timeVec;
theta_reduced = rad2deg(simData_reduced.simulation.theta);
omega_reduced = rad2deg(simData_reduced.simulation.omega);

timeVec_sampled = (0:0.01:timeVec_complete(end))';
theta_complete_sampled = interp1(timeVec_complete,theta_complete,timeVec_sampled);
theta_reduced_sampled = interp1(timeVec_reduced,theta_reduced,timeVec_sampled);



% Position and velocity of complete model
f1 = figure;

subplot(2,1,1)
hold on

plot(timeVec_complete,theta_complete(:,1),'LineStyle','-','Color',[1 0 0])
plot(timeVec_complete,theta_complete(:,2),'LineStyle','-','Color',[0 0 1])
plot(timeVec_complete,theta_complete(:,3),'LineStyle','-','Color',[0.4660 0.6740 0.1880])

%plot(timeVec_complete,theta_complete(:,4),'LineStyle','--','Color',[1 0 0])
%plot(timeVec_complete,theta_complete(:,5),'LineStyle','--','Color',[0 0 1])
%plot(timeVec_complete,theta_complete(:,6),'LineStyle','--','Color',[0.4660 0.6740 0.1880])

hold off

ylim([-200 315])
ylabel('$\zeta$ / $^{\circ}$')
legend('$\zeta_{1}$','$\zeta_{2}$','$\zeta_{3}$')%,'$\theta_{1}$','$\theta_{2}$','$\theta_{3}$')
yticks([-540 -360 -180 0 180])
grid on
box on

subplot(2,1,2)
hold on

plot(timeVec_complete,omega_complete(:,1),'LineStyle','-','Color',[1 0 0])
plot(timeVec_complete,omega_complete(:,2),'LineStyle','-','Color',[0 0 1])
plot(timeVec_complete,omega_complete(:,3),'LineStyle','-','Color',[0.4660 0.6740 0.1880])

%plot(timeVec_complete,omega_complete(:,4),'LineStyle','--','Color',[1 0 0])
%plot(timeVec_complete,omega_complete(:,5),'LineStyle','--','Color',[0 0 1])
%plot(timeVec_complete,omega_complete(:,6),'LineStyle','--','Color',[0.4660 0.6740 0.1880])

hold off

ylim([-350 350])
ylabel('$\dot{\zeta}$ / $^{\circ}$/s')
xlabel('$t$ / s')
yticks([-270 -180 -90 0 90 180 270])
legend('$\dot{\zeta}_{1}$','$\dot{\zeta}_{2}$','$\dot{\zeta}_{3}$')%,'$\dot{\theta}_{1}$','$\dot{\theta}_{2}$','$\dot{\theta}_{3}$')
grid on
box on

f1.Position(3) = plotWidth;
%f1.Position(4) = plotHeight;

if saveData
    fileName = strcat(pathOut,'/completeModel_theta_omega.pdf');
    exportgraphics(f1,fileName,'Append',true);
end

maxThetaDiff = max(abs(theta_complete(:,1:3)-theta_complete(:,4:6)))
thetaEnd = theta_complete(end,1:3)


% Zoom velocity
f2 = figure;

hold on

plot(timeVec_complete,omega_complete(:,3),'LineStyle','-','Color',[1 0 0])
plot(timeVec_complete,omega_complete(:,6),'LineStyle','--','Color',[0 0 1])

hold off

xlim([9.95 10.75])
ylim([-25 10])
ylabel('$\dot{q}$ / $^{\circ}$/s')
xlabel('$t$ / s')
legend('$\dot{\zeta}_{3}$','$\dot{\theta}_{3}$')
grid on
box on

f2.Position(3) = plotWidth;
f2.Position(4) = 275;

if saveData
    fileName = strcat(pathOut,'/completeModel_omega_zoom.pdf');
    exportgraphics(f2,fileName,'Append',true);
end



% Differenz between complete and reduced model
plotHeight = 275;

f3 = figure;

hold on

plot(timeVec_sampled,theta_complete_sampled(:,1) - theta_reduced_sampled(:,1),'LineStyle','-','Color',[1 0 0])
plot(timeVec_sampled,theta_complete_sampled(:,2) - theta_reduced_sampled(:,2),'LineStyle','-','Color',[0 0 1])
plot(timeVec_sampled,theta_complete_sampled(:,3) - theta_reduced_sampled(:,3),'LineStyle','-','Color',[0.4660 0.6740 0.1880])

hold off

ylim([-0.7 0.7])
ylabel('$\Delta\zeta = \zeta_{kom} - \zeta_{red}$ / $^{\circ}$')
xlabel('$t$ / s')
legend('$\Delta\zeta_{1}$','$\Delta\zeta_{2}$','$\Delta\zeta_{3}$')
grid on
box on

f3.Position(3) = plotWidth;
f3.Position(4) = 275;

if saveData
    fileName = strcat(pathOut,'/diffComRed_theta.pdf');
    exportgraphics(f3,fileName,'Append',true);
end

maxDiffComRed = max(abs(theta_complete_sampled(:,1:3)-theta_reduced_sampled(:,1:3)))