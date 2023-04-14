%% Varies poles of lateral controller, runs simulations and graphically 
% compares resuls

clearvars

%% Parameters
simTime = 10;
tlc = 5;
vx0 = 20;
Yr = computeLateralReferenceTrajectory(vx0, tlc, simTime);


m = 2000; % [kg] Total mass of vehicle
lf = 3; % [m] distance from cg to front wheel
lr = 2; % [m] distance from cg to rear wheel
Cf = 12e3; %5e4; % front tire stiffness
Cr = 11e3; %4e4; % rear tire stiffness
Iz = 4000; % moment of inertia around vertical axis
d = 5; % look ahead distance for measuring error

% Controller
[A, B] = getLateralDynamics(vx0, Cf, Cr, lr, lf, m, Iz);

%% Run simulations
gainMultipliers = 1:0.5:3;
Y = cell(1, length(gainMultipliers));
ay = cell(1, length(gainMultipliers));
for n = 1:length(gainMultipliers)
    poles = -[1, 2, 3, 4]*gainMultipliers(n);
    Klat = place(A, B, poles);
    simOut = sim('lateralControlTests', 'SrcWorkspace','current');
    Y{n} = simOut.Y;
    ay{n} = simOut.yddot;
end

%% Plots
fontSize = 15;
yFig = figure; hold on; grid on;
yLegend = cell(length(Y), 1);
set(gca,'ColorOrderIndex',1)
ayFig = figure; hold on; grid on;
for n = 1:length(Y)
    figure(yFig)
    [synchYr, Yt] = synchronize(Yr, Y{n}, 'union');
    plot(synchYr - Yt, 'LineWidth', 2)
    yLegend{n} = num2str(gainMultipliers(n));
    figure(ayFig)
    plot(ay{n}, 'LineWidth', 2)
end
figure(yFig);
ylabel("$e_Y (m)$", 'Interpreter','latex');
xlabel("$t (s)$", 'Interpreter','latex');
set(gca,'fontsize', 15)
lgd = legend(yLegend, "FontSize", fontSize ,'Interpreter','latex', ...
    'AutoUpdate','off');
title(lgd, 'm');
% plot(Yr, '--k', 'LineWidth', 2);

figure(ayFig);
ylabel("$a_y (m/s^2)$", 'Interpreter','latex');
xlabel("$t (s)$", 'Interpreter','latex');
set(gca,'fontsize', 15)
lgd = legend(yLegend, "FontSize", fontSize,'Interpreter','latex');
title(lgd, 'm');

figHandle = [yFig, ayFig];
figName = {'lateral_controller_tuning_errorY', ...
    'lateral_controller_tuning_ay'};
folderPath = 'G:\My Drive\PhD Research\Lane Change\images\';
for iFig = 1:length(figHandle)
    saveas(figHandle(iFig), ['.\..\figures\' figName{iFig}]);
    print(figHandle(iFig), [folderPath figName{iFig}], ...
        '-depsc', '-vector');
end