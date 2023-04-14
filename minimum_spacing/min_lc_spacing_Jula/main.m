%% main file to run tests on the lane change paper from Jula

clearvars;
close all;

% Positions x and y are measured at front-left corner of the vehicle

%% Parameters

% Constants and time parameters
totalTime = 50; % [s] total simulation time
ts = 0.1; % [s] sampling time
tLat = 5; % [s] lane change duration
vehWidth = 1.5; % [m] vehicle width
vehLen = 5; % [m] vehicle length
feetToMeter = 1/3.281;
h = 12*feetToMeter; % [m] lane width

t = 0:ts:totalTime;

% Lateral positions
yLd = h; % [m] leader in destination lane
yFd = h; % [m] follower in destination lane
yLo = 0; % [m] leader in origin lane
yFo = 0; % [m] follower in origin lane

% Longitudinal acceleration
aLd = zeros(length(t), 1); % [m/s] Ld vehicle
aFd = zeros(length(t), 1); % [m/s] Fd vehicle
aLo = zeros(length(t), 1); % [m/s] Lo vehicle
aFo = zeros(length(t), 1); % [m/s] Fo vehicle

% Create vehicle objects
ld = Vehicle('Ld', vehWidth, vehLen, yLd, [], aLd);
fd = Vehicle('Fd', vehWidth, vehLen,  yFd, [], aFd);
lo = Vehicle('Lo', vehWidth, vehLen,  yLo, [], aLo);
fo = Vehicle('Fo', vehWidth, vehLen,  yFo, [], aFo);
vehicles = [ld, fd, lo, fo];

mergingVeh = Vehicle('M', vehWidth, vehLen, 0, [], []);


%% Run Scenarios
% Main adjustable variables:
% tAdj[s]: time for longitudinal adjustment before lane change starts
% tLong[s]: time, after lane change starts, to achieve speed of vehicles on
% adjacent lanes
% Vehicles relative speeds [m/s]
% am[m/s^2]: merging vehicle longitudinal acceleration

%% Const speeds
relVelRange = [-20, 10]; % [m/s] range of relative velocities between 
% surrounding vehicles and the merging vehicle
[~, fig1] = runScenario1(t, tLat, h, relVelRange, vehicles, mergingVeh);
% saveBigPlot(fig1, 'jula_mss_const_speed');

%% Constant acceleration during lane change to match destination lane speed
% (no speed adjustment before lane change)
relVelRange = [-20, 10]; % [m/s] range of relative velocities between 
% surrounding vehicles and the merging vehicle
deltaDO = [-10,10]; % [m/s] difference of speed (Fd, Ld) and (Fo, Lo);
[~, fig2] = runScenario2(t, tLat, h, relVelRange, vehicles, mergingVeh, deltaDO);
% saveBigPlot(fig2, 'jula_mss_switch_accel');

%% Longitudinal adjusmtent before and during lane change
relVelRange = [-20, 10]; % [m/s] range of relative velocities between 
% surrounding vehicles and the merging vehicle
aAdj = 1:2:7; % [m/s^2] magnitude of acceleration - positive or negative depending on scenario
initGap = 20; % [m] initial distance between M and other vehicle
initRelVel = 10; % [m/s] initial relative velocity between M and other vehicle
fig3a = runScenario3(t, tLat, h, relVelRange, vehicles(1:2), mergingVeh, initGap, initRelVel, aAdj);
% saveBigPlot(figs3a, ['jula_mss_speed_adj_' vehicles(k).name]);

initGap = 10; % [m/s] initial distance between M and other vehicle
fig3b = runScenario3(t, tLat, h, relVelRange, vehicles(3:4), mergingVeh, initGap, initRelVel, aAdj);


%% Longitudinal adjusmtent with coasting before and during lane change
relVelRange = [-20, 10]; % [m/s] range of relative velocities between 
% surrounding vehicles and the merging vehicle
aAdj = 1; % [m/s^2] magnitude of acceleration - positive or negative depending on scenario
initGap = 20; % [m] initial distance between M and other vehicle
initRelVel = 10; % [m/s] initial relative velocity between M and other vehicle
ta = initRelVel/aAdj*1.2; % [s] time to begin coasting (accel = 0). It has 
% to be at least equal to the time for the relative speed to be negative
fig4 = runScenario4(t, tLat, ta, h, relVelRange, vehicles(1:2), mergingVeh, initGap, initRelVel, aAdj);
saveBigPlot(fig4(k), ['jula_mss_speed_adj_coast_' vehicles(k).name]);


%% Plots
% scenarioNames = {'const velocity', 'switching accel'};
% 
% for n = 1:length(scenarioNames)
%     figure;
%     sgtitle(['MSS: ' scenarioNames{n}])
%     for k = 1:length(vehicles)
%         
%         vOtherVeh = vehicles(k).v0;
%         nameOtherVeh = vehicles(k).name;
%         
%         switch nameOtherVeh(1)
%             case 'L'
%                 txt = 'vm - vl';
%                 deltaV = mergingVeh.v0 - vOtherVeh;
%             case 'F'
%                 txt = 'vf - vm';
%                 deltaV = vOtherVeh - mergingVeh.v0;
%             otherwise
%         end
%         
%         subplot(2, 2, k);
%         plot(deltaV, MSS(n).(nameOtherVeh));
%         grid on;
%         xlabel(['Rel long speed [m/s]: ' txt]);
%         ylabel('MSS [m]');
%         title(['M and ' nameOtherVeh])
%     end
% end

%% Merging vehicle's angle (still not sure where to use this)
tAdj = 0;
vm = 10:25;
aLat = sinLatAccel(t, tAdj, tLat, h);
vLat = cumtrapz(ts, aLat);
maxTheta = max(atan2(vLat, vm)); % my approach
% maxTheta = atan2(vLat(t==(tAdj+tcLd)), vm0); % paper's approach
l1 = vehLen + vehWidth*sin(maxTheta);

%% Some plots to help analysis
% vLat = cumtrapz(t, aLat);
% y = cumtrapz(ts, vLat);
% figure, hold on, grid on;
% plot(t, aLat);
% figure, hold on, grid on;
% plot(t, vLat);
% figure, hold on, grid on;
% plot(t, y); plot(t, (yLd-vehWidth)*ones(length(t), 1))