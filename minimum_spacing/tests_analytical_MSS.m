%%%% Comparing solutions by numerical integration and analytical %%%%


%% Constants
clearvars
close all

g = 9.8; % [m/s^2]
ts = 0.01; % [s] sampling time

%% Parameters
tLat = 5; % [s] lane change duration
h = 3.6; % [m] lane width
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:ts:totalTime; 
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking

%% Vehicles
% Names:
% M - merging vehicle
% Ld - leader in destination lane
% Fd - follower in destination lane
% Lo - leader in original lane
% Fo - follower in original lane
 
% Types: P -  passenger vehicle, B - bus, T - truck
typeM = 'P';
typeLd = 'T';
typeFd = 'P';
typeLo = 'T';
typeFo = 'P';

% Initial conditions
% Longitudinal positions [m] (irrelevant for this test)
x0 = 0;

% Lateral positions [m]
yM = 0; 
yLd = h;
yFd = h;
yLo = 0; 
yFo = 0;

% Longitudinal speeds [m/s]
speedRangeM = 0:0.5:40;
speedRangeOthers = 10:10:30;
vM0 =  speedRangeM; %
vLd0 = 15; %
vFd0 = 20;
vLo0 = 20;
vFo0 = 25;



mergingVeh = Vehicle(typeM, 'M', x0, yM, vM0, reactionTime);
ld = Vehicle(typeLd, 'Ld', x0, yLd, vLd0, reactionTime);
fd = Vehicle(typeFd, 'Fd', x0, yFd, vFd0, reactionTime);
lo = Vehicle(typeLo, 'Lo', x0, yLo, vLo0, reactionTime);
fo = Vehicle(typeFo, 'Fo', x0, yFo, vFo0, reactionTime);
vehicles = [ld, fd, lo, fo];

%% Vehicle following MSS
td = 0.1; 

% Kanaris MSS
for k = 1:length(vehicles)
    loc = vehicles(k).location;
    kMSS.(loc) = computeFollowingMSS(t, mergingVeh, vehicles(k));
    [aMSS.(loc), specialIdx.(loc)] = analyticalFollowingMSS(mergingVeh, vehicles(k));
    MSSall.(loc) = [kMSS.(loc), aMSS.(loc)];
end
% kMSS.Ld = computeFollowingMSS(t, mergingVeh, ld, td);
% kMSS.Fd = computeFollowingMSS(t, mergingVeh, fd, td);
% kMSS.Lo = computeFollowingMSS(t, mergingVeh, lo, td);
% kMSS.Fo = computeFollowingMSS(t, mergingVeh, fo, td);


% Analytical MSS
% aMSS.Ld = analyticalFollowingMSS(mergingVeh, ld, td);
% aMSS.Fd = analyticalFollowingMSS(mergingVeh, fd, td);
% aMSS.Lo = analyticalFollowingMSS(mergingVeh, lo, td);
% aMSS.Fo = analyticalFollowingMSS(mergingVeh, fo, td);

%% Plots

% figure; hold on; grid on;
% plot(vM0, kMSS.Ld);
% plot(vM0, aMSS.Ld);
 
% plotMSSstruct(kMSS, vM0)
% plotMSSstruct(aMSS, vM0)
plotMSSstruct(MSSall, vM0, specialIdx)

% gapD = aMSS.Ld + aMSS.Fd + mergingVeh.len;
% figure, grid on, hold on;
% plot(vM0, aMSS.Ld);
% plot(vM0, aMSS.Fd);
% plot(vM0, gapD);
% legend('to leader', 'to foll', 'total')
% title('destination lane')
% xlabel('v_M [m/s]');
% ylabel('min gap [m]');
% 
% gapO = aMSS.Lo + aMSS.Fo + mergingVeh.len;
% figure, grid on, hold on;
% plot(vM0, aMSS.Lo);
% plot(vM0, aMSS.Fo);
% plot(vM0, gapO);
% legend('to leader', 'to foll', 'total')
% title('original lane')
% xlabel('v_M [m/s]');
% ylabel('min gap [m]');

