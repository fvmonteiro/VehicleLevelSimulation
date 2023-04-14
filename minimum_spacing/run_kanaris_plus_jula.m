%% We follow Kanaris for MSS in vehicle following then add this to Jula's MSS for lane change

error('Deprecated code - run my_lc_spacing instead')

%% Constants
clearvars
close all

g = 9.8; % [m/s^2]
ts = 0.01; % [s] sampling time

%% Parameters
vehWidth = 1.5; % [m] vehicle width
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking
tLat = 5; % [s] lane change duration
h = 3.6; % [m] lane width
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:ts:totalTime; 

%% Vehicles
% Names:
% M - merging vehicle
% Ld - leader in destination lane
% Fd - follower in destination lane
% Lo - leader in original lane
% Fo - follower in original lane
 
% Types: P -  passenger vehicle, B - bus, T - truck
typeM = 'P';
typeLd = 'P';
typeFd = 'P';
typeLo = 'P';
typeFo = 'P';

% Initial conditions
% Longitidinal positions [m] (irrelevant in this simulation)
x0 = 0;

% Lateral positions [m]
yM = 0; 
yLd = h;
yFd = h;
yLo = 0; 
yFo = 0;

% Longitudinal speeds [m/s]
speedRangeM = 10:0.5:30;
speedRangeOthers = 10:10:30;
vM0 =  speedRangeM; %
vLd0 = 25; %
vFd0 = 25;
vLo0 = 20;
vFo0 = 20;

% Longitudinal accelerations [m/s^2]
aM0 = 0;
aLd0 = 0;
aFd0 = 0;
aLo0 = 0;
aFo0 = 0;

mergingVeh = Vehicle(typeM, 'M', vehWidth, x0, yM, vM0, reactionTime);
ld = Vehicle(typeLd, 'Ld', vehWidth, x0, yLd, vLd0, reactionTime);
fd = Vehicle(typeFd, 'Fd', vehWidth, x0, yFd, vFd0, reactionTime);
lo = Vehicle(typeLo, 'Lo', vehWidth, x0, yLo, vLo0, reactionTime);
fo = Vehicle(typeFo, 'Fo', vehWidth, x0, yFo, vFo0, reactionTime);
vehicles = [ld, fd, lo, fo];

%% Vehicle following MSS
td = 0.2; 

vehFollMSS.Ld = computeFollowingMSS(t, mergingVeh, ld, td);
vehFollMSS.Fd = computeFollowingMSS(t, mergingVeh, fd, td);
vehFollMSS.Lo = computeFollowingMSS(t, mergingVeh, lo, td);
vehFollMSS.Fo = computeFollowingMSS(t, mergingVeh, fo, td);

% gapD = vehFollMSS.Ld + vehFollMSS.Fd + mergingVeh.len;
% figure, grid on, hold on;
% plot(vM0, vehFollMSS.Ld);
% plot(vM0, vehFollMSS.Fd);
% plot(vM0, gapD);
% legend('to leader', 'to foll', 'total')
% title('destination lane')
% xlabel('v_M [m/s]');
% ylabel('min gap [m]');
% 
% gapO = vehFollMSS.Lo + vehFollMSS.Fo + mergingVeh.len;
% figure, grid on, hold on;
% plot(vM0, vehFollMSS.Lo);
% plot(vM0, vehFollMSS.Fo);
% plot(vM0, gapO);
% legend('to leader', 'to foll', 'total')
% title('original lane')
% xlabel('v_M [m/s]');
% ylabel('min gap [m]');
%% Lane change MSS to virtual vehicles

mergingVeh.zeroAccel(t);
ld.zeroAccel(t);
fd.zeroAccel(t);
lo.zeroAccel(t);
fo.zeroAccel(t);
% Merging vehicle lateral movement
tAdj = 0;
mergingVeh.sinLatAccel(t, tAdj, tLat, h)

% tEndLaneChange = tAdj+tLat;
% Simplest scenario: all constant speeds
% virtualLaneChangeMSS = computeLaneChangeMSS(mergingVeh, vehicles, t, t(end));

% Const acceleration to match Ld's speed
tLong = 10;
aM = zeros(length(t), length(mergingVeh.v0));
for n = 1:length(mergingVeh.v0)
    aM(t<=tLong, n) = (ld.v0 - mergingVeh.v0(n))/tLong;
end
mergingVeh.at = aM;
virtualLaneChangeMSS = computeLaneChangeMSS(mergingVeh, vehicles, t, t(end));

%% Lane change MSS
for k = 1:length(vehicles)
    currentVeh = vehicles(k).location;
    laneChangeMSS.(currentVeh) = vehFollMSS.(currentVeh) + ...
        virtualLaneChangeMSS.(currentVeh);
end
gapD = laneChangeMSS.Ld + laneChangeMSS.Fd + mergingVeh.len;
figure, grid on, hold on;
plot(vM0, laneChangeMSS.Ld);
plot(vM0, laneChangeMSS.Fd);
plot(vM0, gapD);
legend('to leader', 'to foll', 'total')
title('destination lane')
xlabel('v_M [m/s]');
ylabel('min gap [m]');

gapO = laneChangeMSS.Lo + laneChangeMSS.Fo + mergingVeh.len;
figure, grid on, hold on;
plot(vM0, laneChangeMSS.Lo);
plot(vM0, laneChangeMSS.Fo);
plot(vM0, gapO);
legend('to leader', 'to foll', 'total')
title('original lane')
xlabel('v_M [m/s]');
ylabel('min gap [m]');




