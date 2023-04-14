%% Constants
clearvars
close all

g = 9.8; % [m/s^2]
ts = 0.01; % [s] sampling time

%% Parameters
% vehWidth = 1.5; % [m] vehicle width
tLat = 5; % [s] lane change duration
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking
h = 3.6; % [m] lane width
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:ts:totalTime; 

%% Vehicles
% Names:
% ego - merging vehicle
% Ld - leader in destination lane
% Fd - follower in destination lane
% Lo - leader in original lane
% Fo - follower in original lane

% Vehicle indices
foIdx = 1;
loIdx = 2;
fdIdx = 3;
ldIdx = 4;

% Locations
locVeh(foIdx) = "Fo";
locVeh(loIdx) = "Lo";
locVeh(fdIdx) = "Fd";
locVeh(ldIdx) = "Ld";

% Types: P -  passenger vehicle, B - bus, T - truck
typeEgo = "P";
typeVeh(foIdx) = "P";
typeVeh(loIdx) = "P";
typeVeh(fdIdx) = "P";
typeVeh(ldIdx) = "P";

% Initial conditions
% Longitidinal positions [m] (irrelevant in this simulation)
x0 = 0;

% Lateral positions [m]
yEgo = 0; 
y(foIdx) = 0;
y(loIdx) = 0; 
y(fdIdx) = h;
y(ldIdx) = h;

% initial longitudinal speeds [m/s]
speedRangeEgo = 10:0.5:30;
vEgo0 =  speedRangeEgo; %
v0(foIdx) = 20;
v0(loIdx) = 15;
v0(fdIdx) = 15;
v0(ldIdx) = 20;

egoVeh = Vehicle(typeEgo, 'ego', x0, yEgo, vEgo0, reactionTime);
vehicles = Vehicle.empty(length(locVeh), 0);
for k = 1:length(locVeh)
    vehicles(k) = Vehicle(typeVeh(k), locVeh(k), x0, y(k), v0(k), reactionTime);
end
% ld = Vehicle(typeLd, 'Ld', vehWidth, x0, yLd, vLd0, reactionTime);
% fd = Vehicle(typeFd, 'Fd', vehWidth, x0, yFd, vFd0, reactionTime);
% lo = Vehicle(typeLo, 'Lo', vehWidth, x0, yLo, vLo0, reactionTime);
% fo = Vehicle(typeFo, 'Fo', vehWidth, x0, yFo, vFo0, reactionTime);
% vehicles = [ld, fd, lo, fo];

%% Vehicle following MSS
vehFollMSS = zeros(length(vehicles), length(speedRangeEgo));
for k = 1:length(vehicles)
    vehFollMSS(k, :) = analyticalFollowingMSS(egoVeh, vehicles(k));
end
% vehFollMSS.Ld = analyticalFollowingMSS(mergingVeh, ld);
% % comparisonMSS.Ld = computeFollowingMSS(t, mergingVeh, ld);
% vehFollMSS.Fd = analyticalFollowingMSS(mergingVeh, fd);
% vehFollMSS.Lo = analyticalFollowingMSS(mergingVeh, lo);
% vehFollMSS.Fo = analyticalFollowingMSS(mergingVeh, fo);

%% Lane change MSS to virtual vehicles

% Simplest scenario: all constant speeds

egoVeh.zeroAccel(t);
for k = 1:length(vehicles)
    vehicles(k).zeroAccel(t);
end
% ld.zeroAccel(t);
% fd.zeroAccel(t);
% lo.zeroAccel(t);
% fo.zeroAccel(t);
% Merging vehicle lateral movement
tAdj = 0;
egoVeh.sinLatAccel(t, tAdj, tLat, h)

tEndLaneChange = tAdj+tLat;
virtualLaneChangeMSS = computeLaneChangeMSS(egoVeh, vehicles, t, tAdj+tLat);

% Const acceleration to match Ld's speed
% tLong = 10;
% aM = zeros(length(t), length(mergingVeh.v0));
% for n = 1:length(mergingVeh.v0)
%     aM(t<=tLong, n) = (ld.v0 - mergingVeh.v0(n))/tLong;
% end
% mergingVeh.at = aM;
% virtualLaneChangeMSS = computeLaneChangeMSS(mergingVeh, vehicles, t, t(end));

%% Lane change MSS
% for k = 1:length(vehicles)
%     currentVeh = vehicles(k).location;
%     laneChangeMSS.(currentVeh) = vehFollMSS.(currentVeh) + ...
%         virtualLaneChangeMSS.(currentVeh);
% end
laneChangeMSS = vehFollMSS + virtualLaneChangeMSS;
% gapD = laneChangeMSS.Ld + laneChangeMSS.Fd + egoVeh.len;
gapD = laneChangeMSS(ldIdx, :) + laneChangeMSS(fdIdx, :) + egoVeh.len;
figure, grid on, hold on;
plot(vEgo0, laneChangeMSS(ldIdx, :));
plot(vEgo0, laneChangeMSS(fdIdx, :));
plot(vEgo0, gapD);
legend('to leader', 'to foll', 'total')
title('destination lane')
xlabel('v_M [m/s]');
ylabel('min gap [m]');

gapO = laneChangeMSS(loIdx, :) + laneChangeMSS(foIdx, :) + egoVeh.len;
figure, grid on, hold on;
plot(vEgo0, laneChangeMSS(loIdx, :));
plot(vEgo0, laneChangeMSS(foIdx, :));
plot(vEgo0, gapO);
legend('to leader', 'to foll', 'total')
title('original lane')
xlabel('v_M [m/s]');
ylabel('min gap [m]');




