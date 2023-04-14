%%% Crossing time as function of braking time %%%

%% Constants
clearvars
close all

% g = 9.8; % [m/s^2]
deltaT = 0.01; % [s] sampling time
deltaS = 0.1; % [m] sampling space
global mpsTokph % m/s to mile per hour
mpsTokph = 3600/1000; 

%% Scenario
laneWidth = 3.6; %[m]
nLanes = 2;
scenarioWidth = nLanes*laneWidth;
scenarioLength = 100; % [m]
longSpace = 0:deltaS:scenarioLength;
latSpace = 0:deltaS:scenarioWidth;

%% Parameters
vehWidth = 1.8; % [m] vehicle width
tLat = 5; % [s] lane change duration
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:deltaT:totalTime; 

%% Vehicles
% Names:
% F - follower
% L - leader
% d - destination lane
% o - original lane

% Locations
locVeh(1) = "Fo";
locVeh(2) = "Lo";
locVeh(3) = "Fd";
locVeh(4) = "Ld";

% Types: P -  passenger vehicle, B - bus, T - truck
typeEgo = "P";
typeVeh(1) = "P";
% typeVeh(2) = "T";
% typeVeh(3) = "T";
% typeVeh(4) = "P";

% Initial conditions 
% Longitudinal positions [m]
xEgo = 50;
xVeh(1) = 30;
xVeh(2) = 90;
xVeh(3) = 10;
xVeh(4) = 90;

% Lateral positions [m]
yEgo = 0.5*laneWidth;
yVeh(1) = 0.5*laneWidth;
yVeh(2) = 0.5*laneWidth;
yVeh(3) = 1.5*laneWidth;
yVeh(4) = 1.5*laneWidth;

% Longitudinal speeds [m/s]
% speedRange = 10:0.2:30;
vEgo0 = 20;
v0Veh(1) =  22;
v0Veh(2) =  18;
v0Veh(3) =  20;
v0Veh(4) =  22;

ego = Vehicle(typeEgo, 'ego', vehWidth, xEgo, yEgo, vEgo0);

vehs = Vehicle.empty(length(typeVeh), 0);
for k = 1:length(typeVeh)
    vehs(k) = Vehicle(typeVeh(k), locVeh(k), vehWidth, xVeh(k), yVeh(k), v0Veh(k));
end

%% 

td = 0.1; % braking delay
% Ego vehilce lateral movement
ego.sinLatAccel(t, 0, tLat, laneWidth)

% First only Fo
fo = vehs(1);
ego.zeroAccel(t);
tcOriginal = crossingTime(t, ego, fo);
ego.brakeFullStop(t, td, fo);
tToFullStop = t(find(ego.at==0, 1));


deltaT0 = 0.5; % [s]
nT0 = ceil(tcOriginal/deltaT0);
tc = zeros(nT0, 1);
% figure; hold on;
for i = 1:nT0
    tc(i) = crossingTime(t, ego, fo);
    ego.delayAccel(t, deltaT0);
end
plot(0:deltaT0:tcOriginal, tc)



