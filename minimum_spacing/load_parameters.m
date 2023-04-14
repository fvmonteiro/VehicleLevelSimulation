% Constants and parameters

% g = 9.8; % [m/s^2]
deltaT = 0.01; % [s] sampling time
deltaS = 0.1; % [m] sampling space
global mpsTokph % m/s to km per hour
mpsTokph = 3600/1000; 

% Scenario
laneWidth = 3.6; %[m]
nLanes = 2;
scenarioWidth = nLanes*laneWidth;
scenarioLength = 100; % [m]
longSpace = 0:deltaS:scenarioLength;
latSpace = 0:deltaS:scenarioWidth;

reactionTime = 0.2;
% platoonReactionTime = 0.1; % [s] interplatoon reaction time
tLat = 5; % [s] lane change duration
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:deltaT:totalTime; 

% Vehicle indices
vehIdx.fo = 1;
vehIdx.lo = 2;
vehIdx.fd = 3;
vehIdx.ld = 4;

