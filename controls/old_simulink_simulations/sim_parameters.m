%% Simulation parameters
% Vehicle parameters (initially, the same for all)
vehMass = 2000;
lf = 1.4;
lr = 1.6;
vLen = lf+lr; % Vehicle length

% Longitudinal Params from Ioannou and Chien: Autonomous Intelligent Cruise Control
maxAcc = 4; %3; % [m/s^2]
minAcc = -8; %-3; % [m/s^2]
maxJerk = 40;%75; % (this one is not provided in the paper) [m/s^3]
minJerk = -75;%-75; %-4; % [m/s^3]
reactionTime = 0.1;
% Controller params
ehMax = 3; % max headway error
ehMin = -100; % min headway error

% Lateral parameters
% From: Path Planning and Cooperative Control for Automated Vehicle Platoon
% Using Hybrid Automata
maxDelta = 25*pi/180;
maxDeltaDot = 9.4*pi/180;
% From: Automated Lane Change Controller Design
maxLatAcc = 0.67;
maxLatJerk = 0.67;
% Default simulink values
Cf = 12e3;
Cr = 11e3;
Iz = 4000;

% ACC parameters
[h, g0] = safeHeadway(maxAcc, minAcc, minJerk, reactionTime);
h = ceil(h*10)*1.1/10; % give it some margin
g0 = ceil(g0*10)*1.1/10;

% Filters in ACC
p = 10; % velocity filter gain
c0 = 10; % gap filter gain

