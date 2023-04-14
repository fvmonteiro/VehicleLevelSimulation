%%%

clearvars
close all
saveFigs = 1;

%% Parameters
load_parameters
deltaT = 0.001; % [s] sampling time (needs to be smaller in this scenario)
t = 0:deltaT:totalTime; 

reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking

% Locations
locVeh(vehIdx.fo) = "Fo";
locVeh(vehIdx.lo) = "Lo";
locVeh(vehIdx.fd) = "Fd";
locVeh(vehIdx.ld) = "Ld";

% Initial lateral positions [m]
yEgo = 0;
y(vehIdx.fo) = 0;
y(vehIdx.lo) = 0;
y(vehIdx.fd) = laneWidth;
y(vehIdx.ld) = laneWidth;

% Types (P -  passenger vehicle, B - bus, T - truck) and longitudinal
typeEgo = "P";
typeOther = "P";

% Initial speeds [m/s]
v0Ego = 10:0.5:30; % ego vehicle tested speeds
v0(vehIdx.fo) = 0; % irrelevant in this scenario
v0(vehIdx.lo) = 0; % lo init speed is defined later as equal to v0Ego
vOtherLane = 20;
v0(vehIdx.fd) = vOtherLane;
v0(vehIdx.ld) = vOtherLane;

% Initial longitidinal positions [m] (only relevant for Lo)
x0Ego = 0;
x0(vehIdx.fo) = 0; 
x0(vehIdx.lo) = 0; % this is defined later for each value of v0Ego
x0(vehIdx.fd) = 0;
x0(vehIdx.ld) = 0; 

figName = 'speed_adj';
%% Run scenario

% Create ego vehicle
egoVeh = Vehicle(typeEgo, 'ego', x0Ego, yEgo, v0Ego, reactionTime);

% Create other vehicles
vehs = Vehicle.empty(length(locVeh), 0);
for n = 1:length(locVeh)
    otherLoc = char(locVeh(n));
    % Create vehicle
    vehs(n) = Vehicle(typeOther, locVeh(n), x0(n), y(n), v0(n), reactionTime);
%     vehs(n).x0 = vehs(n)
    % Assuming const speeds during lane change
    vehs(n).zeroAccel(t);
end

% Speed adjustment lane chage
tAdj = egoVeh.longVelAdjustment(t, vehs(vehIdx.ld), vehs(vehIdx.lo), tLat, laneWidth);
egoVel = egoVeh.computeVel(t);

% Ego vehicle lateral movement
egoVeh.sinLatAccel(t, tAdj, tLat, laneWidth)
tEndLaneChange = tAdj+tLat;

vehFollMSS = zeros(length(locVeh), length(v0Ego));

for n = 1:length(locVeh)
    % Minimum safe vehicle following distance
    otherVehLoc = char(vehs(n).location);
    if otherVehLoc(2) == 'd'
        for k = 1:length(v0Ego)
            tIdx = find(abs(t-tEndLaneChange(k))<deltaT, 1);
            relevantVel = egoVel(tIdx, k);
            vehFollMSS(n, k) = egoVeh.computeFutureFollowingMSS(t, vehs(n), relevantVel);
        end
    else
        relevantVel = egoVel(1, :);
        vehFollMSS(n, :) = egoVeh.computeFutureFollowingMSS(t, vehs(n), relevantVel);
    end
end

% Safe lane change distance for constant speeds
% virtualLaneChangeMSS = zeros(length(locVeh), length(v0Ego));
% for k = 1:length(v0Ego)
virtualLaneChangeMSS = computeLaneChangeMSS(egoVeh, vehs, t, tEndLaneChange);
% end

% Total lane change safe distance
laneChangeMSS = vehFollMSS + virtualLaneChangeMSS;

% Plots
figHandle = figure; hold on; grid on;
figLeg = cell(length(locVeh)-2, 1); % we're not plotting Fo or Lo
counter = 1;
for n = 1:length(locVeh)
    if n~=vehIdx.fo && n~=vehIdx.lo
        plot((egoVeh.v0-vOtherLane)*mpsTokph, laneChangeMSS(n, :), 'LineWidth', 1.5)
        figLeg{counter} = vehs(n).location;
        counter = counter+1;
    end
end
legend(figLeg)
xlabel('v_{E} [km/h]');
ylabel('g_{safe}(E, i) [m]');

velFig = figure;
hold on, grid on;
plot(t(t<=max(tAdj)+tLat), egoVel(t<=max(tAdj)+tLat, 1:end)*mpsTokph, 'LineWidth', 1.5)
xlabel('t [s]')
ylabel('v_{E} [km/h]')

% Save
if (saveFigs)
    mySavePlot(figHandle,  ['min_lc_gaps_' figName], 'png')
    mySavePlot(velFig,  'speed_adj_during_lc', 'png')
end
        
