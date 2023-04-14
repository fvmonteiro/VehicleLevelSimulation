%%% Run chosen scenarios for ***platoon*** min lc spacing:
% Scenario 2: vehicles move individually as soon as gap is available
% starting by the leader

clearvars
close all
% saveFigs = 0;

warning('SCENARIO BASED ON WRONG DERIVATION - DO NOT USE')

%% Constants and parameters

g = 9.8; % [m/s^2]
global mpsTokph
mpsTokph = 3600/1000; % m/s to km per hour
ts = 0.01; % [s] sampling time
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:ts:totalTime; 

reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking
platoonReactionTime = 0.1; % [s] interplatoon reaction time
tLat = 5; % [s] lane change duration
tAdj = 0; % [s] long adjustment time before lane change starts
h = 3.6; % [m] lane width
nP = 3; % number of vehicles in the platoon

% Vehicle parameters
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

% Initial positions [m]
% Longitidinal (irrelevant in this simulation)
x0 = 0;
% Lateral
yP = 0; 
y(foIdx) = 0;
y(loIdx) = 0; 
y(fdIdx) = h;
y(ldIdx) = h;

% Types P -  passenger vehicle, B - bus, T - truck
% Longitudinal speeds depend on each scenario [m/s]
vP0 = 10:0.5:30; % platoon initial speeds

%% Run scenarios
scenarios = 2;
[typeEgo, typeOther, speedL, speedF, figName] = defineScenario(scenarios);

for nType = 1:length(typeEgo)
    % Create platoon
    platoon = Platoon(typeEgo, x0, yP, vP0, reactionTime, platoonReactionTime, nP);
    
%     platoon.leader.sinLatAccel(t, tAdj, tLat, h) % Merging vehicle lateral movement
%     platoon.leader.zeroAccel(t); % simple case when the ego veh keeps constant speed
    for nSpeed = 1:length(speedL) % scenario 2 contains different speeds for L and F
        vehs = Vehicle.empty(length(locVeh), 0);
        vehFollMSS = zeros(length(locVeh), length(vP0));
        virtualLaneChangeMSS = zeros(length(locVeh), length(vP0));
        for nVeh = 1:length(locVeh)
            otherVehLoc = char(locVeh(nVeh));
            if otherVehLoc(1) == 'F'
                v0 = speedF(nSpeed);
                egoVeh = platoon.last;
            else
                v0 = speedL(nSpeed);
                egoVeh = platoon.leader;
            end
            % Relevant platoon vehicle movement
            egoVeh.sinLatAccel(t, tAdj, tLat, h) % Merging vehicle lateral movement
            egoVeh.zeroAccel(t); % simple case when the ego veh keeps constant speed
            
            % Create vehicle
            vehs(nVeh) = Vehicle(typeOther(), locVeh(nVeh), x0, y(nVeh), v0, reactionTime);
            
            % Minimum safe vehicle following distance
            vehFollMSS(nVeh, :) = analyticalFollowingMSS(egoVeh, vehs(nVeh));
            
            % Assuming const speeds during lane change
            vehs(nVeh).zeroAccel(t);
            % Safe lane change distance for constant speeds
            tEndLaneChange = tAdj+tLat;
            virtualLaneChangeMSS(nVeh, :) = computeLaneChangeMSS(egoVeh, vehs(nVeh), t, tEndLaneChange);
        end
        
        % Total safe distance from each vehicle for safe lane change
        laneChangeMSS = vehFollMSS + virtualLaneChangeMSS;
        [hp, d0] = platoon.leader.safeHeadway();
        extraSpacing = (nP-1)*(hp*vP0+d0+platoon.leader.len)*(vehs(fdIdx).v0-vehs(loIdx).v0)./(vP0 - vehs(fdIdx).v0);
        laneChangeMSS(loIdx, :) = laneChangeMSS(loIdx, :) + max(extraSpacing, 0);
        % Total min gap between vehicles for safe lane change
                      
        totalGapOplatoon = sum(laneChangeMSS([foIdx loIdx], :)) + platoon.leader.len;
        totalGapDplatoon = sum(laneChangeMSS([fdIdx ldIdx], :)) + platoon.leader.len;
        
        figOrig = figure;
        hold on, grid on;
        plot(vP0*mpsTokph, totalGapOplatoon);
%         legend(legCell)
        title('original lane')
        xlabel('v_{ego} [km/h]');
        ylabel('min gap [m]');
        figDest = figure;
        hold on, grid on;
        plot(vP0*mpsTokph, totalGapDplatoon);
%         legend(legCell)
        title('destination lane')
        xlabel('v_{ego} [km/h]');
        ylabel('min gap [m]');
%         figDest = plotMinLCGap(platoon, laneChangeMSS([fdIdx ldIdx], :), 'd');
%         figOrig = plotMinLCGap(platoon, laneChangeMSS([foIdx loIdx], :), 'o');
        movegui(figDest, 'west')
        movegui(figOrig, 'east')
        
        % Save
        %     if (saveFigs)
        %         k = nSpeed + nType - 1; % CAUTION: only works for the current 3 scenarios
        %         mySavePlot(figDest,  ['min_lc_gap_d_' figName{k}])
        %         mySavePlot(figOrig,  ['min_lc_gap_o_' figName{k}])
        %     end
        
    end
end
%% Function to define different scenarios
function [typeEgo, typeOther, speedL, speedF, figName] = defineScenario(scenarioNumber)
global mpsTokph

switch scenarioNumber
    case 1
        typeEgo = "P";
        typeOther = "P";
        speedL = 70/mpsTokph; % [m/s]
        speedF = 70/mpsTokph; % [m/s]
        figName = {'equal_speeds'};
    case 2
        typeEgo = "P";
        typeOther = "P";
        speedL = [70/mpsTokph, 55/mpsTokph]; % [m/s]
        speedF = [55/mpsTokph, 70/mpsTokph]; % [m/s]
        figName = {'faster_leader', 'faster_follower'};
    case 3
        typeEgo = ["P", "T"];
        typeOther = ["T", "P"];
        speedL = 70/mpsTokph; % [m/s]
        speedF = 70/mpsTokph; % [m/s]
        figName = {'P_among_Ts', 'T_among_Ps'};
    otherwise
        error('Unknown scenario')
end
            
end
