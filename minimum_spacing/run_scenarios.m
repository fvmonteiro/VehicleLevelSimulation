%%% Run chosen scenarios for min lc spacing:
% Scenario 1: checking relative speeds effects
% - Same type of vehicle
% - All vehicles of the same type and all except M at the same speed
% - Constant speed lane change
% Scenario 2: checking relative speeds effects
% - Same type of vehicle
% - Vehicles in same lane have different speeds
% - Constant speed lane change
% Scenario 3: different vehicle types
% - All vehicles at 20m/s
% - Constant speed lane change

clearvars
close all
saveFigs = 0;

%% Constants and parameters

load_parameters;
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking

% Vehicle parameters
% Names:
% ego - merging vehicle
% Ld - leader in destination lane
% Fd - follower in destination lane
% Lo - leader in original lane
% Fo - follower in original lane

% Locations
locVeh(vehIdx.fo) = "Fo";
locVeh(vehIdx.lo) = "Lo";
locVeh(vehIdx.fd) = "Fd";
locVeh(vehIdx.ld) = "Ld";

% Initial positions [m]
% Longitidinal (irrelevant in this simulation)
x0 = 0;
% Lateral
yEgo = 0; 
y(vehIdx.fo) = 0;
y(vehIdx.lo) = 0;
y(vehIdx.fd) = laneWidth;
y(vehIdx.ld) = laneWidth;

% Types (P -  passenger vehicle, B - bus, T - truck) and longitudinal 
% speeds [m/s] depend on each scenario 
vEgo0 = 10:0.5:30; % ego vehicle tested speeds

%% Run scenarios
scenarios = 3;
[typeEgo, typeOther, speedL, speedF, figName] = defineScenario(scenarios);

for nType = 1:length(typeEgo)
    % Create and define ego vehicle movements
    egoVeh = Vehicle(typeEgo(nType), 'ego', x0, yEgo, vEgo0, reactionTime);
    tAdj = 0; % long adjustment time before lane change starts
    egoVeh.sinLatAccel(t, tAdj, tLat, laneWidth) % Merging vehicle lateral movement
    egoVeh.zeroAccel(t); % simple case when the ego veh keeps constant speed
    for nSpeed = 1:length(speedL) % scenario 2 contains different speeds for L and F
        vehs = Vehicle.empty(length(locVeh), 0);
        vehFollMSS = zeros(length(locVeh), length(vEgo0));
        for nVeh = 1:length(locVeh)
            charLoc = char(locVeh(nVeh));
            if charLoc(1) == 'F'
                v0 = speedF(nSpeed);
            else
                v0 = speedL(nSpeed);
            end
            % Create vehicle
            vehs(nVeh) = Vehicle(typeOther(nType), locVeh(nVeh), x0, y(nVeh), v0, reactionTime);
            
            % Minimum safe vehicle following distance
            vehFollMSS(nVeh, :) = egoVeh.computeFollowingMSS(t, vehs(nVeh));
%             vehFollMSS(nVeh, :) = analyticalFollowingMSS(egoVeh, vehs(nVeh));
            
            % Assuming const speeds during lane change
            vehs(nVeh).zeroAccel(t);
        end
        
        % Safe lane change distance for constant speeds
        tEndLaneChange = tAdj+tLat;
        virtualLaneChangeMSS = computeLaneChangeMSS(egoVeh, vehs, t, tEndLaneChange);
        
        % Total lane change safe distance
        laneChangeMSS = vehFollMSS + virtualLaneChangeMSS;
        
        %%% TEST for time headway
%         laneChangeMSS(fdIdx, :) = laneChangeMSS(fdIdx, :)/vehs(fdIdx).v0;
%         laneChangeMSS(foIdx, :) = laneChangeMSS(foIdx, :)/vehs(foIdx).v0;
%         laneChangeMSS(ldIdx, :) = laneChangeMSS(ldIdx, :)./vEgo0;
%         laneChangeMSS(loIdx, :) = laneChangeMSS(loIdx, :)./vEgo0;
        
        figDest = plotMinLCGap(egoVeh, laneChangeMSS([vehIdx.fd vehIdx.ld], :), 'd');
        figOrig = plotMinLCGap(egoVeh, laneChangeMSS([vehIdx.fo vehIdx.lo], :), 'o');
        movegui(figDest, 'west')
        movegui(figOrig, 'east')
        
        % Save
        if (saveFigs)
            k = nSpeed + nType - 1; % CAUTION: only works for the current 3 scenarios
            mySavePlot(figDest,  ['min_lc_gap_d_' figName{k}])
            mySavePlot(figOrig,  ['min_lc_gap_o_' figName{k}])
        end
        
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
