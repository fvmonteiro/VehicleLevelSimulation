%%%

warning('Something off in this simulation: the lines shouldn''t all cross at delta v = 0');

clearvars
close all

%% Parameters
load_parameters
reactionTime = 0.3; % [s] time a vehicle takes to notice its leader is braking

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
% typeEgo = "P";
% typeOther(vehIdx.fo) = "P";
% typeOther(vehIdx.lo) = "P";
% typeOther(vehIdx.fd) = "P";
% typeOther(vehIdx.ld) = "P";

% Initial speeds [m/s]
% v0Other = 20;
v0Ego = 15:0.5:35; % ego vehicle tested speeds

% Min veh following spaces are computed with max brake during lane change
duringLaneChange = 0; 

%% Run scenario
scenarios = [1, 3, 4];

for nS = 1
[typeEgo, typeOther, speedL, speedF, figName] = defineScenario(nS);

for nType = 1:length(typeEgo)
    % Create ego vehicle
    egoVeh = Vehicle(typeEgo(nType), 'ego', x0, yEgo, v0Ego, reactionTime);
    % Ego vehicle lateral movement
    tAdj = 0; % long adjustment time before lane change starts
    egoVeh.sinLatAccel(t, tAdj, tLat, laneWidth)
    
    % Create other vehicles
    for nSpeed = 1:length(speedL) % scenario 2 contains different speeds for L and F
        vehs = Vehicle.empty(length(locVeh), 0);
        for n = 1:length(locVeh)
            % Create vehicle
            vehs(n) = Vehicle(typeOther(nType), locVeh(n), x0, y(n), 0, reactionTime);
            
            otherLoc = char(locVeh(n));
            if otherLoc(1) == 'F'
                vehs(n).v0 = speedF(nSpeed);
                vehs(n).maxBrake = 0.85*egoVeh.maxBrake;
            else
                vehs(n).v0 = speedL(nSpeed);
                vehs(n).maxBrake = 1.15*egoVeh.maxBrake;
            end
            
            % Assuming const speeds during lane change
            vehs(n).zeroAccel(t);
        end
        
        if nS==4
            egoVeh.longVelAdjustmentDuringLC(t, vehs(vehIdx.ld), tLat)
        else
            egoVeh.zeroAccel(t);
        end
        
        egoVel = egoVeh.computeVel(t);
        
        vehFollMSS = zeros(length(locVeh), length(v0Ego));
        for n = 1:length(locVeh)
            % Minimum safe vehicle following distance
            otherVehLoc = char(vehs(n).location);
            isDestinationLane = otherVehLoc(2) == 'd';
            if isDestinationLane
                relevantVel = egoVel(t==tLat, :);
            else
                relevantVel = egoVel(1, :);
            end
            
            vehFollMSS(n, :) = egoVeh.computeFutureFollowingMSS(t, vehs(n), relevantVel, duringLaneChange);
            %     vehFollMSS(n, :) = egoVeh.computeFollowingMSS(t, vehs(n));
            %     vehFollMSS(nVeh, :) = analyticalFollowingMSS(egoVeh, vehs(nVeh));
        end
        
        % Safe lane change distance for constant speeds
        tEndLaneChange = tAdj+tLat;
        virtualLaneChangeMSS = computeLaneChangeMSS(egoVeh, vehs, t, tEndLaneChange);
        
        % Total lane change safe distance
        laneChangeMSS = vehFollMSS + virtualLaneChangeMSS;
        
        % Plots
        
        %%% TEST for time headway
%         laneChangeMSS(vehIdx.fd, :) = laneChangeMSS(vehIdx.fd, :)/vehs(vehIdx.fd).v0;
%         laneChangeMSS(vehIdx.fo, :) = laneChangeMSS(vehIdx.fo, :)/vehs(vehIdx.fo).v0;
%         laneChangeMSS(vehIdx.ld, :) = laneChangeMSS(vehIdx.ld, :)./v0Ego;
%         laneChangeMSS(vehIdx.lo, :) = laneChangeMSS(vehIdx.lo, :)./v0Ego;
        
        figHandle = figure; hold on; grid on;
        figLeg = cell(length(locVeh)-1, 1); % we're not plotting Fo
        counter = 1;
        for n = 1:length(locVeh)
            if n~=vehIdx.fo
                plot((egoVeh.v0-vehs(n).v0)*mpsTokph, laneChangeMSS(n, :), 'LineWidth', 1.5)
                figLeg{counter} = vehs(n).location;
                counter = counter+1;
            end
        end
        legend(figLeg)
        xlabel('\Delta v(0) [km/h]');
        ylabel('g^{safe}_{E,k}(0) [m]');
              
        
        %%% UNCOMMENT TO SAVE %%%%
%         k = nSpeed + nType - 1; % CAUTION: only works for the current 3 scenarios
%         mySavePlot(figHandle,  ['min_lc_gaps_' figName{k}])
        
    end
end

end
%% Function to define different scenarios
function [typeEgo, typeOther, speedL, speedF, figName] = defineScenario(scenarioNumber)
% global mpsTokph

switch scenarioNumber
    case 1
        typeEgo = "P";
        typeOther = "P";
        speedL = 25; % [m/s]
        speedF = 25; % [m/s]
        figName = {'equal_speeds'};
    case 2
        typeEgo = "P";
        typeOther = "P";
        speedL = [25, 20]; % [m/s]
        speedF = [20, 25]; % [m/s]
        figName = {'faster_leader', 'faster_follower'};
    case 3
        typeEgo = ["P", "T"];
        typeOther = ["T", "P"];
        speedL = 25; % [m/s]
        speedF = 25; % [m/s]
        figName = {'P_among_Ts', 'T_among_Ps'};
    case 4 % difference to case 1 is in the speed adjustment
        typeEgo = "P";
        typeOther = "P";
        speedL = 25; % [m/s]
        speedF = 25; % [m/s]
        figName = {'speed_adj_duringLC'};
    otherwise
        error('Unknown scenario')
end

end
