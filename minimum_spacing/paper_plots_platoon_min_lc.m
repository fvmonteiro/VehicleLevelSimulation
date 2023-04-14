%%% Note: most of this code is equal to "paper_plots_min_lc_gap". Under the
%%% assumption that platoon vehicles are homogeneous, we can compute the
%%% gaps from p1 (platoon leader) to Lo and Ld and from pN (last platoon
%%% vehicle) to Fd in the same way we computed gaps for single vehicles.
%%% The difference here is that we'll add the platoon length to the total
%%% necessary gap at the adjacent lane. 


clearvars
close all

%% Parameters
load_parameters
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking
interPlatoonReactionTime = 0.1;

% Locations
locVeh(vehIdx.fo) = "Fo";
locVeh(vehIdx.lo) = "Lo";
locVeh(vehIdx.fd) = "Fd";
locVeh(vehIdx.ld) = "Ld";

% Initial positions [m]
% Longitidinal (irrelevant in this simulation)
x0 = 0;
% Lateral
yPlatoon = 0;
y(vehIdx.fo) = 0;
y(vehIdx.lo) = 0;
y(vehIdx.fd) = laneWidth;
y(vehIdx.ld) = laneWidth;



% Initial speeds [m/s]
% v0Other = 10:0.5:30;
% v0P = 20; %10:0.5:30; % ego vehicle tested speeds
maxN = 10;

%% Run scenario

%%% In order to run all scenarios in the same loop, the indexing of some
%%% arrays got quite confusing. Probably better to rearrange in the future
%%% (Oct 22, 2019)

scenarios = [1, 2, 3];

for nS = scenarios
[typePlatoon, typeOther, speedP, speedOther, vehiclesInPlatoon, figName, figLeg] = defineScenario(nS);

nVel = max(length(speedP), length(speedOther));

% figHandle = cell(max(length(typePlatoon), length(speedP)), 1);

gapAdjLane = zeros(max([length(vehiclesInPlatoon), length(speedP), length(typePlatoon)]), nVel);

for nType = 1:length(typePlatoon)
    for nVP = 1:length(speedP)
    
        % Create ego vehicle
        platoonVeh = Vehicle(typePlatoon(nType), 'ego', x0, yPlatoon, speedP(nVP), reactionTime);
        % Ego vehicle lateral movement
        tAdj = 0; % long adjustment time before lane change starts
        platoonVeh.sinLatAccel(t, tAdj, tLat, laneWidth)
    
        % Create other vehicles
        vehs = Vehicle.empty(length(locVeh), 0);
        for nP = 1:length(locVeh)
            % Create vehicle
            vehs(nP) = Vehicle(typeOther(nType), locVeh(nP), x0, y(nP), speedOther, reactionTime);
            % Assuming const speeds during lane change
            vehs(nP).zeroAccel(t);
        end
        
        platoonVeh.zeroAccel(t);       
        egoVel = platoonVeh.computeVel(t);
        
        vehFollMSS = zeros(length(locVeh), nVel);
        for nP = 1:length(locVeh)
            % Minimum safe vehicle following distance
            otherVehLoc = char(vehs(nP).location);
            if otherVehLoc(2) == 'd'
                relevantVel = egoVel(t==tLat, :);
            else
                relevantVel = egoVel(1, :);
            end
            vehFollMSS(nP, :) = platoonVeh.computeFutureFollowingMSS(t, vehs(nP), relevantVel);
            %     vehFollMSS(n, :) = egoVeh.computeFollowingMSS(t, vehs(n));
            %     vehFollMSS(nVeh, :) = analyticalFollowingMSS(egoVeh, vehs(nVeh));
        end
        
        % Safe lane change distance for constant speeds
        tEndLaneChange = tAdj+tLat;
        virtualLaneChangeMSS = computeLaneChangeMSS(platoonVeh, vehs, t, tEndLaneChange);
        
        % Total lane change safe distance for each vehicle
        laneChangeMSS = vehFollMSS + virtualLaneChangeMSS;
        
        % Necessary gap on the adjacent lane
        for nP = 1:length(vehiclesInPlatoon)
            platoon = Platoon(typePlatoon(nType), x0, yPlatoon, speedP(nVP), reactionTime, interPlatoonReactionTime, vehiclesInPlatoon(nP)); 
            gapAdjLane(nType+nVP+nP-2, :) = sum(laneChangeMSS([vehIdx.fd, vehIdx.ld], :)) + platoon.len;
        end   
        

    end

end

figHandle = figure; hold on; grid on;
for n = 1:max([length(vehiclesInPlatoon), length(speedP), length(typePlatoon)])
%   plot((platoonVeh.v0-speedOther)*mpsTokph, gapAdjLane(n, :), 'LineWidth', 1.5)
    plot((speedOther)*mpsTokph, gapAdjLane(n, :), 'LineWidth', 1.5)
end
legend(figLeg, 'Location', 'best')
xlabel('v_{dest}(0) [km/h]');
ylabel('g_{adj lane} [m]');
    
    %%% UNCOMMENT TO SAVE %%%%
    % CAUTION: only works for the current 2 scenarios
%     mySavePlot(figHandle,  ['min_lc_platoon_gaps_' figName])

end
%% Function to define different scenarios
function [typeEgo, typeOther, speedP, speedOther, vehsInPlatoon, figName, figLeg] = defineScenario(scenarioNumber)
% global mpsTokph

switch scenarioNumber
    case 1
        typeEgo = "P";
        typeOther = "P";
        speedP = 20; % [m/s]
        speedOther = 10:0.5:30; %20; % [m/s]
        vehsInPlatoon = 1:2:9;
        figName = 'by_N';
        figLeg = cell(length(vehsInPlatoon), 1);
        for n = 1:length(vehsInPlatoon)
            figLeg{n} = ['N = ' num2str(vehsInPlatoon(n))];
        end
    case 2
        typeEgo = "P";
        typeOther = "P";
        speedP = 15:5:25; % [m/s]
        speedOther = 10:0.5:30; %20; % [m/s]
        vehsInPlatoon = 5;
        figName = 'by_Vel';
        figLeg = cell(length(speedP), 1);
        for n = 1:length(speedP)
            figLeg{n} = ['v_P(0) = ' num2str(speedP(n)*3.6)];
        end
    case 3
        typeEgo = ["P", "T"];
        typeOther = ["T", "P"];
        speedP = 20; % [m/s]
        speedOther = 10:0.5:30; % [m/s]
        vehsInPlatoon = 5;
        figName = 'diff_types';
        figLeg = {'PV platoon between Trucks', 'Truck platoon between PVs'};
        
    case 4 % difference to case 1 is in the speed adjustment
        error('Scenario not used for platoons');
%         typeEgo = "P";
%         typeOther = "P";
%         speedL = 20; % [m/s]
%         speedF = 20; % [m/s]
%         figName = {'speed_adj_duringLC'};
    otherwise
        error('Unknown scenario')
end

end
