%%%% Code to plot min lc distance vs relative speed %%%%

%%% Tested on May 8, 2020 %%%
%%% Results don't match expectations - probably some wrong parameter
%%% setting after updating classes but not updating this code.


clearvars
close all


%% Parameters

% Some constants
deltaT = 0.01; % [s] sampling time
mps2kph = 3.6; % m/s to km per hour

% Scenario
laneWidth = 3.6; %[m]
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:deltaT:totalTime; 

% 
reactionTime = 0.3; % [s] time to start emergency braking
tLat = 5; % [s] lane change duration



%% Create vehicles

vehLoc = {'Lo', 'Fd', 'Ld', ... Fo
    };
%     'Fr', 'Lr'};
nVeh = length(vehLoc);

for iterVeh = 1:nVeh
    vehIdx.(vehLoc{iterVeh}) = iterVeh;   
end

% Ego vehicle
type = 'P';
x0Ego = 0;
y0Ego = 0;
v0 = 25; %[m/s]

% Test speed variation
deltaV = 10;

% Variations in max brake
gammaL = 1.15;
gammaF = 0.85;

egoVeh = VehicleRiskMap(type, 'ego', [x0Ego, y0Ego, v0], reactionTime);

% For now, x0 and y0 of ego are zero
v0Left = v0;
vehs = VehicleRiskMap.empty(nVeh, 0);
x0 =  [0, 0, 0, 0]; % [Fl, Ll, Fo, Lo] % zeros(nVeh, 1);
y0 = zeros(nVeh, 1);

for iterVeh = 1:nVeh
    otherVeh = vehLoc{iterVeh};
    
    switch lower(otherVeh(2))
        case 'd'
            % Create vehicle to the left
            y0(iterVeh) = y0Ego + laneWidth;
            vehs(iterVeh) = VehicleRiskMap(type, vehLoc{iterVeh}, [x0(iterVeh), y0(iterVeh), v0Left], reactionTime);
            
        case 'o'
            % Create vehicle on the same lane
            y0(iterVeh) = y0Ego;
            vehs(iterVeh) = VehicleRiskMap(type, vehLoc{iterVeh}, [x0(iterVeh), y0(iterVeh), v0], reactionTime);
    end
    
    vehs(iterVeh).brakeFullStop(t);
    % Even "worse" case: assume followers brake less than leaders
    switch upper(otherVeh(1))
        case 'L'
            vehs(iterVeh).maxBrake = gammaL*egoVeh.maxBrake;
        case 'F'
            vehs(iterVeh).maxBrake = gammaF*egoVeh.maxBrake;
        otherwise
            error('unknown vehicle location')
    end
    
end


%% Compute min spacings for each ego vehicle speed

v0array = v0-deltaV:0.1:v0+deltaV;
nV0 = length(v0array);

deltaGapsDuringLC = zeros(nVeh, nV0);
minVehFollowingGap = zeros(nVeh, nV0);
gapACC = zeros(nVeh, nV0);
maxVel = 30;
rho = 0.9;
for iterVel = 1:nV0
    for iterVeh = 1:nVeh
        otherVeh = vehs(iterVeh);
        
        egoVeh.v0 = v0array(iterVel);
        egoVeh.sinLatAccel(t, 0, tLat, laneWidth); % assumes left lane change
        deltaGapsDuringLC(iterVeh, iterVel) = egoVeh.computeGapVariationDuringLC(t, otherVeh, tLat);
        
        egoVeh.brakeFullStop(t, otherVeh);
        otherVeh.brakeFullStop(t);
        switch upper(otherVeh.name(1))
            case 'L'
                egoVeh.adjustTimeHeadway(otherVeh, maxVel, rho);
                [minVehFollowingGap(iterVeh, iterVel), gapACC(iterVeh, iterVel)] = egoVeh.computeFollowingMSSDuringLC(t, otherVeh, tLat);
            case 'F'
                otherVeh.adjustTimeHeadway(egoVeh, maxVel, rho);
                [minVehFollowingGap(iterVeh, iterVel), gapACC(iterVeh, iterVel)] = otherVeh.computeFollowingMSS(egoVeh);
        end
        
    end
end

minLCGap = minVehFollowingGap+deltaGapsDuringLC;

%% Plots

%%%% Decided to use gapACC as the minimum following gap - despite knowing
%%%% this doesn't guarantee safety in beyond assumption scenario.

figure;
hold on; grid on;
title('Min Veh Foll');
plot((v0array-v0)*mps2kph, gapACC, 'LineWidth', 1.5);
legend(vehLoc);

figure;
hold on; grid on;
title('Delta gap')
plot((v0array-v0)*mps2kph, deltaGapsDuringLC, 'LineWidth', 1.5);
legend(vehLoc);

figHandles = figure; 
hold on; grid on;
plot((v0array-v0)*mps2kph, gapACC+deltaGapsDuringLC, 'LineWidth', 1.5);
legend(vehLoc, 'FontSize', 14);

% figHandles = figure; 
% hold on; grid on;
% plot((v0array-v0)*mpsTokph, minLCGap, 'LineWidth', 1.5);
% legend(vehLoc,  'FontSize', 14);


%% Save

imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/images/';
% imgFolder = 'G:/My Drive/Lane Change/images/';
figNames = {'min_lc_gaps_equal_speeds'};
% for k = 1:length(figNames)
%     saveas(figHandles(k), ['figures/' figNames{k}]);
%     saveas(figHandles(k), [imgFolder figNames{k} '.eps'], 'epsc');
% end
