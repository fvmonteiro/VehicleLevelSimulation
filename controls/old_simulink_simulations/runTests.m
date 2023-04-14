%% Run tests

clearvars;
close all;

saveFigs = 1;%1;

% Desired ACC poles
polesACC = [0.5, 2, 3];

%% ACC
% close all
% 
% nVehicles = 5;
% accScenario = SimulinkScenario(nVehicles, saveFigs);
% maxBrakeData = accScenario.accMaxBrake(polesACC);
% accZtoMaxToZ = accScenario.accZeroToMaxToZero(polesACC);

%% CACC 1
% close all
% 
% ka = 0.3;
% nVehicles = 5;
% caccScenario = SimulinkScenario(nVehicles, saveFigs);
% % cacc1maxBrake = caccScenario.caccMaxBrake(polesACC, ka);
% cacc1ZtoMaxToZ = caccScenario.caccZeroToMaxToZero(polesACC, ka);


%% CACC 2
% close all
% 
% ka = 0.3;
% nVehicles = 5;
% caccScenario = SimulinkScenario(nVehicles, saveFigs);
% maxBrakeStates = caccScenario.caccFilterMaxBrake(polesACC, ka);
% cacc2ZtoMaxToZ = caccScenario.caccFilterZeroToMaxToZero(polesACC, ka);

%% Gap generation
close all

KiVelControl = 1;
vehiclesPerLane = [2 3];
ggScenario = SimulinkScenario(vehiclesPerLane, saveFigs);
ggData = ggScenario.gapGeneration(polesACC, KiVelControl);

%% Lane Change for single vehicle
% close all
% 
% nVehicles = 1;
% % Desired lane change poles
% polesLC = -[0.5, 1, 2, 3];
% lcScenario = SimulinkScenario(nVehicles, saveFigs);
% measuredVars = 1:4;%1;%[1, 4];
% observer = 1;
% lcScenario.laneChangeSingleVeh(polesLC, measuredVars);

%% Lane Change with full state feedback and observer
% close all
% 
% nVehicles = 5;
% Desired lane change poles
% polesLC = -[0.5, 1, 1.5, 2];
% lcScenario = SimulinkScenario(nVehicles, saveFigs);
% measuredVars = [1, 4];
% comms = 1;
% lcScenario.laneChangeFullStateWithObserver(polesLC, measuredVars, comms);

% lcScenario.laneChangeFullState(polesLC, comms);
% comms = 0;
% lcScenario.laneChangeFullState(polesLC, measuredVars, comms);

%% Lane Change with output feedback
% close all

% nVehicles = 1;
% % Desired lane change poles
% polesLC = [0.5, 1, 1.5, 2, 3]/2;
% lcScenario = SimulinkScenario(nVehicles, saveFigs);
% lcScenario.laneChangeOutputFeedback(polesLC);

%% Lateral control
% latControlModelName = 'lateralControl';
% nFollowers = 1;
% ka = 0.3;
% useLeaderAccelFilter = 1;
% latPoles = [0.02, 0.1, 0.2];
% latControlScenario = SimulinkScenario(latControlModelName, nFollowers);
% lcStates = latControlScenario.platoonLaneChange([poles;latPoles], ka, useLeaderAccelFilter);
