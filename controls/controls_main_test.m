%%%%% TEST SCRIPT FOR CONTROLS APPROACH %%%%%
% clc
clearvars;
close all;

%% Tests with matlab coded vehicle model (double integrator)
% testObj = MatlabScenario();

% n_states = 3;
% fig1 = testObj.testOpenLoopControllers(1);
% fig2 = testObj.testOpenLoopControllers(2, 1);
% fig3 = testObj.testOpenLoopControllers(3, 1);

% Not working - left aside for now
% testObj.testThrottleController();

% Test if filtered speed reference works
% testObj.testFilters();

% Main longitudinal model tests
% vehArrays = testObj.testACC();
% platoons = testObj.testVehArray();
% [platoon, lqr_fig] = testObj.testInfTimeQueueLQR();
% va = testObj.testGapGenerationImpactVsN();
% testObj.testGapGenerationImpactVsDeterministicSpacing();
% testObj.testGapGenerationImpactVsRandomSpacing();

% Lane change tests
% testObj.testBicycleModel();
% [states_fig, inputs_fig] = testObj.testSeqPlatoonLC();
% [states_fig, ~] = testObj.testPlatoonLCwithGapGeneration();

% Saving plots
% testObj.saveBigPlot(fig1, 'min_time');
% testObj.saveBigPlot(fig2, 'min_time_u2');
% testObj.saveBigPlot(fig3, 'min_time_u1');
% testObj.saveBigPlot(lqr_fig, 'mult_veh_lqr');
% testObj.saveBigPlot(states_fig, 'platoon_seq_lc_states');
% testObj.saveBigPlot(inputs_fig, 'platoon_seq_lc_inputs');

%% ACC string stability tests
 
% saveFigs = 0;
% polesACC = [1, 2, 3]; % Desired ACC poles
% polesVelCtr = [0.5, 1]; % Vel control poles
% vehiclesPerLane = 5;
% maxBrakeScenario = SimulinkScenario(vehiclesPerLane, saveFigs);
% [platoon, desVel] = maxBrakeScenario.accMaxBrake(polesACC, polesVelCtr);

%% CACC tests
% Having issues with low max deceleration value, which puts the vehicle in
% a situation where it brakes as in an emergency

% vehFollParams = [0.1]; % Parameters of the veh following control
% 
% percentOvershoot = 1e-2;
% logTemp = log(1/percentOvershoot)^2;
% % settlingTime = 60;
% % peakTime = 50;
% riseTime = 40;
% 
% zeta = 1.1; %sqrt(logTemp/(pi^2+logTemp));
% % wn = 4/zeta/settlingTime;
% % wn = pi/(peakTime*sqrt(1-zeta^2));
% wn = -log(0.1)/(zeta - sqrt(zeta^2-1))/riseTime;
% 
% sqrtDelta = sqrt(zeta^2-1);
% dominantPoles = -wn*(zeta + [sqrtDelta, -sqrtDelta]);
% velCtrlPoles = [-wn*zeta*5, dominantPoles]; % Vel control poles
% ctrlParams = struct('vehFollCtrlParams', vehFollParams, 'hasAccelFeedback',...
%     true, 'velCtrlPoles', velCtrlPoles);
% 
% caccScenario = SimulinkScenario();
% deltaX0 = 300;
% % vehArray = caccScenario.prepareCACCSimulation(ctrlParams, deltaX0);
% vehArray = caccScenario.caccBasics(ctrlParams, deltaX0);

%% Single veh/ platoon gap generation in Simulink

% vehFollPoles = [1, 2, 3]/2; % Desired ACC poles
% polesVelCtr = [0.5, 1]; % Vel control poles
% relVel = 0; % suggested: 0, 4, -4
%  
% ggScenario = SimulinkScenario();
% vehArrays = ggScenario.gapGeneration(vehFollPoles, polesVelCtr, relVel, 'single_veh');
% vehArrays = ggScenario.gapGeneration(polesACC, polesVelCtr, relVel, 'platoon');

%% Get all results and save all (gap generation)
% saveResults = 0;
% polesACC = [1, 2, 3]/2; % Desired ACC poles
% polesVelCtr = [0.5, 1]; % Vel control poles
% ggScenario = SimulinkScenario(saveResults);
% ggScenario.ggRunAndSaveAll(polesACC, polesVelCtr);

%% Gap generation + lane change maneuver

% vehFollPoles = [1, 2, 3]/2; % Desired ACC poles
% velCtrlPoles = -[0.5, 1]; % Vel control poles
% latPoles = -[0.5, 1, 2, 3]*2; % pole placement
% % latParams = {[1 0 0 0;0 1 0 0;0 0 0 0;0 0 0 0], 200}; % LQR
% ctrlParams = struct('vehFollCtrlParams', vehFollPoles, 'hasAccelFeedback', false,...
%     'velCtrlPoles', velCtrlPoles, 'latCtrlParams', latPoles);
% relVel = 0; %[0, 5, -5]; % suggested: 0, 5, -5 
% 
% fullScenario = SimulinkScenario();
% % actuatorLag = 0;
% % vehArrays = fullScenario.prepareLCSimulation(actuatorLag, ctrlParams, ...
%     relVel, 'single_veh');
% % fullScenario.saveResults = true;
% isPlatoon = false;
% simConfig = struct('isPlatoon', isPlatoon, 'relVel', relVel);
% vehArrays = fullScenario.fullManeuver(ctrlParams, simConfig);

% platoonLCStrategy = 2; %[2, 3]; %1: synchronous;  2: leader first; 3: last first 
% fullScenario.prepareLCSimulation(vehFollPoles, velCtrlPoles, latParams, ...
%     relVel, 'platoon', platoonLCStrategy);
% fullScenario.saveResults = true;
% vehArrays = fullScenario.fullManeuver(vehFollPoles, velCtrlPoles, ...
%     latParams, relVel, 'platoon', platoonLCStrategy);

% Get all results and save all (gap generation + lane change)
% fullScenario.saveResults = true;
% fullScenario.fullManeuverPlatoonRunAll(vehFollPoles, velCtrlPoles, latParams);

%% Gap generation + lane change maneuver with acceleration feedback

% Vehicle following controller parameters
% vehFollParams = [0.9, 0.1]; % Parameters of the veh following control
% velCtrlPoles = -[1, 2, 3]/8; % Vel control poles
vehFollParams = 0.1; % Parameter of the veh following control (CACC PD)

% Velocity controller parameters
% percentOvershoot = 1e-2;
% logTemp = log(1/percentOvershoot)^2;
% settlingTime = 60;
% peakTime = 50;
riseTime = 30;
zeta = 0.8; %sqrt(logTemp/(pi^2+logTemp));
% wn = 4/zeta/settlingTime;
% wn = pi/(peakTime*sqrt(1-zeta^2));
if zeta>1
    wn = -log(0.1)/(zeta - sqrt(zeta^2-1))/riseTime;
else
    wn = -log(0.1)/(zeta - sqrt(1-zeta^2))/riseTime;
end
sqrtDelta = sqrt(zeta^2-1);
dominantPoles = -wn*(zeta + [sqrtDelta, -sqrtDelta]);
velCtrlPoles = [-wn*zeta*5, dominantPoles];

% Lateral controller parameters
latPoles = -[0.5, 1, 2, 3]*2; % pole placement
ctrlParams = struct('vehFollCtrlParams', vehFollParams, 'hasAccelFeedback',...
    true, 'velCtrlPoles', velCtrlPoles, 'latCtrlParams', latPoles);

% Scenario parameters
isPlatoon = true;
% Strategy options: synchronous, leaderFirst, lastFirst, leaderFirstInvert
lcStrategy = SimulinkVehicle.possibleStrategies;
relVel = [-5, 0, 5]; % suggested: 0, 5, -5
simConfig = struct('isPlatoon', isPlatoon, 'relVel', relVel, ...
  'lcStrategy', lcStrategy, 'isTruckLaneChange', false);

fullScenario = SimulinkScenario();
% Set to save and run scenario
fullScenario.saveResults = true;
vehArrays = fullScenario.fullManeuver(ctrlParams, simConfig);

% Just set parameters of simulink model
% isTruckLaneChange = false;
% scenario = 'platoon';
% relVel = 0;
% vehArrays = fullScenario.prepareLCSimulation(isTruckLaneChange, ...
%                 ctrlParams, relVel, scenario, lcStrategy);
% for n = 1:size(vehArrays, 1)
%     try
%         [arrayArray, arrayBehind] = fullScenario.addDummyVehicles(...
%             vehArrays(n, :));
%     catch ME
%         warning('Issue creating dummy vehicles')
%         warning(ME.message)
%     end
% end