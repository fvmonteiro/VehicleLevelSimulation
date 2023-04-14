% Plot the most general worst-cases

warning('[Jun 10, 2020] All code transferred to SimulationScenarios class')

close all
clearvars;

%% Parameters
n = 1;
t = 0:0.001:15; % Simulation time
v0L = 25;
v0F = 25;
% Perception delays (human, autonomous, connected)
delay1 = [1, 0.3, 0.3]; 
delay2 = [1, 0.3, 0];

%% Leader only brake (no jerk phase) and follower types 

leader = VehicleLongitudinal('P', 'L', [0, v0L], 0);
leader.maxJerk = inf;
leader.brakeFullStop(t);

nF = 3;
follower(nF) = VehicleLongitudinal();

% Plot 1
figHandles(n) = leader.plotState(t, 'a');
for k = 1:nF
    follower(k) = VehicleLongitudinal('P', 'F', [0, v0F], [delay1(k), delay2(k)]);
    follower(k).maxBrake = 0.8*follower(k).maxBrake;
    follower(k).brakeFullStop(t);
    follower(k).plotState(t, 'a', figHandles(n));
%     follower(k).createPlotAnnotations(figHandles(n));
end
legend({'leader', 'human follower', 'autonomous follower', 'connected follower'}, 'FontSize', 14);
% figNames{n} = 'follower_accel_profiles';
figNames{n} = 'all_accel_profiles';
n = n + 1;

%% Leader with jerk and human follower
leader = VehicleLongitudinal('P', 'L', [0, v0L], 0);
follower = VehicleLongitudinal('P', 'F', [0, v0F], [delay1(1), delay2(1)]);
follower.maxBrake = leader.maxBrake*0.8;

leader.brakeFullStop(t);
follower.brakeFullStop(t);
% Plot 2
figHandles(n) = leader.plotState(t, 'a');
figHandles(n) = follower.plotState(t, 'a', figHandles(n));
legend({'leader', 'follower'}, 'FontSize', 14)
figNames{n} = 'worst_case_braking_both';
n = n + 1;

%% Leader only brake (no jerk phase) and simplified follower
leader = VehicleLongitudinal('P', 'L', [0, v0L], 0);
follower = VehicleLongitudinal('P', 'F', [0, v0F], [delay1(1), delay2(1)]);
follower.maxBrake = leader.maxBrake*0.8;

leader.maxJerk = inf;
% follower.timeToBrake = follower.timeToBrake - ...
%     (follower.comfBrake-follower.maxAccel)/follower.comfJerk + follower.timeToEmergencyBrake;
% follower.timeToBrake = follower.timeToBrake; % - 0.5; % better looking plot
follower.timeToEmergencyBrake = 0;
follower.comfJerk = follower.maxJerk;
leader.brakeFullStop(t);
follower.brakeFullStop(t);
% Plot 3
figHandles(n) = leader.plotState(t, 'a');
figHandles(n) = follower.plotState(t, 'a', figHandles(n));
legend({'leader', 'follower'}, 'FontSize', 16)
figNames{n} = 'worst_case_braking_simplified';
n = n + 1;

%% Save
% imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/images/';
% % imgFolder = 'G:/My Drive/Lane Change/images/';
% 
% for iterFig = 1:length(figHandles)
%     saveas(figHandles(iterFig), ['figures/' figNames{iterFig} ]);
%     saveas(figHandles(iterFig), [imgFolder figNames{iterFig} '.eps'], 'epsc');
% end

