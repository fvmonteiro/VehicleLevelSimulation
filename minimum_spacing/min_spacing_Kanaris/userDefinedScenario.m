clearvars;
close all;

%% Constants
global mileToMeter
global g 
global Ts
mileToMeter = 1609.34;
g = 9.8; % [m/s^2]
Ts = 0.01;

%% Scenario (mix of vehicle types)
% Legend: P - passenger vehicle, B - bus, T - truck
% First letter is the leader type, second letter is the follower type
leader = 'P';
follower = 'T';
vehicles = [leader follower];

%% AHS concepts:
%1 Autonomous Vehicles
%2 Free Agent Vehicles - Infrastucture Supported
%3 Free Agent Vehicles - Infrastucture Managed
%4 Platooning without Coordinated Braking
%5 Platooning with Coordinated Braking and no delay
%6 Vehicle Platoons with coordinated braking and staggered timing
%7 Infrastructure Managed Slotting (don't use)
ahsConcept = 4;

%% Leader initial conditions
vl0 = 60*mileToMeter/3600; % [m/s]

%% Follower initial conditions
% Suggested values for AHS concepts 1, 2 and 3
% vf0 = 63*mileToMeter/3600; % [m/s]
% af0 = 0.15*g; % [m/s^2]
% Suggested values for AHS concepts 4, 5 and 6
vf0 = 61.5*mileToMeter/3600; % [m/s]
af0 = 0; % [m/s^2]

%% Maximum collision speed (usually used only for AHS concepts 4, 5 and 6)
maxDeltaVmph = 5; % [miles per hour]
maxDeltaV = maxDeltaVmph*mileToMeter/3600; % [m/s]

%% Compute MSS
params = [vl0, af0, vf0, maxDeltaV];
if ahsConcept>=4
    if vehicles(1)~= vehicles(2)
        fprintf(['In platoons, there is no mixing of vehicles. '...
            'Follower type changing from %s to %s \n'], vehicles(2), vehicles(1))
        vehicles(2) = vehicles(1);
    end
end
[MSS, al, af] = runScenario(vehicles, ahsConcept, params);

%% Print results
fprintf('AHS concept: %d , Vehicle type: %s%s \n', ahsConcept, vehicles(1), vehicles(2));
fprintf('\t Collision free MSS: %0.3fm \n', MSS(1));
fprintf('\t Allowing %dmph collisions min headway: %0.3fm , max headway %0.3fm \n',...
    maxDeltaVmph, MSS(2), MSS(3));

%% Plots

plotChoice = 0;

if plotChoice
    t = 0:Ts:(length(af)-1)*Ts;
    
    figure; hold on; grid on;
    plot(t, al);
    plot(t, af);
    legend(sprintf('%s leader', vehicles(1)), sprintf('%s follower', vehicles(2)));
    title('accel')
    
    vl = vl0 + cumtrapz(Ts, al);
    vf = vf0 + cumtrapz(Ts, af);
    figure; hold on; grid on;
    plot(t, vl); plot(t, vf);
    legend('leader', 'foll');
    title('speed')
    
    deltaGap = cumtrapz(Ts, vl-vf);
    figure; hold on; grid on;
    plot(t, deltaGap)
    title('\Delta S')
end