clc;
clearvars;
close all;

% Constants
global mileToMeter
global g 
global Ts
mileToMeter = 1609.34;
g = 9.8; % [m/s^2]
Ts = 0.1;

% Legend: P - passenger vehicle, B - bus, T - truck
possibleVehicles = {'P', 'B', 'T'};
% AHS concepts:
%1 Autonomous Vehicles
%2 Free Agent Vehicles - Infrastructure Supported
%3 Free Agent Vehicles - Infrastructure Managed
%4 Platooning without Coordinated Braking
%5 Platooning with Coordinated Braking and no delay
%6 Vehicle Platoons with coordinated braking and staggered timing
%7 Infrastructure Managed Slotting (don't use)

% Leader initial conditions
vl0 = 60*mileToMeter/3600; % [m/s]

% Follower initial conditions
vf0 = 63*mileToMeter/3600; % [m/s]
af0 = 0.15*g; % [m/s^2]
params = [vl0, af0, vf0];
for ahsConcept = 1:3
    fprintf('AHS concept: %d \n', ahsConcept);
    for v = 1:length(possibleVehicles)
        for f = 1:length(possibleVehicles)
            % First letter is the leader type, second letter is the follower type
            leader = possibleVehicles{v};
            follower = possibleVehicles{f};
            vehicles = [leader follower];
            [MSS, al, af] = runScenario(vehicles, ahsConcept, params);
            fprintf('\tScenario: %s%s, MSS: %0.3f \n', ...
                    leader, follower, MSS);
        end
    end
end

% Follower initial conditions
vf0 = 61.5*mileToMeter/3600; % [m/s]
af0 = 0; % [m/s^2]
maxDeltaVmph = 5; % [miles per hour]
maxDeltaV = maxDeltaVmph*mileToMeter/3600; % [m/s]
params = [vl0, af0, vf0, maxDeltaV];
for ahsConcept = 4:6
    fprintf('AHS concept: %d \n', ahsConcept);
    vehicle = possibleVehicles{1};
    fprintf('\t Vehicle type: %s \n', vehicle);
    % First letter is the leader type, second letter is the follower type
    leader = vehicle;
    follower = vehicle;
    vehicles = [leader follower];
    [MSS, al, af] = runScenario(vehicles, ahsConcept, params);
    fprintf('\t\t Collision free MSS: %0.3fm \n', MSS(1));
    fprintf('\t\t Allowing %dmph collisions min headway: %0.3fm , max headway %0.3fm \n',...
        maxDeltaVmph, MSS(2), MSS(3));
end