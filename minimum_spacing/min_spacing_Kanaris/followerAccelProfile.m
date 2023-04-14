function [af, t] = followerAccelProfile(v0, a0, concept, maxJerk, maxBrake)
%followerAccelProfile Create the follower's acceleration profile
%   Detailed explanation goes here

% Global constants
global g
global Ts

% We assume maximum deceleration and jerk are provided as positive values
% To guarantee algebra will work:
maxJerk = abs(maxJerk);
maxBrake = abs(maxBrake);

% These values only differ for the first concept.
afauto = a0;
jfauto = 1; % any value different than zero

switch concept
    case 1 % Autonomous Vehicles
        % Parameters
        afauto = -0.1*g; % [m/s^2] comfortable acceleration
        jfauto = 5; % [m/s^3] comfortable jerk
        delays = [0.2, 0.3]; 
    case 2 % Free Agent Vehicles - Infrastucture Supported
        delays = [0.1, 0];
    case 3 % Free Agent Vehicles - Infrastucture Managed
        delays = [0, 0];
    case 4 % Platooning without Coordinated Braking
        %%%% TODO: intra and inter platoon spacing (the second one allows
        %%%% low relative speed collisions
        delays = [0.1, 0];
%         a0 = 0; % [m/s^2]
%         v0 = 61.5*mileToMeter/3600; % [m/s]
    case 5 % Platooning with Coordinated Braking and no delay
        delays = [0, 0];
%         a0 = 0; % [m/s^2]
%         v0 = 61.5*mileToMeter/3600; % [m/s]
    case 6 % Vehicle Platoons with coordinated braking and staggered timing
        delays = [-0.1, 0];
%         a0 = 0; % [m/s^2]
%         v0 = 61.5*mileToMeter/3600; % [m/s]
    case 7 % Infrastructure Managed Slotting
        warning('Not coded because it is not presented in detail');
    otherwise
        error('Unknown AHS concept')
end

intervals = zeros(6, 1);
intervals(2) = delays(1); % [s] delay to start braking
intervals(3) = -(afauto-a0)/jfauto; % [s] extra time to reach afauto
intervals(4) = delays(2); % [s] delay between regular braking and emergency braking
intervals(5) = (-maxBrake - afauto)/(-maxJerk); % [s] extra time to reach max brake

vt1 = v0 + a0*intervals(2);
vt2 = vt1 + a0*intervals(3) - 1/2*jfauto*intervals(3)^2;
vt3 = vt2 + afauto*intervals(4);
vt4 = vt3 + afauto*intervals(5) - 1/2*maxJerk*intervals(5)^2;
intervals(6) = vt4/maxBrake; % [s] time to full stop

changeTimes = cumsum(intervals);
t = 0:Ts:ceil(changeTimes(end));
booleanIntervals = false(length(intervals)-1, length(t));
for k = 1:size(booleanIntervals, 1)
    booleanIntervals(k, :) = logical(t>=changeTimes(k) & t<changeTimes(k+1));
end

af = zeros(length(t), 1);
af(booleanIntervals(1, :)) = a0;
af(booleanIntervals(2, :)) = a0 - jfauto*(t(booleanIntervals(2, :))-changeTimes(2));
af(booleanIntervals(3, :)) = afauto;
af(booleanIntervals(4, :)) = afauto - maxJerk*(t(booleanIntervals(4, :))-changeTimes(4));
af(booleanIntervals(5, :)) = -maxBrake;

end

