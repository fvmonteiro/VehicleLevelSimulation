function [MSS, al, af] = runScenario(vehicles, ahsConcept, params)
%runScenario Function provides the acceleration profiles of leader and
%follower and returns the minimum safety spacing between the two vehicles.

% Constants
global g

vl0 = params(1);
af0 = params(2);
vf0 = params(3);

% Define maximum acceleration and jerks based on vehicle type
[leaderMaxBrake, leaderMaxJerk] = setMaxAccelAndJerk(vehicles(1), 1);
[follMaxBrake, follMaxJerk] = setMaxAccelAndJerk(vehicles(2), 2);

% Acceleration profiles
[al] = leaderAccelProfile(vl0, leaderMaxJerk, leaderMaxBrake);
[af] = followerAccelProfile(vf0, af0, ahsConcept, follMaxJerk, follMaxBrake);

if length(af)>length(al)
    al = [al; zeros(length(af)-length(al), 1)]; % zero padding
else
    af = [af; zeros(length(al)-length(af), 1)]; % zero padding
end

% Minimum safety spacing
vehLength = 4; % vehicle length is not provided in the report, but it
% makes no difference in finding the min gap anyway.

MSS = computeMSS(vf0, af, vehLength, vl0, al);
if length(params)==4
    maxDeltaV = params(4);
    MSS(2:3) = computeMSS(vf0, af, vehLength, vl0, al, maxDeltaV);   
end

    function [maxBrake, maxJerk] = setMaxAccelAndJerk(vehicle, position)
               
        % To conservatively account for within-type different braking 
        % capabilities, followers get 90% maximum deceleration of the leaders
        if position==1
            factor= 1;
        elseif position==2
            factor = 0.9;
        else
            error('Unknown vehicle position')
        end
        
        % Typical vehicle parameters
        switch vehicle
            case 'P' % passenger vehicle
                maxBrake = factor*0.8*g;% [m/s^2]
                maxJerk = 50; % [m/s^3]
            case 'B' % bus
                maxBrake = factor*0.4*g; % [m/s^2] bus
                maxJerk = 40; % [m/s^3]
            case 'T' % truck
                maxBrake = factor* 0.3*g; % [m/s^2] truck
                maxJerk = 30; % [m/s^3]
            otherwise
                error('Unknown vehicle type')
        end
        
    end

end