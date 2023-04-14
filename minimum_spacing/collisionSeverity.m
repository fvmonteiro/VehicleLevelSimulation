function [g0, s2, minLatCollisionFreeGap] = collisionSeverity(simTime, deltaS, ego, other)
%collisionSeverity Computes collision severity for equally spaced different
%initial gaps
%   tCrossing is an optional parameter

warning('Deprecated: this function is now a method of the vehicle class');

% checkForLateralCollision = 0;
% if nargin==6
%     checkForLateralCollision = 1;
%     warning('Computation of lateral collision in worst case braking scenario is not ready.')
% end

% Acceleration profiles
ego.brakeFullStop(simTime, other);
other.brakeFullStop(simTime);

otherLoc = char(other.location);
mainIsLeader = otherLoc(1)=='F'; %strncmpi(other.location, 'F', 1);
% bothInSameLane = otherLoc(2)=='o'; %

% Minimum safety spacing
% minSafeGap = computeFollowingMSS(simTime, main, other, delay);
minSafeGap = analyticalFollowingMSS(ego, other); 
% above line is not an efficient way of dealing with the problem, but it's
% easy :)
g0range = 0:deltaS:max(minSafeGap);

vM = ego.computeVel(simTime);
vOther = other.computeVel(simTime);
g0 = repmat(g0range', 1, length(ego.v0)); % = g0range?
s2 = zeros(length(g0range), length(ego.v0));
minLatCollisionFreeGap = zeros(length(ego.v0), 1);
for n = 1:length(ego.v0)

    deltaV = (-1)^mainIsLeader*vM(:, n) - (-1)^mainIsLeader*vOther; 
    delta = (-1)^mainIsLeader*cumtrapz(simTime, vM(:, n)) - ...
    (-1)^mainIsLeader*cumtrapz(simTime, vOther);

    if max(delta)>0
        % Note: there might be two virtual collisions and we want to be
        % sure to get the first one.
        [~, collisionIdx] = mink(abs(g0(:, n)' - delta), 10);
        collisionIdx = min(collisionIdx);
        s2(:, n) = deltaV(collisionIdx); %.^2;
        
        % Checking for lateral collisions
        %%%% This algorithm is wrong - it assumes the lateral acceleration
        %%%% profile is constant independent of the longitudinal 
        %%%% acceleration (the mistake is not in these lines of code alone
        %%%% but mostly in the computation of tCrossing using a single
        %%%% aLat profile)
%         if checkForLateralCollision
%             collisionTimes = simTime(collisionIdx);
%             if bothInSameLane
%                 minLatCollisionFreeGap(:, n) = max(g0(collisionTimes<=tCrossing));
%             else
%                 minLatCollisionFreeGap(:, n) = max(g0(collisionTimes>=tCrossing));
%             end
%         end

    end
    
end



end

