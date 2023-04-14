function [MSS] = computeFollowingMSS(simTime, ego, other)
%computeFollowingMSS Computes minimum following distance similar to Kanaris
%work.
%   Solution is found by directly solving the integral for all t and then
%   looking for the maximum delta S.

% Acceleration profiles
ego.brakeFullStop(simTime, other);
other.brakeFullStop(simTime);

mainIsLeader = strncmpi(other.location, 'F', 1);

% Minimum safety spacing
MSS = zeros(length(ego.v0), length(other.v0));
vM = ego.computeVel(simTime);
vOther = other.computeVel(simTime);
for n = 1:length(ego.v0)

    delta = (-1)^mainIsLeader*cumtrapz(simTime, vM(:, n)) - ...
    (-1)^mainIsLeader*cumtrapz(simTime, vOther);

    MSS(n, :) = max(delta);
end

end