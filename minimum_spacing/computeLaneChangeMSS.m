function [MSS, allCrossingTimes] = computeLaneChangeMSS(egoVeh, vehicles, t, tEndLaneChange)
%computeLaneChangeMSS Find minimum spaces for lane changing following Jula
%   Algorithm assumes all other vehicles keep constant speed and finds
%   spacing to all four surrounding vehicles such that no collision occurs.

vM = egoVeh.computeVel(t);
% # of relative speeds being tested (we assume all vehicles other than the
% ego have the same number of possible initial speeds)
nScenario = max(length(egoVeh.v0), length(vehicles(1).v0));
if isscalar(tEndLaneChange)
    tEndLaneChange = ones(nScenario, 1)*tEndLaneChange;
end

allCrossingTimes = -ones(length(vehicles), nScenario); % for tests and checks


MSS = zeros(length(vehicles), nScenario);
for k = 1:length(vehicles)
    otherVeh = vehicles(k);
    vehName = char(otherVeh.location);
    
    switch upper(vehName(1))
        case 'L'
            vl = otherVeh.computeVel(t);
            vf = vM;
        case 'F'
            vl = vM;
            vf = otherVeh.computeVel(t);
        otherwise
            error('unknown vehicle')
    end
    
    deltaS = cumtrapz(t, vf-vl); % relative position
    
    switch lower(vehName(2))
        case {'d', 'r', 'l'} % destination, right, or left
            intervalIdx = 1;
        case 'o'
            intervalIdx = 2;
        otherwise
            error('unknown vehicle')
    end
    
    tc = crossingTime(t, egoVeh, otherVeh);
    if isscalar(tc)
        tc = tc*ones(1, nScenario);
    end
    
    for n = 1:nScenario % the function may be computing MSS for several scenarios at once
        interval = [0 tEndLaneChange(n)];
        interval(intervalIdx) = tc(n);
        indices = t>=interval(1) & t<=interval(2);
        MSS(k, n) = max(deltaS(indices, n));
    end
    allCrossingTimes(k, :) = tc;
        
end

% Not elegant recent change to make it easier to work vehicle by vehicle
% if length(vehicles)==1
%     MSScell = struct2cell(MSS);
%     MSS = cell2mat(MSScell);
% end

end

