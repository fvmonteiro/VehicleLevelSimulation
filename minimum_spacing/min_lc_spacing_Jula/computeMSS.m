function [MSS] = computeMSS(mergingVeh, vehicles, ...
    aLat, t)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

finalTime = t(end);
ts = t(2)-t(1);
MSS = struct();

vm = mergingVeh.computeVel(ts);
vLat = cumtrapz(t, aLat);
yFrontLeft = mergingVeh.y0 + cumtrapz(t, vLat);

for k = 1:length(vehicles)
%     interval = repmat([0 finalTime], size(vm, 2), 1);
    interval = [0 finalTime];
    otherVeh = vehicles(k);
    vehName = otherVeh.name;
    
    y = yFrontLeft;
    switch vehName(1)
        case 'L'
            vl = otherVeh.computeVel(ts);
            vf = vm;
        case 'F'
            y = y - mergingVeh.len*vLat./sqrt(vLat.^2+vm.^2);
            vl = vm;
            vf = otherVeh.computeVel(ts);
        otherwise
            error('unknown vehicle')
    end
        
    yCrossing = otherVeh.y0;
    switch vehName(2)
        case 'd'
            yCrossing = yCrossing-mergingVeh.width;
            intervalIdx = 1;
        case 'o'
            y = y - mergingVeh.width*vm./sqrt(vLat.^2+vm.^2);
            intervalIdx = 2;
        otherwise
            error('unknown vehicle')
    end
    
    deltaS = cumtrapz(t, vf-vl); % relative position
    nScenario = size(deltaS, 2); % # of relative speeds being tested
    
    tc = crossingTime(t, y, yCrossing);
    if isscalar(tc)
        tc = tc*ones(nScenario, 1);
    end
    
    MSS.(vehName) = zeros(nScenario, 1);
    for n = 1:nScenario % the function may be computing MSS for several scenario at once
        interval(:, intervalIdx) = tc(n);
        indices = t>=interval(1) & t<=interval(2);
        MSS.(vehName)(n) = max(deltaS(indices, n));
    end
        
end


end

