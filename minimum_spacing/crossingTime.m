function [tc] = crossingTime(simTime, egoVeh, otherVeh)
%crossingTime Find the time after which the merging vehicle might collide 
% with others on the adjacent lane

vLat = egoVeh.computeVelLat(simTime);
vM = egoVeh.computeVel(simTime);

yCenter = egoVeh.y0 + cumtrapz(simTime, vLat);
yFrontLeft = yCenter + (egoVeh.width/2)*vM./sqrt(vLat.^2+vM.^2); 

y = yFrontLeft;
vehName = char(otherVeh.location);
switch vehName(1)
    case 'L'
        % do nothing
    case 'F'
        y = y - egoVeh.len*vLat./sqrt(vLat.^2+vM.^2); %y rear left
    otherwise
        error('unknown vehicle')
end

switch vehName(2)
    case {'d', 'l'} % d: destination or l: left
        yCrossing = otherVeh.y0-otherVeh.width/2;
    case 'o'
        yCrossing = otherVeh.y0+otherVeh.width/2;
        y = y - egoVeh.width*vM./sqrt(vLat.^2+vM.^2); % y right
    otherwise
        error('unknown vehicle')
end


[~, idx] = min(abs(y-yCrossing)); % we can code it this way because 
% we know there's a single crossing point.

tc = simTime(idx);

end

