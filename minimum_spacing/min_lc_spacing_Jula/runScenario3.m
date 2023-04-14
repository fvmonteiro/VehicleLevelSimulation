function [figHandle] = runScenario3(t, tLat, h, relVelRange, otherVehs, ...
    mergingVeh, deltaS0, deltaV0, aAdj)
%runScenario3 Run and plot results for scenario where merging vehicle is 
% adjust its position and speed before and during lane change

ta = t(end); % in scenario3, there's no coasting (accel = 0) period
[figHandle] = runScenario4(t, tLat, ta, h, relVelRange, otherVehs, ...
    mergingVeh, deltaS0, deltaV0, aAdj);

end

