function [aLat] = sinLatAccel(t, tAdj, tLat, h)
%sinLatAccel Creates sinusoidal profile for lateral acceleration
% t: simulation time
% tAdj: maneuver start time (longitudinal adjustment time) [s]
% tLat: lane change duration [s]
% h: total lateral displacement [m]

aLat = zeros(length(t), 1);
nonZeroIndices = t>=tAdj & t<=(tLat+tAdj);
aLat(nonZeroIndices) = 2*pi*h/(tLat^2) * sin(2*pi/tLat * (t(nonZeroIndices)-tAdj));

end

