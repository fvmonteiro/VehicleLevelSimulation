function [tc] = crossingTime(t, yLat, yC)
%crossingTime Find the time after which the merging vehicle might collide 
% with others on the adjacent lane

[~, idx] = min(abs(yLat-yC)); % we can code it this way because 
% we know there's a single crossing point.

tc = t(idx);

end

