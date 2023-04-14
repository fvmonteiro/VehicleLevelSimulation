function [al, t] = leaderAccelProfile(v0, maxJerk, maxBrake)
%leaderAccelProfile Creates the leader acceleration profile

global Ts

timeToMaxBrake = maxBrake/maxJerk;

vt1 = v0 - maxJerk/2*timeToMaxBrake^2; % speed at timeToMaxBrake
timeToFullStop = timeToMaxBrake + vt1/maxBrake;

t = 0:Ts:ceil(timeToFullStop);
al = zeros(length(t), 1);
al(t<=timeToMaxBrake) = -maxJerk*t(t<=timeToMaxBrake);
al(t>timeToMaxBrake & t<=timeToFullStop) = -maxBrake;

end

