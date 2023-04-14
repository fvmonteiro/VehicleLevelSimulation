function [h, g0] = safeHeadway(maxAccel, minAccel, minJerk, reactionTime)
%safeHeadway computes the minimum safe following distance considering the
%case where the leader brakes with maximum force and main vehicle is
%travelling with maximum acceleration
% Function expects: minAccel<0, minJerk<0

% Distance travelled by the leader before coming to a full stop
% leaderDist = -leaderVel^2/(2*leaderBrake); 
% Distance travelled by the main vehicle
% First interval: vehicle still travelling at full acceleration
% vel0 = initVel;
% vel1 = vel0 + maxAccel*reactionTime;
% dist1 = (vel1^2-vel0^2)/(2*maxAccel);
% % Second interval: from max acceleration to max brake
% t1 = (maxBrake-maxAccel)/minJerk;
% dist2 = vel1*t1 + 1/2*maxAceel*t1^2 + 1/6*minJerk*t1^3;
% % Third interval: getting to zero speed at max brake
% vel2 = vel1 + maxAccel*t1 + 1/2*minJerk*t1^2;
% dist3 = -vel2^2/(2*maxBrake);
% 
% % Sum them all
% dist = dist1+dist2+dist3;
% exactWorstCase = leaderDist-dist;

T = reactionTime;
t1 = (minAccel-maxAccel)/minJerk;
h = T + t1 - ...
    1/minAccel*(maxAccel*T + maxAccel*t1+1/2*minJerk*t1^2);

g0 = 1/2*maxAccel*T^2 + maxAccel*T*t1 + + 1/2*maxAccel*t1^2 + 1/6*minJerk*t1^3 - ...
    1/(2*minAccel)*(maxAccel*T + maxAccel*t1+ 1/2*minJerk*t1^2)^2;

end