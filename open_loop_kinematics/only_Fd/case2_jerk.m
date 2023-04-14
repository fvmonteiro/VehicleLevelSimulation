function [jerk] = case2_jerk(s, tf, a0)
%case3_jerk Compute jerk given total maneuver time
%   Detailed explanation goes here

jerk = 6*(s-tf^2*a0/2)/(tf^3);

end

