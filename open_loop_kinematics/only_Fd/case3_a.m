function a = case3_a(t, jerk, a0, a_max, t1)
%case3_a Compute a(t) for case 3
%   Detailed explanation goes here

a = [a0+jerk*t(t<=t1) (a_max)*ones(1,sum(t>t1))];

end

