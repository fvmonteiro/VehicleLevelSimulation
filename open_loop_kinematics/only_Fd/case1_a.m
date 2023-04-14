function a = case1_a(t, s, tf)
%case1_a Compute a(t) for case 1: constant a(t)
%   Detailed explanation goes here

a = (2/tf^2)*s*ones(1, length(t));


end

