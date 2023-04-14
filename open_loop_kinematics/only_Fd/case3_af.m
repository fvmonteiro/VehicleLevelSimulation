function af = case3_af(tf, s, a0, jerk)
%case3_a Compute a(t) for case 3
%   Detailed explanation goes here

alpha0 = -(a0^2*(a0/3+tf*jerk) + 2*s*jerk^2);
alpha1 = (a0+tf*jerk)^2;
alpha2 = -(a0+tf*jerk);
alpha3 = 1/3;

af = roots([alpha3 alpha2 alpha1 alpha0]);

% We return the real solution
af = af(imag(af)==0);

end
