function [jerk] = case3_jerk(s, tf, a0, af)
%case3_jerk Compute possible jerk given intended final acceleration and
%total maneuver time

%alpha0 = (af-a0)^2*(2*af+a0)/3;
alpha0 = (af-a0)^3/6;
%alpha1 = -af*(af-a0)*tf;
alpha1 = -(af-a0)^2*tf/2;
alpha2 = af*tf^2/2-s;

delta = alpha1^2 - 4*alpha0*alpha2; % delta = b^2 - 4ac
if delta < 0
    jerk = [];
    fprintf('No feasible jerk \n');
else
    pol = [alpha2 alpha1 alpha0];
    jerk = roots(pol);
end

% New attempt
% syms x
% t1 = (af-a0)/x;
% eqn = t1^2*a0/2 + t1^3*x/6 + (tf-t1)^2*af/2 - s == 0;
% jerk_alt = solve(eqn, x);

% eqn2 = a0*t1*tf - a0*t1^2/2 + x*t1^2*tf/2 - x*t1^3/3 + (tf-t1)^2*af/2 - s == 0;

end