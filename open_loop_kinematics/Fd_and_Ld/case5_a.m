function [at] = case5_a(t, tf, d, v0, vf, a0, af, jerk0, jerkf)
%case5_a Compute a(t) for leading and following vehicle.
%   Returns a matrix whose first column is the leading acceleration and the
% second is the following acceleration.

np = 5; % polynomial order

% Eq constraints (initial and terminal conditions)
[Aeq, beq] = create_eq_constr(np, tf, d, v0, vf, a0, af, jerk0, jerkf);
M = Aeq;
% Solve system
x = M\beq;

at = poly_acc(t, x, np+1);

end

