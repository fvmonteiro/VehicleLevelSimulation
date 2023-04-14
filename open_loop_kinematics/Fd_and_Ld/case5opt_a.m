function [at] = case5opt_a(t, tf, d, v0, vf, a0, af, jerk0, jerkf, weights, np)
%case5_a Compute a(t) as a polynomial with optimization
%   Detailed explanation goes here

n_coeff = np+1; % number of coefficients = polynomial order + 1
nv = length(v0); % number of vehicles

% Eq constraints (initial and terminal conditions)
[Aeq, beq] = create_eq_constr(np, tf, d, v0, vf, a0, af, jerk0, jerkf);
% Cost function
Q = cost_matrix(np, nv, tf, weights);
M = [Q Aeq';Aeq zeros(size(Aeq,1))];

% Solve system
x_lambda = M\[zeros(nv*n_coeff,1);beq];

x = x_lambda(1:nv*n_coeff);

at = poly_acc(t, x, np+1);

end

