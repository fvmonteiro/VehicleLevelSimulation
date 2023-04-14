function [at] = case5constr_a(t, tf, d, v0, vf, a0, af, jerk0, jerkf, ...
    v_bounds, a_bounds, jerk_bounds, weights, np)
%case5_a Compute a(t) as a polynomial with optimization and constraints
%   Detailed explanation goes here

%% Equality constraints and x0
n_coeff = np+1; % number of coefficients = polynomial order + 1
nv = length(v0); % number of vehicles

% Eq constraints (initial and terminal conditions)
[Aeq, beq] = create_eq_constr(np, tf, d, v0, vf, a0, af, jerk0, jerkf);
% Cost function
Q = cost_matrix(np, nv, tf, weights);
M = [Q Aeq';Aeq zeros(size(Aeq,1))];

% Solution of the problem with only equality constraints is used as
% starting point
x0_lambda = M\[zeros(nv*n_coeff,1);beq];
x0 = x0_lambda(1:nv*n_coeff);
% TODO: decide on x0
% x0 = zeros(nv*n_coeff, 1);

%% Inequality constraints
[Aineq, bineq] = create_ineq_constr(np, tf, v0, a_bounds, jerk_bounds, v_bounds);

%% Solve optimization problem
costfun = @(x) x'*Q*x/2;
%options = optimoptions('fmincon', 'MaxFunctionEvaluations', 10000);
x = fmincon(costfun, x0, Aineq, bineq, Aeq, beq);%,...
 %   [], [], [], options);

at = poly_acc(t, x, np+1);

end