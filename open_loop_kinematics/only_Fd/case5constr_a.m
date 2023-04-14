function [a, alpha] = case5constr_a(t, tf, s, v0, vf, a0, af, jerk0, jerkf, ...
    v_bounds, a_bounds, jerk_bounds, np)
%case5_a Compute a(t) as a polynomial with optimization and constraints
%   Detailed explanation goes here

n_coeff = np+1; % number of coefficients = polynomial order + 1

%% Cost function
Q = zeros(n_coeff, n_coeff);
for i = 1:np
    for j = 1:np
        Q(i+1,j+1) = i*j*tf^(i+j-1)/(i+j-1);
    end
end

costfun = @(alpha)alpha'*Q*alpha;

%% Constraints
% Equality
neq_const = 6; % the number of equality constraints is fixed
n = 0:np;
Tf = zeros(neq_const, n_coeff);
Tf(1, 1) = 1;
Tf(2, 2) = 1;
Tf(3, :) = tf.^n;
Tf(4, :) = (n).*tf.^(n-1);
Tf(5, :) = tf.^(n+1)./(n+1);
Tf(6, :) = tf.^(n+2)./((n+1).*(n+2));

b = [a0; jerk0; af; jerkf; vf - v0; s];

% Inequality
% We create a 3D matrix - each 2D matrix represents the constraints for a
% single sampled time instant. Afterwards we transform this in a single 2D
% matrix to properly pass it to the optimizer
nineq_const = 3; % acceleration, jerk and velocity
delta = 0.1; % maybe using the original sampling time creates too many constraints
tk = 0:delta:tf;
K = length(tk);
Tk = zeros(nineq_const, n_coeff, K);
for k = 1:K
    Tk(1, :, k) = tk(k).^n;
    Tk(2, :, k) = (n).*tk(k).^(n-1);
    Tk(3, :, k) = tk(k).^(n+1)./(n+1);
end

Tk = permute(Tk, [1 3 2]);
Tk = reshape(Tk, 3*K, n_coeff);

lower_bounds = [a_bounds(1); jerk_bounds(1); v_bounds(1)-v0];
lower_bounds = repmat(lower_bounds, K, 1);
%lower_bounds(3:3:end) = lower_bounds(3:3:end) - v0*tk';
upper_bounds = [a_bounds(2); jerk_bounds(2); v_bounds(2)-v0];
upper_bounds = repmat(upper_bounds, K, 1);
%upper_bounds(3:3:end) = upper_bounds(3:3:end) - v0*tk';

%% Solving the system
M = [Q Tf';Tf zeros(neq_const)];
x0_lambda = M\[zeros(n_coeff,1);b];
x0 = x0_lambda(1:n_coeff); % initial guess is the solution of unconstrained problem
% lambda = alpha_lambda(n_coeff:end);
%x0 = zeros(n_coeff, 1);

%options = optimoptions('fmincon', 'MaxFunctionEvaluations', 10000);
alpha = fmincon(costfun, x0, [Tk; -Tk], [upper_bounds; -lower_bounds], Tf, b);%,...
 %   [], [], [], options);
%% Getting a(t)
powers_of_t = ones(n_coeff, length(t));
for n = 2:n_coeff
    powers_of_t(n, :) = powers_of_t(n-1, :).*t;
end
a = alpha'*powers_of_t;

end