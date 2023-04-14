function [a, alpha] = case5opt_a(t, tf, s, v0, vf, a0, af, jerk0, jerkf, np)
%case5_a Compute a(t) as a polynomial with optimization
%   Detailed explanation goes here

n_coeff = np+1; % number of coefficients = polynomial order + 1

% Cost function matrix
Q = zeros(n_coeff, n_coeff);
for i = 1:np
    for j = 1:np
        Q(i+1,j+1) = i*j*tf^(i+j-1)/(i+j-1);
    end
end

% Constraint matrix
n_const = 6; % the number of constraints is fixed
n = 0:np;
T = zeros(n_const, n_coeff);
T(1, 1) = 1;
T(2, 2) = 1;
T(3, :) = tf.^n;
T(4, :) = (n).*tf.^(n-1);
T(5, :) = tf.^(n+1)./(n+1);
T(6, :) = tf.^(n+2)./((n+1).*(n+2));

b = [a0; jerk0; af; jerkf; vf - v0; s];

% Solving the system
A = [Q T';T zeros(n_const)];
alpha_lambda = A\[zeros(n_coeff,1);b];


alpha = alpha_lambda(1:n_coeff);
% lambda = alpha_lambda(n_coeff:end);

powers_of_t = ones(n_coeff, length(t));
for n = 2:n_coeff
    powers_of_t(n, :) = powers_of_t(n-1, :).*t;
end

a = alpha'*powers_of_t;


%% Checking the cost
% sampling_period = t(2)-t(1);
% % Numerically
% jerk_num = diff(a)/sampling_period;
% cost_num = trapz(sampling_period, jerk_num.^2);
% 
% % Analytical steps
% jerk_an = (alpha(2:end)'.*(1:np))*powers_of_t(1:end-1, :);
% cost1 = trapz(sampling_period, jerk_an.^2); % ok
% jerk_square = zeros(length(t), 1);
% for k = 1:length(t)
%     for i = 1:np
%         for j = 1:np
%             jerk_square(k) = jerk_square(k) + i*j*alpha(i+1)*alpha(j+1)*t(k)^(i+j-2);
%         end
%     end
% end
% cost2 = trapz(sampling_period, jerk_square); % ok
% 
% cost3 = 0;
% for i = 1:np
%     for j = 1:np
%         cost3 = cost3 + i*j*tf^(i+j-1)*alpha(i+1)*alpha(j+1)/(i+j-1);
%     end
% end
% 
% cost4 = alpha(2:end)'*Q*alpha(2:end);
% 
% fprintf(' Numerical cost: %.3f \n Step 1: %.3f\n Step 2: %.3f\n Step 3: %.3f\n Step 4: %.3f\n', ...
%     cost_num, cost1, cost2, cost3, cost4)

end

