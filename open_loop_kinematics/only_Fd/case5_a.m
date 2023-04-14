function [a, alpha] = case5_a(t, tf, s, v0, vf, a0, af, jerk0, jerkf)
%case5_a Compute a(t)
%   Detailed explanation goes here

np = 5; % polynomial order
n_coeff = np+1; % number of coefficients = polynomial order + 1
n_const = 6; % the number of constraints is fixed

Tf = zeros(n_const, n_coeff);
n = 0:np;
Tf(1, 1) = 1;
Tf(2, 2) = 1;
Tf(3, :) = tf.^(n);
Tf(4, :) = n.*tf.^(n-1);
Tf(5, :) = tf.^(n+1)./(n+1);
Tf(6, :) = tf.^(n+2)./((n+1).*(n+2));

b = [a0; jerk0; af; jerkf; vf - v0 ; s];

alpha = Tf\b;

powers_of_t = ones(n_coeff, length(t));
for n = 2:n_coeff
    powers_of_t(n, :) = powers_of_t(n-1, :).*t;
end

a = alpha'*powers_of_t;

end

