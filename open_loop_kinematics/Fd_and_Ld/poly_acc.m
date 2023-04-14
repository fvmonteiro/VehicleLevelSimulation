function [at] = poly_acc(t, coeff, n_coeff)
%poly_acc Acceleration as a function of time given polynomial coefficients
%   Detailed explanation goes here

powers_of_t = ones(n_coeff, length(t));
for k = 2:n_coeff
    powers_of_t(k, :) = powers_of_t(k-1, :).*t;
end

at = (reshape(coeff, n_coeff, [])'*powers_of_t)'; % one row for each vehicle

end

