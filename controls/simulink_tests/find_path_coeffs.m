function [y] = find_path_coeffs(tf, x0, xT)

n = length(x0);
A = zeros(2*n);
% Lazy solution
if n == 2 % speed references
    A(1, 1) = 1;
    A(2, 2) = 1;
    for i = 1:4
        A(3, i) = tf^(i-1);
        A(4, i) = (i-1)/tf * A(3, i);
    end
elseif n == 3 % position references
    A(1, 1) = 1;
    A(2, 2) = 1;
    A(3, 3) = 2;
    for i = 1:6
        A(4, i) = tf^(i-1);
        A(5, i) = (i-1)/tf * A(4, i);
        A(6, i) = (i-2)/tf * A(5, i);
    end
end
b = [x0;xT];
y = A\b;

end