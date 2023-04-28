function [y] = findPathCoefficients(tf, x0, xT)

A = zeros(6);
A(1, 1) = 1;
A(2, 2) = 1;
A(3, 3) = 2;
for i = 1:6
    A(4, i) = tf^(i-1);
    A(5, i) = (i-1)/tf * A(4, i);
    A(6, i) = (i-2)/tf * A(5, i);
end
b = [x0;xT];
y = A\b; 

end