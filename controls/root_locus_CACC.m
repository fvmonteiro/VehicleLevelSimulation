%% Root locus of CACC controller

tau = 0.5;
h = 0.6;
ka = 1; %1.1*tau/h;
alpha = 0.9;
polOrder = 3; %4;

rangeK = 0:0.0001:1;
realPart = zeros(length(rangeK), polOrder);
imagPart = zeros(length(rangeK), polOrder);

for n = 1:length(rangeK)
    k = rangeK(n);
%     ki = alpha*k^2/(2*(h*(ka*k*h+ka+1)-tau));
%     coeff = [tau, ka+h*ka*k+1, 1/h+k, k/h+h*ki, ki];
    coeff = [tau, ka+h*ka*k+1, 1/h+k, k/h];
    z = roots(coeff);
    [realPart(n, :), idx] = sort(real(z));
    imagPart(n, :) = imag(z(idx));
end

figure;
hold on;
grid on;
for n = 1:polOrder
    plot(realPart(:, n), imagPart(:, n));
end