%%% Van der Pol oscilator with failures from Polycarpou papers

clearvars;
close all;

% Oscilator parameters
x0 = [0.2; 0.1];
w = 0.9;
psi = 0.6;
mu = 0.95;
vdpParams = [w, psi, mu];

% Approximator parameters (RBF network)
xHat0 = [0; 0];
N = 11;
theta0 = zeros(N, 1); % parameters to be learned
sigma = 0.6;
c = linspace(-2, 2, N);
gamma = 10;
approximatorParams = [gamma, c, sigma];
p = 1;

% testing ode
T = 50;
deltaT = 0.1;
simTime = 0:deltaT:T;

y0 = [x0;xHat0; theta0];

% Reference
r = 0.5 + 0.6*sin(simTime) - 0.3*sin(3*simTime);
W = tf(1, [1 4 3]);
yR = lsim(W, r, simTime);

% tResult = [];
% yResult = [];
% for k = 2:length(simTime)
% 
%     % Integrate:
%     vdp = @(t,x) vanDerPolWithFailure(t, x, vdpParams, u, 'sin', p, approximatorParams);
%     t = simTime(k-1:k);
%     [t, tempX] = ode45(vdp, t, y0);
%       
%     % Collect results
%     tResult = cat(1, tResult, t);
%     yResult = cat(1, yResult, tempX);
%     
%     % Initial value update
%     y0 = tempX(end,:)';
% end

%%%% Paper says yR-r, but it is yR+r that works. 
% Without reconfiguration
vdp = @(t,x) vanDerPolWithFailure(t, x, vdpParams, 't*sin', p, approximatorParams, [simTime', yR+r'], 0);
[tResult, yResult] = ode45(vdp, simTime([1, end]), y0);

x = yResult(:, 1:2);
xHat = yResult(:, 3:4);
e = x - xHat;
theta = yResult(:, 5:end);
rbf = exp(-(x(:, 1)-c).^2/sigma^2);
fHat = zeros(length(tResult), 1);
for k = 1:length(tResult)
    fHat(k) = theta(k,:)*rbf(k, :)';
end

% With reconfiguration
vdp = @(t,x) vanDerPolWithFailure(t, x, vdpParams, 't*sin', p, approximatorParams, [simTime', yR+r'], 1);
[tResultFtc, yResultFtc] = ode45(vdp, simTime([1, end]), y0);

xFtc = yResultFtc(:, 1:2);
xHatFtc = yResultFtc(:, 3:4);
eFtc = xFtc - xHatFtc;
thetaFtc = yResultFtc(:, 5:end);
rbf = exp(-(xFtc(:, 1)-c).^2/sigma^2);
fHatFtc = zeros(length(tResultFtc), 1);
for k = 1:length(tResultFtc)
    fHatFtc(k) = theta(k,:)*rbf(k, :)';
end

%% Plot solution
figure; 
hold on; grid on;
plot(simTime,yR)
plot(tResult, x(:, 1));
plot(tResultFtc, xFtc(:, 1));
xlabel('t')
ylabel('y and y_R')
title('van der Pol')

legend({'reference', 'no reconfiguration', 'with reconfiguration'})