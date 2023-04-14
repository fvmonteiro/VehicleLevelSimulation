%%% Van der Pol oscilator with failures from Polycarpou papers

clearvars;
close all;

% Oscilator parameters
x0 = [1; 0];
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
T = 20;
deltaT = 0.1;
simTime = 0:deltaT:T;

y0 = [x0;xHat0; theta0];
r = 0;

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

vdp = @(t,x) vanDerPolWithFailure(t, x, vdpParams, 'sin', p, approximatorParams, []);
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

%% Plot solution
figure;
plot(tResult,x(:,1))
xlabel('t')
ylabel('solution y')
title('van der Pol')

figure;
plot(tResult, sqrt(e(:,1).^2+e(:, 2).^2))
xlabel('t')
ylabel('|e|')
title('error norm')

figure;
plot(tResult, abs(fHat))
xlabel('t')
ylabel('|f hat|')
title('OLA')
