% Root-locus and performance of vehicle following controller

%% Trying to set the poles
clearvars;
tau = 0.4;
h = 0.6;
overshoot = 0.1;
ts = 30; % settling time
zeta = -log(overshoot)/sqrt(pi^2+log(overshoot)^2);
wn = 4/zeta/ts;
lambda0 = 5*wn*zeta;

delta = sqrt(1-zeta^2);
poles = [lambda0, wn*(zeta+delta*1i), wn*(zeta-delta*1i)];

C = [1 h 0; 0 1 h; 0 0 1];
d = [sum(poles); poles(1)*poles(2)+poles(1)*poles(3)+poles(2)*poles(3); ...
    prod(poles)]*tau;
A = [-2 -h 0];
b = -(2*tau/h-1);
lb = zeros(length(poles), 1);
ub = Inf*ones(length(poles), 1);
options = optimoptions('lsqlin','Display','off');
x = lsqlin(C, d, A, b, [], [], lb, ub, [], options);

kg = x(3);
kv = 1/h;
kd = x(2);
ka = x(1);

% Check final performance and double-check string stability
numLeaderVel = [tau-h*ka, 1-h*kv, 0];
den = [tau, ka+h*kd+1, kd+kv+h*kg, kg];
tfLeaderVel = tf(numLeaderVel, den);
figure;
step(tfLeaderVel);

numGapErrors = [ka, (kv+kd), kg];
tfGapErrors = tf(numGapErrors, den);
figure;
step(tfGapErrors); % same as vel to vel
figure;
bode(tfGapErrors)

%% Coding it all
% close all
clearvars

% Conditions: ka>tau/h, kd>=0, kv = 1/h, kg>0
polOrder = 3;

tau = 0.3;
h = 0.6;
ka = 1.1*tau/h;
kv = 1/h;
kdArray = 0.1; %0:1:3;
kgArray = 0.1:.001:10;

realPart = zeros(length(kdArray), length(kdArray), polOrder);
imagPart = zeros(length(kdArray), length(kdArray), polOrder);

for m = 1:length(kdArray)
    kd = kdArray(m);
    for n = 1:length(kgArray)
        kg = kgArray(n);
        coeffs = [tau, ka+h*kd+1, kd+kv+h*kg, kg];
        z = roots(coeffs);
        [realPart(m, n, :), idx] = sort(real(z));
        imagPart(m, n, :) = imag(z(idx));
    end
end

figure;
hold on;
grid on;
lineStyle = {'-', '--', ':', '-.'};
for m = 1:length(kdArray)
    for p = 1:polOrder
        plot(realPart(m, :, p), imagPart(m, :, p), lineStyle{m}, ...
            'LineWidth', 1, 'SeriesIndex', p);
    end
end

%% Using the rlocus function: one plot per kd

% close all
clearvars

% Conditions: ka>tau/h, kd>=0, kv = 1/h, kg>0
polOrder = 3;

tau = 0.4;
h = 0.6;
etaArray = [-1/3, 0, 0.1*tau/h, 4*tau/h];
kv = 1/h;

kdArray = [0, 0.2, 1, 3];
% kgRootLocusFixedKa(tau, h, kv, 1.1*tau/h, kdArray)

kd = 0.2;
kgRootLocusFixedKd(tau, h, kv, etaArray, kd)

kgArray = 0:3;
% kdRootLocus(tau, h, kv, etaArray, kgArray)

%% Functions
function [] = kgRootLocusFixedKa(tau, h, kv, ka, kdArray)
    num = [h 1];
    sys = tf(zeros(1,1, length(kdArray)));
    % Create TFs
    for m = 1:length(kdArray)
        kd = kdArray(m);
        den = [tau, ka+h*kd+1, kd+kv, 0];
        sys(:, :, m) = tf(num, den);
    end
    
    % Plot using rlocus function
    figure;
    M = length(kdArray);
    lineColor = {'c', 'b', 'k', 'r', 'm'};
    leg = cell(1, length(kdArray));
    hold on;
    for m = 1:M
        if M>1 % color different kd's differently
            rlocus(sys(:, :, m), ...
                lineColor{rem(m, length(lineColor))+1})
        else % color each root differently
            rlocus(sys(:, :, m), 'LineWidth', 1)
        end
        leg{m} = ['$kd = $' num2str(kdArray(m))];
    end
    title('$k_a = 1.1 \tau/h$', 'Interpreter', 'latex', 'Fontsize', 20)
    legend(leg, 'Interpreter', 'latex', 'Fontsize', 20)
end


function [] = kgRootLocusFixedKd(tau, h, kv, etaArray, kd)
    num = [h 1];
    sys = tf(zeros(1,1, length(etaArray)));
    % Create TFs
    for n = 1:length(etaArray)
        ka = tau/h + etaArray(n);
        den = [tau, ka+h*kd+1, kd+kv, 0];
        sys(:, :, n) = tf(num, den);
    end
    
    % Plot using rlocus function
    figure;
    N = length(etaArray);
    lineColor = {'c', 'b', 'k', 'r', 'm'};
    % Special legend for paper plot
    leg = {'$\eta = - 1/3$', '$\eta = 0$', ...
            '$\eta = 0.1\tau/h$', '$\eta = 4\tau/h$'};
    hold on;
    for n = 1:N
        if N>1 % color different ka's differently
            rlocus(sys(:, :, n), ...
                lineColor{rem(n, length(lineColor))+1})
        else % color each root differently
            rlocus(sys(:, :, n))
        end 
%         leg{n} = ['ka = ' num2str(kdArray(n))];
    end
    title(['$k_d = ' num2str(kd) '$'], 'Interpreter', 'latex', ...
        'Fontsize', 20)
    legend(leg, 'Interpreter', 'latex', 'Fontsize', 20)
end


function [] = kgRootLocusAll(tau, h, kv, etaArray, kdArray)
    kaArray = tau/h + etaArray;
    num = [h 1];
    sys = tf(zeros(1,1,length(kaArray), length(kdArray)));
    for n = 1:length(kaArray)
        % Create TFs
        ka = kaArray(n);

        for m = 1:length(kdArray)
            kd = kdArray(m);
            den = [tau, ka+h*kd+1, kd+kv, 0];
            sys(:, :, n, m) = tf(num, den);
        end
    end

    % Plot using rlocus function
    figure;
    N = length(kaArray);
    M = length(kdArray);
    for n = 1:N
        subplot(ceil(N/2), max(1, N-ceil(N/2)), n)
        % lineColor = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', ...
        %     '#4DBEEE', '#A2142F'};
        lineColor = {'c', 'b', 'k', 'r', 'm'};
        leg = cell(1, length(kdArray));
        % Special titles for paper plot
        titleStr = {'$k_a = \tau/h - 1/3$', '$k_a = \tau/h$', ...
            '$k_a = 1.1 \tau/h$', '$k_a = 5\tau/h$'}; 
        hold on;

        for m = 1:M
            if M>1 % color different kd's differently
                rlocus(sys(:, :, n, m), ...
                    lineColor{rem(m, length(lineColor))+1})
            else % color each root differently
                rlocus(sys(:, :, n, m))
            end
            leg{m} = ['kd = ' num2str(kdArray(m))];
        end
%         titleStr = ['$k_a = \tau/h + ' num2str(etaArray(n)) '$'];
        title(titleStr{n}, 'Interpreter', 'latex')
        legend(leg)
    end
end

function [] = kdRootLocus(tau, h, kv, etaArray, kgArray)
    kaArray = tau/h + etaArray;
    num = [h 1 0];
    sys = tf(zeros(1,1,length(kaArray), length(kgArray)));
    for n = 1:length(kaArray)
        % Create TFs
        ka = kaArray(n);

        for m = 1:length(kgArray)
            kg = kgArray(m);
            den = [tau, ka+1, kv+h*kg, kg];
            sys(:, :, n, m) = tf(num, den);
        end
    end

    % Plot using rlocus function
    figure;
    N = length(kaArray);
    M = length(kgArray);
    for n = 1:N
        subplot(ceil(N/2), max(1, N-ceil(N/2)), n)
        % lineColor = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', ...
        %     '#4DBEEE', '#A2142F'};
        lineColor = {'c', 'b', 'k', 'r', 'm'};
        leg = cell(1, length(kgArray));
        hold on;

        for m = 1:M
            rlocus(sys(:, :, n, m), lineColor{rem(m, length(lineColor))+1})
            leg{m} = ['kg = ' num2str(kgArray(m))];
        end
        title(['$ka = \tau/h + $' num2str(etaArray(n))], ...
            'Interpreter', 'latex')
        legend(leg)
    end
end