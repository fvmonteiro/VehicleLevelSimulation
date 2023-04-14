clearvars;
close all;
%% Parameters

m = 2000; % [kg] Total mass of vehicle
lf = 3; % [m] distance from cg to front wheel
lr = 2; % [m] distance from cg to rear wheel
% Default simulink values
Cf = 5e4;%5e4 or 12e3;
Cr = 4e4;%4e4 or 11e3;
Iz = 4000;

a1 = 2*(Cf+Cr)/m;
a2 = 2*(Cf*lf-Cr*lr)/m;
a3 = 2*(Cf*lf-Cr*lr)/Iz;
a4 = 2*(Cf*lf^2+Cr*lr^2)/Iz;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;
% C = [1, 0, 0, 0; 0, 0, 0, 1];

%% Controller
% Numerical robustness to variations in speed

poles = -[1, 2, 3, 4];

vxbar = 25;
Anom = statesMatrix2(vxbar, [a1, a2, a3, a4]); %[0 1 vxbar 0; 0 -a1/vxbar 0 -vxbar-a2/vxbar; 0 0 0 1; 0 -a3/vxbar 0 -a4/vxbar];
Bnom = [0; b1; 0; b2];

Klat = place(Anom, Bnom, poles);
epsilon = -0.5:0.01:0.5;
DeltaSkeleton = [0 0 0 0; 0 a1 0 a2; 0 0 0 0; 0 a3 0 a4];
lambda = zeros(length(Bnom), length(epsilon));
for d = 1:length(epsilon)
%     vx = vxbar*(1+ epsilon(d));
%     A = statesMatrix2(vx, [a1, a2, a3, a4]);
    A = Anom + epsilon(d)/(1+epsilon(d))/vxbar*DeltaSkeleton;
    lambda(:, d) = sort(real(eig(A - Bnom*Klat)));
end

plot(epsilon, max(lambda))
xlabel('\epsilon')
ylabel('max(Re(\lambda))')
grid

idx = find(max(lambda)>0, 1);
disp(epsilon(idx))

%% Controller
% Numerical robustness to variations in speed and cornering stiffness
poles = -[1, 2, 3, 4];

vxbar = 25;
a1 = 2*(Cf+Cr)/m;
a2 = 2*(Cf*lf-Cr*lr)/m;
a3 = 2*(Cf*lf-Cr*lr)/Iz;
a4 = 2*(Cf*lf^2+Cr*lr^2)/Iz;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;
Anom = statesMatrix2(vxbar, [a1, a2, a3, a4]); %[0 1 vxbar 0; 0 -a1/vxbar 0 -vxbar-a2/vxbar; 0 0 0 1; 0 -a3/vxbar 0 -a4/vxbar];
Bnom = [0; -b1; 0; -b2];

Klat = place(Anom, Bnom, poles);
epsilonV = -0.6:0.005:0.6;
epsilonC = (0.2:0.01:2)-1;
lambda = zeros(length(epsilonV), length(epsilonC));
for iterV = 1:length(epsilonV)
    for iterC = 1:length(epsilonC)
        vx = vxbar*(1+ epsilonV(iterV));
        Cfnew = (epsilonC(iterC)+1)*Cf;
        Crnew = (epsilonC(iterC)+1)*Cr;
        a1 = 2*(Cfnew+Crnew)/m;
        a2 = 2*(Cfnew*lf-Crnew*lr)/m;
        a3 = 2*(Cfnew*lf-Crnew*lr)/Iz;
        a4 = 2*(Cfnew*lf^2+Crnew*lr^2)/Iz;
        b1 = 2*Cfnew/m;
        b2 = 2*Cfnew*lf/Iz;
        A = statesMatrix2(vx, [a1, a2, a3, a4]);
        B = [0; -b1; 0; -b2];
        lambda(iterV, iterC) = max(real(eig(A - B*Klat)));
    end
end

firstUnstable = zeros(length(epsilonC), 1);
for iterC = 1:length(epsilonC)
    unstableIdx = find(lambda(:, iterC)>0, 1);
    if isempty(unstableIdx)
        firstUnstable(iterC) = length(epsilonV);
    else
        firstUnstable(iterC) = epsilonV(unstableIdx);
    end
end

figure; hold on;
imagesc(epsilonC, epsilonV, lambda);
c = colorbar;
c.Label.String = 'max[Re[\lambda(A-BK)]]';
plot(epsilonC, firstUnstable, 'r', 'LineWidth', 1.5)
legend('Stability limits')
xlim([min(epsilonC), max(epsilonC)]);
ylim([min(epsilonV), max(epsilonV)]);
xlabel('\alpha_c')
ylabel('\epsilon_v')

figure;
unstableMap = lambda>=0;
imagesc(epsilonC, epsilonV, unstableMap)
xlabel('C_f and C_r uncertainty')
ylabel('v_x uncertainty')
set(gca,'YDir','normal')
c = colorbar;
c.Ticks = [0 1];
c.TickLabels = {'Stable', 'Unstable'};
% plot(epsilonV, lambda)
% xlabel('\epsilon')
% ylabel('Re(\lambda)')
% grid

%% Controller
% Numerical robustness to different variations in cornering stiffness
poles = -[4, 1, 2, 3];

vxbar = 25;
a1 = 2*(Cf+Cr)/m;
a2 = 2*(Cf*lf-Cr*lr)/m;
a3 = 2*(Cf*lf-Cr*lr)/Iz;
a4 = 2*(Cf*lf^2+Cr*lr^2)/Iz;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;
Anom = statesMatrix2(vxbar, [a1, a2, a3, a4]);
Bnom = [0; -b1; 0; -b2];

Klat = place(Anom, Bnom, poles);
epsilonCf = (0.2:0.01:2)-1;
epsilonCr = (0.2:0.01:2)-1;
lambda = zeros(length(epsilonCf), length(epsilonCr));
for iterCf = 1:length(epsilonCf)
    for iterCr = 1:length(epsilonCr)
        vx = vxbar;
        Cfnew = (epsilonCf(iterCf)+1)*Cf;
        Crnew = (epsilonCr(iterCr)+1)*Cr;
        a1 = 2*(Cfnew+Crnew)/m;
        a2 = 2*(Cfnew*lf-Crnew*lr)/m;
        a3 = 2*(Cfnew*lf-Crnew*lr)/Iz;
        a4 = 2*(Cfnew*lf^2+Crnew*lr^2)/Iz;
        b1 = 2*Cfnew/m;
        b2 = 2*Cfnew*lf/Iz;
        A = statesMatrix2(vx, [a1, a2, a3, a4]);
        B = [0; -b1; 0; -b2];
        lambda(iterCf, iterCr) = max(real(eig(A - B*Klat)));
    end
end

firstUnstable = zeros(length(epsilonCr), 1);
for iterCr = 1:length(epsilonCr)
    unstableIdx = find(lambda(:, iterCr)>0, 1);
    if isempty(unstableIdx)
        firstUnstable(iterCr) = length(epsilonCf);
    else
        firstUnstable(iterCr) = epsilonCf(unstableIdx);
    end
end

figure; hold on;
imagesc(epsilonCr, epsilonCf, lambda);
c = colorbar;
c.Label.String = 'max(Re(\lambda))';
plot(epsilonCr, firstUnstable, 'r', 'LineWidth', 1.5)
legend('Stability limits')
xlim([min(epsilonCr), max(epsilonCr)]);
ylim([min(epsilonCf), max(epsilonCf)]);
xlabel('C_r uncertainty')
ylabel('C_f uncertainty')

figure;
unstableMap = lambda>=0;
imagesc(epsilonCr, epsilonCf, unstableMap)
xlabel('C_r uncertainty')
ylabel('C_f uncertainty')
set(gca,'YDir','normal')
c = colorbar;
c.Ticks = [0 1];
c.TickLabels = {'Stable', 'Unstable'};
% plot(epsilonV, lambda)
% xlabel('\epsilon')
% ylabel('Re(\lambda)')
% grid

%% Checking robustness through Lyapunov function and equation
Q = eye(4);

Anom = statesMatrix2(vxbar, [a1, a2, a3, a4]); %[0 1 vxbar 0; 0 -a1/vxbar 0 -vxbar-a2/vxbar; 0 0 0 1; 0 -a3/vxbar 0 -a4/vxbar];
Bnom = [0; b1; 0; b2];

epsilon = 0.1;
Delta = epsilon/(1+epsilon)/vxbar*[0 0 0 0; 0 a1 0 a2; 0 0 0 0; 0 a3 0 a4];

poles = -[1, 2, 3, 4];
maxK = 50;
maxEig = zeros(maxK, 1);
for k = 1:maxK
    Klat = place(Anom, Bnom, poles*k);
    Acl = Anom-Bnom*Klat;
    
    P = lyap(Acl, Q);
    DeltaQ = P*Delta+Delta'*P;
    maxEig(k) = max(eig(DeltaQ));
    if maxEig(k)<1
        disp(['Robust to epsilon = ' num2str(epsilon)]);
        disp(['k = ' num2str(k)]);
        break
    end
end
if k==maxK
    disp(['Max k = ' num2str(k)])
    disp(['Not robust to epsilon = ' num2str(epsilon)]);
    plot(1:maxK, maxEig) 
end

%% Analytical state space
% clearvars
asym = sym('a', [4, 1], 'positive');
ksym = sym('k', [1, 4], 'real');
bsym = sym('b', [2, 1], 'positive');
syms vx %p1 p2 p3 p4
A = statesMatrix(vx, asym);
B = [0; bsym(1); 0; bsym(2)];
K = ksym;
Acl = A-B*K;
ceqOL = charpoly(A);
ceqCL = charpoly(Acl);
% poles = (lambda-p1)*(lambda-p2)*(lambda-p3)*(lambda-p4);

% ceq_num = subs(ceq_sym, [a1s, a2s, a3s, a4s, b1s, b2s, k1, k2, k3, k4], [a1, a2, a3, a4, b1, b2, 7, 1, 9, 10]);

%% Analytical freq domain

asym = sym('a', [4, 1]);
ksym = sym('k', [1, 4]);
bsym = sym('b', [2, 1]);
syms vx d s%p1 p2 p3 p4

A = statesMatrix(vx, asym);
B = [0; bsym(1); 0; bsym(2)];
K = ksym;

ceqOL = charpoly(A);

C = [1 0 d 0];
G = C*((s*eye(4)-A)\B);

%% Disturbance transfer function
syms alpha0 alpha1 beta0 beta1 beta2 a3 rho s vx
G_true = (s^2*(vx^2+2*rho*vx)*beta2+s*(vx+rho)*beta1 + (vx^2+2*rho*vx)*beta0)/(s^2*(s^2*(vx^2+2*rho*vx)+(vx+rho)*alpha1*s + alpha0 - a3*(vx^2+2*rho*vx)));
G = subs(G_true, rho, 0);
%% Checking algebra
% clearvars
% syms Cr Cf lr lf vx m Iz alpha beta
% a1 = (Cf + Cr)/m;
% a2 = (Cf*lf - Cr*lr)/m;
% a3 = (Cf*lf - Cr*lr)/Iz;
% a4 = (Cf*lf^2 + Cr*lr^2)/Iz;
% b1 = Cf/m;
% b2 = Cf*lf/Iz;
% 
% coeffs = [1, -(a1+a4)*beta, (a1*a4-a2*a3)*beta^2];

%% Robustness wrt cornering stiffness (analytical)
asym = sym('a', [4, 1]);
delta = sym('delta', [4, 1]);
b = sym('b', [2 1]);
k = sym('k', [1, 4]);
syms alpha vx

A = statesMatrix(vx, asym);
B = [0; -b(1); 0; -b(2)];


deltaA = [0 0 0 0; 0 -delta(1)/vx 0 -delta(2)/vx; 0 0 0 0; 0 -delta(3)/vx 0 -delta(4)/vx];
deltaB = (alpha-1)*B;
deltaCL = deltaA + deltaB*k;
ceqDeltaCL = charpoly(deltaCL);
 
ACLalpha = A + alpha*B*k;
ceqCLalpha = charpoly(ACLalpha);

%% Robustness wrt cornering stiffness (numerical)

poles = -[0.5, 1, 2, 3];

a1 = 2*(Cf+Cr)/m;
a2 = 2*(Cf*lf-Cr*lr)/m;
a3 = 2*(Cf*lf-Cr*lr)/Iz;
a4 = 2*(Cf*lf^2+Cr*lr^2)/Iz;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;
vx = 25;
Anom = statesMatrix2(vx, [a1, a2, a3, a4]); %[0 1 vxbar 0; 0 -a1/vxbar 0 -vxbar-a2/vxbar; 0 0 0 1; 0 -a3/vxbar 0 -a4/vxbar];
Bnom = [0; -b1; 0; -b2];

Klat = place(Anom, Bnom, poles);

alpha = 0.1:0.001:2;
beta = alpha;
lambda = zeros(length(Bnom), length(alpha));

for k = 1:length(alpha)
    Cfnew = alpha(k)*Cf;
    Crnew = beta(k)*Cr;
    a1 = 2*(Cfnew+Crnew)/m;
    a2 = 2*(Cfnew*lf-Crnew*lr)/m;
    a3 = 2*(Cfnew*lf-Crnew*lr)/Iz;
    a4 = 2*(Cfnew*lf^2+Crnew*lr^2)/Iz;
    b1 = 2*Cfnew/m;
    b2 = 2*Cfnew*lf/Iz;
    A = statesMatrix2(vx, [a1, a2, a3, a4]);
    B = [0; -b1; 0; -b2];
    lambda(:, k) = sort(real(eig(A - B*Klat)));
end

figure;
plot(alpha, real(lambda))
grid

idx = find(max(real(lambda))>0, 1, 'last');
disp(alpha(idx))

%% New state space
asym = sym('a', [4, 1], 'positive');
epsilon = sym('delta', [4, 1]);
bsym = sym('b', [2, 1], 'positive');
k = sym('k', [1, 4]);
syms vx positive

A = statesMatrix2(vx, asym);
B = [0; bsym(1); 0; bsym(2)];

%% Support function
function A = statesMatrix2(vx, params)

A = [0 1 0 0; 
    0 -params(1)/vx params(1) -params(2)/vx; 
    0 0 0 1; 
    0 -params(3)/vx params(3) -params(4)/vx];

end

function A = statesMatrix(vx, params)
% warning('old state space realization')
A = [0, 1, vx, 0; 
%     0, -params(1)/vx, 0, -vx-params(2)/vx; 
    0, -params(1), 0, -vx-params(2); 
    0, 0, 0, 1; 
%     0, -params(3)/vx, 0, -params(4)/vx];
    0, -params(3), 0, -params(4)];
end