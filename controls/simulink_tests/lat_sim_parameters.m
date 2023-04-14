clearvars;

%% Parameters

% Vehicle
m = 2000; % [kg] Total mass of vehicle
lf = 3; % [m] distance from cg to front wheel
lr = 2; % [m] distance from cg to rear wheel
Cf = 12e3; %5e4; % front tire stiffness
Cr = 11e3; %4e4; % rear tire stiffness
Iz = 4000; % moment of inertia around vertical axis
d = 5; % look ahead distance for measuring error

vx0 = 20; % desired speed

% Controller
[A, B] = getLateralDynamics(vx0, Cf, Cr, lr, lf, m, Iz);
C = [1 0 d 0];
poles = -[1, 2, 3, 4]*2;
Klat = place(A, B, poles);


%% Error on Cf and Cr
alpha = 1;
beta = 1;

CfReal = alpha*Cf;
CrReal = beta*Cr;

%% Numerically checking controller robustness
rhoArray = -0.5:0.01:0.5;
lambdaReal = zeros(length(B), length(rhoArray));
unstable_idx = 0;
for d = 1:length(rhoArray)
    vxReal = vx0*(1 + rhoArray(d));
    [AerrorReal, ~] = getLateralDynamics(vxReal, Cf, Cr, lr, lf, m, Iz);
    lambdaReal(:, d) = real(eig(AerrorReal - B*Klat));
    if any(lambdaReal(:, d)>0) && unstable_idx==0
        unstable_idx = d;
    end
end

% plot(rhoArray, real(lambdaReal))
% grid
if unstable_idx>0
    fprintf('System unstable at rho = %.2f\n', rhoArray(unstable_idx));
end