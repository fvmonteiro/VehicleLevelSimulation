%%%% Testing risk cost from MIT paper
% paper 1: Navigating Congested Environments with Risk Level Sets
% paper 2: Learning Risk Level Set Parameters from Data Sets for Safer Driving

%% Preamble and constants
clearvars;
close all;

kmhToms = 1000/3600;
maxVel = 130*kmhToms; % [m/s]
maxBrake = -5; % [m/s^2]
maxJerk = 50; % [m/s^3]
delay = 1; % [s]

%% Ego vehicle
egoX = 0;
egoY = 0;
egoVx = 30; %90*kmhToms;
egoVy = 0;
egoAccel = 0;

%% Vehicle
% Dimensions [m]
length = 4; 
width = 2; 
% Position [m]
vehX = 20;
vehY = 0;
% Velocities [m/s]
velX = 25; %100*kmhToms;
velY = 0;
% Acceleration [m/s^2]
accelX = 0;

%% Risk function parameters
% Vehicle radius [m] - there's a collision if distance<=rc
rc = 2.5; % not defined in papers
% Max speed [m/s]

% Variance
sigmax2 = length/2 + abs(velX-egoVx); % not specified in paper 1, vehWdith/2+|vel| in paper 2
sigmay2 = width/2 + abs(velY-egoVy); % not specified in paper 1 or 2
sigmaMax2 = max(sigmax2, sigmay2);
invCovMatrix = diag([1/sigmax2, 1/sigmay2]);

beta = 1.5; % 1 (paper 1) for regular Gaussian or 1.5 (paper 2) from human drivers data
alpha = 0.8; % not specified in paper 1, 0.8 from human drivers data in paper 2
% Paper 1 specifies a condition for alpha:
if alpha*maxVel*exp(-alpha*maxVel*rc)/(1+exp(-alpha*maxVel*rc)) > 2*rc/sigmaMax2
    warning('Not approppriate choice of alpha')
    return;
end

%% Max cost before collision
maxCollRisk = ellipticalRiskCost(rc, 0, [maxVel-egoVx; 0], diag([1/sigmaMax2, 1/sigmaMax2]), alpha, beta);

%% Threshold to avoid collisions
% These kinematic equations are only valid if egoAccel=0
relVel = velX-egoVx;
relAccel = accelX-egoAccel;
velAfterDelay = relVel+relAccel*delay;
% dist1 = (velAfterDelay^2-relVel^2)/2/relAccel;
dist1 = relVel*delay + 1/2*relAccel*delay^2;
tToMaxBrake = -(maxBrake-accelX)/maxJerk;
dist2 = velAfterDelay*tToMaxBrake + 1/2*relAccel*tToMaxBrake^2 ...
    - 1/6*maxJerk*tToMaxBrake^3;
velAtFullBrake = relVel + relAccel*(delay+tToMaxBrake) - 1/2*maxJerk*tToMaxBrake^2;
dist3 = -velAtFullBrake^2/2/maxBrake;

brakingDist = dist1+dist2+dist3;

sigmaMin2 = min(sigmax2, sigmay2);
riskThreshold = ellipticalRiskCost(brakingDist, 0, [0; 0], diag([1/sigmaMin2, sigmaMin2]), alpha, beta);

%% Risk plot
% Positions
qX = 0:.1:50; % [m]
qY = -6:.1:6; % [m]
% Velocities
% egoVelX = 60*kmhToms;
% egoVelY = 0;

distX = qX-vehX;
distY = qY-vehY;
% relVel = [otherVelX-egoVelX];% otherVelY-egoVelY];

risk = ellipticalRiskCost(distX, distY, [velX-egoVx; velY-egoVy], invCovMatrix, alpha, beta);
% plot(qX, risk)
figure, hold on;
title('Risk');
imagesc(qX, qY, risk); colorbar
set(gca,'YDir','normal')
% contour(qX, qY, risk);
axis equal
xlim(qX([1 end]));
ylim(qY([1 end]));
scatter(vehX, vehY, 'filled', 'MarkerFaceColor', 'r');
% legend('vehicle position')

%
figure, hold on;
title('Safe region')
imagesc(qX, qY, risk<riskThreshold); colorbar
set(gca,'YDir','normal')
axis equal
xlabel('x [m]'), xlim(qX([1 end]));
xlabel('y [m]'), ylim(qY([1 end]));
scatter(vehX, vehY, 'filled', 'MarkerFaceColor', 'r');