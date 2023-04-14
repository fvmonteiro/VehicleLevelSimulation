%% Constants
clearvars
close all

load_parameters
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking
%% Vehicles
% Names:
% F - follower
% L - leader

% Types: P -  passenger vehicle, B - bus, T - truck
typeEgo = 'P';
typeF = 'P';
typeL = 'P';

% Initial conditions 
% Longitudinal positions [m](irrelevant for now)
xEgo = 0;
xF = 0;
xL = 0;

% Lateral positions [m](irrelevant for now)
yEgo = 0;
yF = 0; 
yL = 0;

% initial longitudinal speeds [m/s]
speedRange = 10:0.2:30;
vEgo0 = speedRange;
vF0 =  20;
vL0 = 20;

ego = Vehicle(typeEgo, 'ego', xEgo, yEgo, vEgo0, reactionTime);
foll = Vehicle(typeF, 'F', xF, yF, vF0, reactionTime);
lead = Vehicle(typeL, 'L', xL, yL, vL0, reactionTime);

%% Collision severity

[g0F, s2F] = ego.longCollisionSeverity(t, deltaS, foll);
minGapF = ego.computeFollowingMSS(t, foll);

[g0L, s2L] = ego.longCollisionSeverity(t, deltaS, lead);
minGapL = ego.computeFollowingMSS(t, lead);

%% Plots
% Severity for each speed as a function of g0
if length(speedRange)<5 % being sure we don't plot too many results
for n = 1:length(vF0)
    collisionIdx = g0L(:,n)>=0;
    if sum(collisionIdx)>2
        figure; hold on; grid on;
        plot(g0L(collisionIdx, n), s2L(collisionIdx, n));
        xlabel('g(0) [m]'); ylabel('\Delta v^2 [m^2/s^2]')
    else
        disp('No initial gap causes collision')
    end
    
end
end

% Severity for all speeds
plotSeverity(g0F, s2F, minGapF, vEgo0);
title(['to follower at ' num2str(vF0*mpsTokph) ' km/h'])
plotSeverity(g0L, s2L, minGapL, vEgo0);
title(['to leader at ' num2str(vL0*mpsTokph) ' km/h'])

% Total
s2Fnew = [s2F; zeros(size(s2L,1)-size(s2F,1), size(s2F,2))];
totalS2 = s2Fnew+s2L;
plotSeverity(g0L, totalS2, minGapF+minGapL, speedRange);
title(['leader at ' num2str(vL0*mpsTokph) ' km/h and follower at ' num2str(vF0*mpsTokph) ' km/h'])

function [figHandle] = plotSeverity(g0, s2, minGap, speedRange)
global mpsTokph;

speedRange = speedRange*mpsTokph;
s2 = s2*mpsTokph^2;
gLim = [0 max(g0(:))];
figHandle = figure; 
hold on;
% colormap('gray')
imagesc(speedRange([1 end]), gLim, s2);
set(gca,'YDir','normal')
xlim(speedRange([1 end]));
ylim(gLim);
xlabel('v_{ego} [km/h]');
ylabel('g(0) [m]')
c = colorbar;
c.Label.String = 'Severity - \Delta v^2 [(km/h)^2]';

% figure; hold on; grid on;
plot(speedRange, minGap, 'r', 'LineWidth', 1.5);
legend('Min safe gap')
end

