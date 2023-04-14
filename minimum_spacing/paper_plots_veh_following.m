%% Constants
clearvars
close all

%% Parameters
load_parameters
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking
% Vehicles
% Names:
% F - follower
% L - leader

% Types: P -  passenger vehicle, B - bus, T - truck
types = ["P", "T"];

% Initial conditions 
% Longitudinal positions [m](irrelevant for now)
x0 = 0;

% Lateral positions [m](irrelevant for now)
y0 = 0;

% initial longitudinal speeds [m/s]
vOther =  20;
speedRange = 10:0.1:30;%vOther-10:0.1:vOther+10;
vEgo0 = speedRange;

%% Collision severity
minGap = zeros(length(types), length(types), length(speedRange));
sev = cell(length(types), length(types));
figHandle = zeros(2*length(types), 1);
figName = cell(2*length(types), 1);
k = 1;
for egoType = 1:length(types)
    ego = Vehicle(types(egoType), 'ego', x0, y0, vEgo0, reactionTime);
    previousMaxMinGap = 0; % used to keep the same y axis scales for the same ego vehicle types
    for otherType = 1:length(types)
        other = Vehicle(types(otherType), 'L', x0, y0, vOther, reactionTime);
        [minGap(egoType, otherType, :), sev{egoType, otherType}] = ego.longCollisionSeverity(t, deltaS, other);
        figHandle(k) = plotSeverity(squeeze(minGap(egoType, otherType, :)), sev{egoType, otherType}, vEgo0-vOther, previousMaxMinGap);
        previousMaxMinGap = max(squeeze(minGap(egoType, otherType, :)));
        
        figName{k} = ['severity-vs-vel_' char(types(egoType)) char(types(otherType))];
        k = k + 1;

    end
end

%%% UNCOMMENT TO SAVE %%%%
% k = 1;
% for egoType = 1:length(types)
%     for otherType = 1:length(types)
%         mySavePlot(figHandle(k), figName{k});
%         k = k + 1;
%     end
% end


% [g0L, s2L] = ego.longCollisionSeverity(t, deltaS, lead);
% minGapL = ego.computeFollowingMSS(t, lead);

% % Severity for all speeds in several 2D plots
% for i = 1:length(types)
%     for j = 1:length(types)
%         plotSeverity(squeeze(minGap(i, j, :)), sev{i, j}, vEgo0);
%     end
% end

%% Plot function

function [figHandle] = plotSeverity(minGap, deltaV, speedRange, previousMaxMinGap)
global mpsTokph;

% timeHeadway = minGap./speedRange;
gLim = [0 max(minGap(:))];
speedRange = speedRange*mpsTokph;
deltaV = deltaV*mpsTokph;

if previousMaxMinGap>max(minGap(:))
    gLim = [0 previousMaxMinGap];
    neededSamples = ceil(previousMaxMinGap)*10;
    deltaV = [deltaV; zeros(neededSamples-size(deltaV, 1), size(deltaV, 2))];
%     deltaV = [zeros()];
end

figHandle = figure; 
hold on;
% colormap('gray')
imagesc(speedRange([1 end]), gLim, deltaV);
set(gca,'YDir','normal')
xlim(speedRange([1 end]));
ylim(gLim);
xlabel('\Delta v(0) [km/h]');
ylabel('g(0) [m]')
c = colorbar;
c.Label.String = 'Severity - \Delta v [(km/h)]';

plot(speedRange, minGap, 'r', 'LineWidth', 1.5);
legend('Veh Following Gap')
end

