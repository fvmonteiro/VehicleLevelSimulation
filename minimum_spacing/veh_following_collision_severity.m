%% Constants
clearvars
close all

clearvars
close all

load_parameters
y.ego = 0.5*laneWidth; 
y.others(vehIdx.fo) = 0.5*laneWidth;
y.others(vehIdx.lo) = 0.5*laneWidth;
y.others(vehIdx.fd) = 1.5*laneWidth;
y.others(vehIdx.ld) = 1.5*laneWidth;
type.ego = 'ego';
type.others = cell(length(fields(vehIdx)), 1);
type.others(:) = {''};
[ego, vehs] = createVehicles(vehIdx, {'y', 'vehType'}, {y, type});

%% Collision severity

sevMap = zeros(length(latSpace), length(longSpace));
minSafeGaps = zeros(length(vehs), 2);

for k = 1:length(vehs)
    singleVehSevMap = zeros(length(latSpace), length(longSpace));
    vehs(k).location = 'F';
    [minSafeGapAhead, s2Ahead] = ego.longCollisionSeverity(t, deltaS, vehs(k));
%     minSafeGapAhead = max(g0Ahead);
    vehs(k).location = 'L';
    [minSafeGapBehind, s2Behind] = ego.longCollisionSeverity(t, deltaS, vehs(k));
%     minSafeGapBehind = max(g0Behind);
    s2Behind = flip(s2Behind);
    
%     minSafeGapAhead = max(g0Ahead);
%     minSafeGapBehind = max(g0Behind);    
    
    unsafeZoneAhead = longSpace>=(vehs(k).x0) & longSpace<=(vehs(k).x0+minSafeGapAhead+deltaS/10);
    unsafeZoneBehind = longSpace<=(vehs(k).x0-vehs(k).len) & longSpace>=(vehs(k).x0-vehs(k).len-minSafeGapBehind);
    vehLatIdx = latSpace>=vehs(k).y0-vehs(k).width/2 & latSpace<=vehs(k).y0+vehs(k).width/2;
    
    % in case the unsafe zone doesn't fit in the current plot:
    maxSizeAhead = sum(unsafeZoneAhead);
    maxSizeBehind = sum(unsafeZoneBehind);
    
    singleVehSevMap(vehLatIdx, unsafeZoneAhead) = repmat(s2Ahead(1:maxSizeAhead)', sum(vehLatIdx), 1);
    singleVehSevMap(vehLatIdx, unsafeZoneBehind) = repmat(s2Behind(1:maxSizeBehind)', sum(vehLatIdx), 1);
    
    sevMap = max(sevMap,singleVehSevMap);
    minSafeGaps(k, :) = [minSafeGapBehind; minSafeGapAhead];
end

%% Plots

figure, hold on;
title('Safe Following Distances')
imagesc([0 scenarioLength], [0 scenarioWidth], sevMap*(mpsTokph^2));
% colormap('gray')
c = colorbar;
set(gca,'YDir','normal')
% contour(longSpace, latSpace, sevMap);
axis equal
xlabel('x [m]')
ylabel('y [m]')
xlim([0 scenarioLength]);
ylim([0 scenarioWidth]);
c.Label.String = 'Severity [(km/h)^2]';

% Plot vehicles
egoRear = ego.x0-ego.len;
egoRight = ego.y0-ego.width/2;
rectangle('Position', [egoRear, egoRight, ego.len, ego.width], 'FaceColor', 'r', 'Curvature', 1)
    
for k = 1:length(vehs)
    rear = vehs(k).x0-vehs(k).len;
    right = vehs(k).y0-vehs(k).width/2;
    rectangle('Position', [rear, right, vehs(k).len, vehs(k).width], 'FaceColor', 'w', 'Curvature', 1)
    safeZoneSize = vehs(k).x0+minSafeGaps(k,2) - (rear-minSafeGaps(k,1));
    rectangle('Position', [rear-minSafeGaps(k,1), right, safeZoneSize, vehs(k).width], 'EdgeColor', 'r');
end