%%%% Code to plot minimum lane change distances along with collision
%%%% severity in case distances are not followed.
%%%% Issue not yet solved: possible lateral collision during deceleration
%%%% (i.e. in worst case scenarios). The assumed sinusoidal lateral
%%%% acceleration has to change depending on the vehicle longitudinal
%%%% acceleration. How to do that?

clearvars
close all

load_parameters
clear 'vehIdx'

vehLoc = {'Fl', 'Ll', ...
    'Fo', 'Lo'}; ...
%     'Fr', 'Lr'};
nVeh = length(vehLoc);

for n = 1:nVeh
    vehIdx.(vehLoc{n}) = n;   
end

% Ego vehicle
type = 'P';
x0Ego = 0;
y0Ego = 0;
v0Ego = 25; %[m/s]

egoVeh = Vehicle(type, 'ego', x0Ego, y0Ego, v0Ego, reactionTime);

% For now, x0 and y0 of ego are zero
deltaV = 5; % [m/s] -> *3.6[km/h]
v0Left = v0Ego + deltaV;
v0Right = v0Ego - deltaV;
vehs = Vehicle.empty(nVeh, 0);
x0 = zeros(nVeh, 1);
y0 = zeros(nVeh, 1);
% v0 = zeros(nVeh, 1);
transientGaps = zeros(nVeh, 1);
minVehFollowingGap = zeros(nVeh, 1);
minLCGaps = zeros(nVeh, 1);
% sevMap = zeros(length(latSpace), length(longSpace));
% singleVehSevMaps = zeros(length(latSpace), length(longSpace), nVeh);
severity = cell(nVeh, 1);
for n = 1:nVeh
    currentVeh = vehLoc{n};
    
    switch lower(currentVeh(2))
        case 'l'
            % Create vehicle to the left
            y0(n) = y0Ego + laneWidth;
%             v0(n) = v0Ego + deltaV;
            vehs(n) = Vehicle(type, vehLoc{n}, x0(n), y0(n), v0Left, reactionTime);
            vehs(n).zeroAccel(t);

            egoVeh.sinLatAccel(t, 0, tLat, laneWidth);
%             egoVeh.zeroAccel(t); % remember to consider using speed adjustment
            egoVeh.longVelAdjustmentDuringLC(t, v0Left, tLat);
%             transientGaps(n) = computeLaneChangeMSS(egoVeh, vehs(n), t, tLat);
%             [minVehFollowingGap(n), severity{n}] = egoVeh.longCollisionSeverity(t, deltaS, vehs(n));
            
        case 'o'
            % Create vehicle on the same lane
            y0(n) = y0Ego;
%             v0(n) = v0Ego;
            vehs(n) = Vehicle(type, vehLoc{n}, x0(n), y0(n), v0Ego, reactionTime);
            vehs(n).zeroAccel(t);
            % Compute safe distance
%             egoVeh.zeroAccel(t); 
%             transientGaps(n) = computeLaneChangeMSS(egoVeh, vehs(n), t, tLat);
            % ISSUE in above line: the transientGap depends if ego is
            % moving left (increasing speed) or right (decreasing) when
            % there is speed adj during lane change.
%             [minVehFollowingGap(n), severity{n}] = egoVeh.longCollisionSeverity(t, deltaS, vehs(n));
        case 'r'
            warning('Right lane changes are not yet properly coded')
            % Create vehicle to the right
            y0(n) = y0Ego - laneWidth;    
%             v0(n) = v0Ego - deltaV;
            vehs(n) = Vehicle(type, vehLoc{n}, x0(n), y0(n), v0Right, reactionTime);

            egoVeh.sinLatAccel(t, 0, tLat, -laneWidth);
            egoVeh.zeroAccel(t); % remember to consider using speed adjustment
%             egoVeh.longVelAdjustmentDuringLC(t, vehs(vehIdx.Lr), tLat);
%             transientGaps(n) = computeLaneChangeMSS(egoVeh, vehs(n), t, tLat);
%             [minVehFollowingGap(n), severity{n}] = egoVeh.longCollisionSeverity(t, deltaS, vehs(n));
    end
    
    transientGaps(n) = computeLaneChangeMSS(egoVeh, vehs(n), t, tLat);
    [minVehFollowingGap(n), severity{n}] = egoVeh.longCollisionSeverity(t, deltaS, vehs(n));
    minLCGaps(n) = minVehFollowingGap(n) + transientGaps(n);
    
    switch upper(currentVeh(1))
        case 'L'
            vehs(n).x0 = egoVeh.x0 + minLCGaps(n) + vehs(n).len;
        case 'F'
            vehs(n).x0 = egoVeh.x0 - egoVeh.len - minLCGaps(n);
    end
    
%     vehLatIdx = latSpace>=vehs(k).y0-vehs(k).width/2 & latSpace<=vehs(k).y0+vehs(k).width/2;
    % in case the unsafe zone doesn't fit in the current plot:
%     maxZoneSize = sum(riskyZone);
%     singleVehSevMaps(vehLatIdx, riskyZone, k) = repmat(severity(1:maxZoneSize)', sum(vehLatIdx), 1);
    
end

%% Set severity in a proper mapping to be plotted
minX0 = min([vehs.x0] - [vehs.len]);
minY0 = min([vehs.y0] - laneWidth/2);
longSpace = minX0:deltaS:(minX0+scenarioLength);
latSpace = minY0:deltaS:(minY0+scenarioWidth);

singleVehSevMaps = zeros(length(latSpace), length(longSpace), nVeh);
for n = 1:nVeh
    currentVeh = vehs(n);
    
    switch upper(currentVeh.location(1))
        case 'L'            
            vehRear = currentVeh.x0-currentVeh.len;
            collisionZone = longSpace<=(vehRear) & ...
                longSpace>=(vehRear-transientGaps(n));
            riskyZone = longSpace<=(vehRear-transientGaps(n)) & ...
                longSpace>=(vehRear-transientGaps(n)-minVehFollowingGap(n));
        case 'F'
            collisionZone = longSpace>=(currentVeh.x0) & ...
                longSpace<=(currentVeh.x0+transientGaps(n)+deltaS/10);
            riskyZone = longSpace>=(currentVeh.x0+transientGaps(n)) & ...
                longSpace<=(currentVeh.x0+transientGaps(n)+minVehFollowingGap(n)+deltaS/10);
    end
    
    vehLatIdx = latSpace>=currentVeh.y0-currentVeh.width/2 & ...
        latSpace<=currentVeh.y0+currentVeh.width/2;
    singleVehSevMaps(vehLatIdx, riskyZone, n) = repmat(severity{n}', sum(vehLatIdx), 1);
end

sevMap = max(singleVehSevMaps, [], 3);


%% Plots

figure, hold on;
title('Safe Gaps for Lane Change')
scenarioX = [minX0, minX0+scenarioLength];
scenarioY = [minY0, minY0+scenarioWidth];
imagesc(scenarioX, scenarioY, sevMap*(mpsTokph));
% colormap('gray')
c = colorbar;
set(gca,'YDir','normal')
% contour(longSpace, latSpace, sevMap);
axis equal
xlabel('x [m]')
ylabel('y [m]')
xlim(scenarioX);
ylim(scenarioY);
c.Label.String = 'Severity [(km/h)]';

% Plot vehicles and min gaps
egoRear = egoVeh.x0-egoVeh.len;
egoRight = egoVeh.y0-egoVeh.width/2;
rectangle('Position', [egoRear, egoRight, egoVeh.len, egoVeh.width], 'FaceColor', 'r', 'Curvature', 1)
    
for n = 1:length(vehs)
    rear = vehs(n).x0-vehs(n).len;
    right = vehs(n).y0-vehs(n).width/2;
    rectangle('Position', [rear, right, vehs(n).len, vehs(n).width], 'FaceColor', [0,0.5,0], 'Curvature', 1)
    
    vehFollGap = minVehFollowingGap(n);
    currentLCgap = transientGaps(n);
    if strncmpi(vehs(n).location, 'F', 1) % Followers
        collisionAreaStart = vehs(n).x0;
        riskyAreaStart = vehs(n).x0+currentLCgap;
        
        % Worst case scenario lateral collision
%         rectangle('Position', [vehs(k).x0, right, max(0, minLatCollisionFreeGap(k)), vehs(k).width], 'EdgeColor', 'b');
%         rectangle('Position', [vehs(k).x0, right, minLCGaps(k), vehs(k).width], 'EdgeColor', 'y');
    else % Leaders
        collisionAreaStart = rear-currentLCgap;
        riskyAreaStart = rear-currentLCgap-vehFollGap;
        % Worst case scenario lateral collision
%         rectangle('Position', [rear-minLatCollisionFreeGap(k), right, max(0, minLatCollisionFreeGap(k)), vehs(k).width], 'EdgeColor', 'b');
%         rectangle('Position', [rear-minLCGaps(k), right, minLCGaps(k), vehs(k).width], 'EdgeColor', 'y');
    end
    % Rearend possible collision
    rectangle('Position', [riskyAreaStart, right, vehFollGap, vehs(n).width], 'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
    % Const speed collision during lc
    if currentLCgap>0
        rectangle('Position', [collisionAreaStart, right, currentLCgap, vehs(n).width], 'EdgeColor', 'y', 'LineWidth', 1.5);
    end

end

