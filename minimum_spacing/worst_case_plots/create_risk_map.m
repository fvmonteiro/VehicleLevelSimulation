%%%% Code to plot collision severity around vehicle

clearvars
close all

%% Parameters

% Some constants
deltaT = 0.01; % [s] sampling time
deltaS = 0.1; % [m] sampling space
mps2kph = 3.6; % m/s to km per hour

% Scenario
laneWidth = 3.6; %[m]
nLanes = 2;
scenarioWidth = nLanes*laneWidth;
totalTime = 15; % [s] total simulation time (maybe this could be computed from the scenario)
t = 0:deltaT:totalTime; 

% 
reactionTime = 0.3; % [s] time to start emergency braking
tLat = 5; % [s] lane change duration

%% Create vehicles

vehLoc = {'Fd', 'Ld', ... 
    'Fo', 'Lo'};
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

% Variations in max brake
gammaL = 1.15;
gammaF = 0.85;

% Assumption for ACC
rho = 0.9; % multiplicative factor vL/vF
maxVel = 30;

egoVeh = VehicleRiskMap(type, 'ego', x0Ego, y0Ego, v0Ego, reactionTime, 0);

% For now, x0 and y0 of ego are zero
deltaV = 10/3.6; % [m/s] -> *3.6[km/h]
v0Left = v0Ego + deltaV;
v0Right = v0Ego - deltaV; % not being used
vehs = VehicleRiskMap.empty(nVeh, 0);
x0 =  [x0Ego-egoVeh.len-40, x0Ego+50, ...
    x0Ego-egoVeh.len-35, x0Ego+40]; % [Fl, Ll, Fo, Lo] % zeros(nVeh, 1);
y0 = zeros(nVeh, 1);

for n = 1:nVeh
    otherVeh = vehLoc{n};
    
    switch lower(otherVeh(2))
        case {'l', 'd'} % left or destination lane
            % Create vehicle to the left
            y0(n) = y0Ego + laneWidth;
            vehs(n) = VehicleRiskMap(type, vehLoc{n}, x0(n), y0(n), v0Left, reactionTime, 0);
            
        case 'o'
            % Create vehicle on the same lane
            y0(n) = y0Ego;
            vehs(n) = VehicleRiskMap(type, vehLoc{n}, x0(n), y0(n), v0Ego, reactionTime, 0);
            
        case 'r'
            warning('Right lane changes are not yet properly coded')
            % Create vehicle to the right
            y0(n) = y0Ego - laneWidth;    
            vehs(n) = VehicleRiskMap(type, vehLoc{n}, x0(n), y0(n), v0Right, reactionTime, 0);
    end
    
    % Even "worse" case: assume followers brake less than leaders
    switch upper(otherVeh(1))
        case 'L'
            vehs(n).maxBrake = gammaL*vehs(n).maxBrake;
        case 'F'
            vehs(n).maxBrake = gammaF*vehs(n).maxBrake;
        otherwise
            error('unknown vehicle location')
    end
    
end


%% Compute risks and create severity matrix

minX = min([vehs.x0] - [vehs.len]);
maxX = max([vehs.x0])+5;
minY = min([vehs.y0] - laneWidth/2);
longSpace = minX:deltaS:maxX;
latSpace = minY:deltaS:(minY+scenarioWidth);

scenarios = 2; %with or without lane change
duringLC = [0, 1];
sevMap = cell(scenarios, 1);

for k = 1:scenarios

    deltaGapsDuringLC = zeros(nVeh, 1);
    minVehFollowingGap = zeros(nVeh, 1);
    gapACC = zeros(nVeh, 1);
    severity = cell(nVeh, 1);
    singleVehSevMaps = zeros(length(latSpace), length(longSpace), nVeh);
    
    for n = 1:nVeh
        disp(n)
        otherVeh = vehs(n);
        
        egoVeh.sinLatAccel(t, 0, tLat, laneWidth); % assumes left lane change
        
        if duringLC(k)
            deltaGapsDuringLC(n) = egoVeh.computeGapVariationDuringLC(t, otherVeh, tLat);
            lcSpatialShift = deltaGapsDuringLC(n);
        else
            lcSpatialShift = 0;
        end
        egoVeh.brakeFullStop(t, otherVeh);
        otherVeh.brakeFullStop(t);
        switch upper(otherVeh.name(1))
            case 'L'
                egoVeh.adjustTimeHeadway(otherVeh, maxVel, rho);
                if duringLC(k)
                    [minVehFollowingGap(n), gapACC(n)] = egoVeh.computeFollowingMSSDuringLC(t, otherVeh, tLat);
                else
                    [minVehFollowingGap(n), gapACC(n)] = egoVeh.computeFollowingMSS(otherVeh);
                end
                severity{n} = egoVeh.computeSeverityByDistance(otherVeh, gapACC(n), deltaS);
                severity{n} = flip(severity{n});
                vehRear = otherVeh.x0-otherVeh.len;
                riskyZone = longSpace<=(vehRear-floor(lcSpatialShift)) & ...
                    longSpace>=(vehRear-floor(lcSpatialShift)-ceil(gapACC(n)));
            case 'F'
                otherVeh.adjustTimeHeadway(egoVeh, maxVel, rho);
                [minVehFollowingGap(n), gapACC(n)] = otherVeh.computeFollowingMSS(egoVeh);
                severity{n} = otherVeh.computeSeverityByDistance(egoVeh, gapACC(n), deltaS);
                riskyZone = longSpace>=(otherVeh.x0+floor(lcSpatialShift)) & ...
                    longSpace<=(otherVeh.x0+floor(lcSpatialShift)+ceil(gapACC(n)));
        end
        
        vehLatIdx = latSpace>=otherVeh.y0-otherVeh.width/2 & ...
            latSpace<=otherVeh.y0+otherVeh.width/2;
        singleVehSevMaps(vehLatIdx, riskyZone, n) = repmat(severity{n}', sum(vehLatIdx), 1);
    end
    
    sevMap{k} = max(singleVehSevMaps, [], 3);
    
end
% Find last interesting longitudinal point (where sev>0)
% widthIdx = size(sevMap, 1);
% idxLaneD = find(sevMap(round(3/4*widthIdx), :), 1, 'last');
% idxLaneO = find(sevMap(round(1/4*widthIdx), :), 1, 'last');
% idxMaxX = max([idxLaneD, idxLaneO]) + 10;

% reducedSevMap = sevMap(:, 1:idxMaxX+10);
%% Plots

figHandles = zeros(scenarios, 1);

cmax = ceil(max(max(cell2mat(sevMap)))*(mps2kph));

for k = 1:scenarios
    figHandles(k) = figure;
    hold on;
    % title('Safe Gaps for Lane Change')
    scenarioLength = longSpace(end);%(idxMaxX+10);
    scenarioX = [minX, scenarioLength];
    scenarioY = [minY, minY+scenarioWidth];
    imagesc(scenarioX, scenarioY, sevMap{k}*(mps2kph));
    % colormap('gray')
%     if k==1
    c = colorbar;
    c.Location = 'northoutside';
    set(gca,'YDir','normal')
    caxis([0 cmax]);
    c.Label.String = 'Severity [km/h]';
%     set(gca,'xcolor','none')
%     end
    % contour(longSpace, latSpace, sevMap);
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    xlim(scenarioX);
    ylim(scenarioY);
    
    % To avoid having squeezed numbers on axis:
    position = get(gcf, 'Position');
    aspectRatio = position(3)/position(4);
    increment = 200;
    newPosition = position + [-increment*aspectRatio -increment increment*aspectRatio increment];
    set(gcf, 'Position', newPosition);
    
    
    % Plot vehicles and min gaps
    egoRear = egoVeh.x0-egoVeh.len;
    egoRight = egoVeh.y0-egoVeh.width/2;
    rectangle('Position', [egoRear, egoRight, egoVeh.len, egoVeh.width], 'FaceColor', 'r', 'Curvature', 1)
    
    for n = 1:length(vehs)
        rear = vehs(n).x0-vehs(n).len;
        right = vehs(n).y0-vehs(n).width/2;
        rectangle('Position', [rear, right, vehs(n).len, vehs(n).width], 'FaceColor', [0,0.5,0], 'Curvature', 1)
        
        vehFollGap = gapACC(n);
        
        % Indicate collision areas if lane change starts
        if duringLC(k)
            currentLCgap = deltaGapsDuringLC(n);
            
            if strncmpi(vehs(n).name, 'F', 1) % Followers
                collisionAreaStart = vehs(n).x0;
                if currentLCgap>0
                    rectangle('Position', [collisionAreaStart, right, currentLCgap, vehs(n).width], 'EdgeColor', 'y', 'LineWidth', 1.5);
                end
                
            else % Leaders
                collisionAreaStart = rear-currentLCgap;
                if currentLCgap>0
                    rectangle('Position', [collisionAreaStart, right, currentLCgap, vehs(n).width], 'EdgeColor', 'y', 'LineWidth', 1.5);
                end
                
            end
        end
    end

end
%% Save

imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/images/';
% imgFolder = 'G:/My Drive/Lane Change/images/';
figNames = {'riskMap_no_LC', 'riskMap_with_LC'};
% for k = 1:length(figNames)
% %     saveas(figHandles(k), ['figures/' figNames{k}]);
%     saveas(figHandles(k), [imgFolder figNames{k} '_no_labels.eps'], 'epsc');
% end
