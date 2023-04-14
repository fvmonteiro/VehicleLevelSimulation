% Severity color maps for each different follower drivers
% ploted as a function of v0 and headway

warning('[Sept 4, 2020] Results seem tranposed somehow after changes') 

close all
clearvars;

% Simulation time
t = 0:0.01:15;

% Initial states
v0L = 25;
v0array = v0L-10:0.1:v0L+10;
nV0 = length(v0array);

% Perception delays
delay1 = [1, 0.3, 0.3];
delay2 = [1, 0.3, 0];


% Create vehicles
leader = VehicleLongitudinal('P', 'L', [0, v0L], [0, 0]);
leader.maxJerk = inf;
leader.brakeFullStop(t);

nF = 3;
follower(nF) = VehicleLongitudinal();
for k = 1:nF
    follower(k) = VehicleLongitudinal('P', 'F', [0, v0array(1)], [delay1(k), delay2(k)]);
    follower(k).maxBrake = 0.8*leader.maxBrake;
    if k == nF
        follower(end).comfJerk = follower(end).maxJerk;
    end
    follower(k).brakeFullStop(t);
end

minGap = zeros(nF, nV0);
minHeadway = zeros(nF, nV0);
severity = cell(nF, nV0);

maxnHeadways = 0;
maxMinHeadway = 0;

for iterV = 1:nV0
    % Followers brake
    for iterFtype = 1:nF
        follower(iterFtype).v0 = v0array(iterV);
        follower(iterFtype).brakeFullStop(t);
        
        % Find minimum gaps
        minGap(iterFtype, iterV) = follower(iterFtype).computeFollowingMSS(leader);
        minHeadway(iterFtype, iterV) = minGap(iterFtype, iterV)/v0array(iterV);
        
        % Simulate for different initial gaps
        initGapArray = 0.01:0.01:minHeadway(iterFtype, iterV); % the first min gap (from the human follower) is always the largest
        nHeadways = length(initGapArray);
        
        if minHeadway(iterFtype, iterV)>maxMinHeadway
            maxMinHeadway = minHeadway(1, iterV);
            maxnHeadways = nHeadways;
        end
        
        severity{iterFtype, iterV} = follower(iterFtype).computeSeverity(leader, initGapArray);
        
%         severity{iterFtype, iterV} = zeros(nHeadways, 1);
%         for iterG = 1:nHeadways
%             leader.x0 = initGapArray(iterG)*v0array(iterV);
%             [~, ~, severity{iterFtype, iterV}(iterG)] = follower(iterFtype).computeFollowingMSS(leader);
%         end
    end
end

% Fill the shorter arrays with zeros
for iterV = 1:nV0
    for iterFtype = 1:nF
        severity{iterFtype, iterV}(end:maxnHeadways) = 0;
    end
end

sevMatrices = zeros(maxnHeadways, nV0, nF);
for iterFtype = 1:nF
    sevMatrices(:, :, iterFtype) = cell2mat(severity(iterFtype, :));
end

%% Plots
figHandles = zeros(3, 1);
for k = 1:nF
    figHandles(k) = figure; 
    hold on;
    % colormap('gray')
    xRange = (v0array([1 end])-v0L)*3.6;
    yRange = [0 maxMinHeadway];
    imagesc(xRange, yRange, sevMatrices(:, :, k));
    set(gca,'YDir','normal')
    xlim(xRange);
    ylim(yRange);
    xlabel('\Delta v(0) [km/h]');
    ylabel('headway(0) [s]')
    c = colorbar;
    c.Label.String = '\Delta v [(km/h)]';
    c.Label.FontSize = 12;
    
    plot((v0array-v0L)*3.6, minHeadway(k, :), 'r', 'LineWidth', 1.5);
    legend('Recommended Min Headway', 'FontSize', 14)
end

%% Save
% imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/minimum_spacing_presentation/';
% imgFolder = 'G:/My Drive/Lane Change/minimum_spacing_presentation/';
% figNames = {'humanSev_by_headway_and_v0', 'autonomousSev_by_headway_and_v0', 'connectedSev_by_headway_and_v0'};
% for k = 1:nF
%     saveas(figHandles(k), ['figures/' figNames{k}]);
%     saveas(figHandles(k), [imgFolder figNames{k} '.png']);
% end


