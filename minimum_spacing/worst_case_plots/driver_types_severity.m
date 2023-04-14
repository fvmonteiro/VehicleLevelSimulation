% Severity color maps for each driver type

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
gamma = 0.8;
follMaxVel = 30;
for k = 1:nF
    follower(k) = VehicleLongitudinal('P', 'F', [0, v0L], [delay1(k), delay2(k)]);
    follower(k).maxBrake = gamma*leader.maxBrake;
    follower(k).brakeFullStop(t);
    
%     follower(k).adjustTimeHeadway(leader, follMaxVel);
end

minGap = zeros(nF, nV0);
severity = cell(nF, nV0);

maxnGaps = 0;
maxMinSafeGap = 0;

for iterV = 1:nV0
    % Followers brake
    for iterFtype = 1:nF
        follower(iterFtype).v0 = v0array(iterV);
        follower(iterFtype).brakeFullStop(t);
        
        % Find minimum gaps
        minGap(iterFtype, iterV) = follower(iterFtype).computeFollowingMSS(leader);
        
        % Simulate for different initial gaps
        initGapArray = 0.1:0.1:minGap(iterFtype, iterV); % the first min gap (from the human follower) is always the largest
        nGaps = length(initGapArray);
        
        if minGap(iterFtype, iterV)>maxMinSafeGap
            maxMinSafeGap = minGap(1, iterV);
            maxnGaps = nGaps;
        end
        
%         leader.x0 = initGapArray;
        severity{iterFtype, iterV} = follower(iterFtype).computeSeverity(leader, initGapArray);%zeros(nGaps, 1);
        
    end
end

% Fill the shorter arrays with zeros
for iterV = 1:nV0
    for iterFtype = 1:nF
        severity{iterFtype, iterV}(end:maxnGaps) = 0;
    end
end

sevMatrices = zeros(maxnGaps, nV0, nF);
for iterFtype = 1:nF
    sevMatrices(:, :, iterFtype) = cell2mat(severity(iterFtype, :));
end

%% Plot
figHandles = zeros(3, 1);
for k = 1:nF
    figHandles(k) = figure; 
    hold on;
    % colormap('gray')
    xRange = (v0array([1 end])-v0L)*3.6;
    yRange = [0 maxMinSafeGap];
    imagesc(xRange, yRange, sevMatrices(:, :, k));
    set(gca,'YDir','normal')
    xlim(xRange);
    ylim(yRange);
    xlabel('\Delta v(0) [km/h]');
    ylabel('g(0) [m]')
    c = colorbar;
    c.Label.String = '\Delta v [(km/h)]';
    c.Label.FontSize = 12;
    
    plot((v0array-v0L)*3.6, minGap(k, :), 'r', 'LineWidth', 1.5);
%     legend('Min Safe Gap', 'FontSize', 14)
    legend('g*_{E,L}(0)', 'FontSize', 14)
end

%% Save
% imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/images/';
% % imgFolder = 'G:/My Drive/Lane Change/images/';
% figNames = {'humanSev', 'autonomousSev', 'connectedSev'};
% for k = 1:nF
%     saveas(figHandles(k), ['figures/' figNames{k}]);
%     saveas(figHandles(k), [imgFolder figNames{k} '.eps'], 'epsc');
% end


