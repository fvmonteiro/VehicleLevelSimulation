% Severity color maps

warning('Old code: newest method is inside SimulationScenarios class')

close all
clearvars;

% Simulation time
t = 0:0.01:15;

% Initial states
v0L = 25;
v0array = v0L-10:0.1:v0L+10;
nV0 = length(v0array);

% Perception delays
delay = 0.3;

% Vehicle types
types = {'P', 'T'};
nTypes = length(types);

minGap = zeros(nTypes, nTypes, nV0);
severity = cell(nTypes, nTypes, nV0);

maxnGaps = 0;
maxMinSafeGap = 0;

for iterLeaderType = 1:nTypes
    leaderType = types{iterLeaderType};
    leader = VehicleLongitudinal(leaderType, 'L', [0, v0L], 0);
    leader.maxJerk = inf;
    leader.brakeFullStop(t);
    
    for iterFollowerType = 1:nTypes
        followerType = types{iterFollowerType};
        follower = VehicleLongitudinal(followerType, 'F', [0, v0array(1)], delay);
%         if strcmp(leaderType, followerType)
%             follower.maxBrake = 0.8*follower.maxBrake;
%         end
        follower.comfJerk = follower.maxJerk;
        
        for iterV = 1:nV0
            % Followers brake
            follower.v0 = v0array(iterV);
            follower.brakeFullStop(t);
            
            % Find minimum gaps
            minGap(iterLeaderType, iterFollowerType, iterV) = follower.computeFollowingMSS(leader);
            
            % Simulate for different initial gaps
            initGapArray = 0.1:0.1:minGap(iterLeaderType, iterFollowerType, iterV);
            nGaps = length(initGapArray);
            
            if minGap(iterLeaderType, iterFollowerType, iterV)>maxMinSafeGap
                maxMinSafeGap = minGap(iterLeaderType, iterFollowerType, iterV);
                maxnGaps = nGaps;
            end
            
            severity{iterLeaderType,  iterFollowerType, iterV} = zeros(nGaps, 1);
            
            severity{iterLeaderType,  iterFollowerType,iterV} = follower.computeSeverity(leader, initGapArray);
%             for iterG = 1:nGaps
%                 leader.x0 = initGapArray(iterG);
%                 severity{iterLeaderType,  iterFollowerType,iterV}(iterG) = follower.computeSeverity(leader);
%             end
        end
    end
end

% Fill the shorter arrays with zeros
for iterLeaderType = 1:nTypes    
    for iterFollowerType = 1:nTypes
        for iterV = 1:nV0
            severity{iterLeaderType, iterFollowerType, iterV}(end:maxnGaps) = 0;
        end
    end
end

sevMatrices = zeros(maxnGaps, nV0, nTypes, nTypes);
for iterLeaderType = 1:nTypes    
    for iterFollowerType = 1:nTypes
        sevMatrices(:, :, iterLeaderType, iterFollowerType) = ...
            squeeze(cell2mat(severity(iterLeaderType, iterFollowerType, :)));
    end
end

%% Plots

figHandles = zeros(nTypes, nTypes);

for iterLeaderType = 1:nTypes
    for iterFollowerType = 1:nTypes
        figHandles(iterLeaderType, iterFollowerType) = figure;
        hold on;
        % colormap('gray')
        xRange = (v0array([1 end])-v0L)*3.6;
        yRange = [0 maxMinSafeGap];
        imagesc(xRange, yRange, sevMatrices(:, :, iterLeaderType, iterFollowerType));
        set(gca,'YDir','normal')
        xlim(xRange);
        ylim(yRange);
        xlabel('\Delta v(0) [km/h]');
        ylabel('g(0) [m]')
        c = colorbar;
        c.Label.String = '\Delta v [(km/h)]';
        c.Label.FontSize = 12;
        
        plot((v0array-v0L)*3.6, squeeze(minGap(iterLeaderType, iterFollowerType, :)), 'r', 'LineWidth', 1.5);
        legend('Min Safe Gap', 'FontSize', 14)
    end
end

%% Save
% imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/images/';
% imgFolder = 'G:/My Drive/Lane Change/images/';
% figNames = {{'leadP', 'leadT'}, {'follP', 'follT'}};
% for iterLeaderType = 1:nTypes
%     for iterFollowerType = 1:nTypes
%         saveas(figHandles(iterLeaderType, iterFollowerType), ['figures/sev' figNames{1}{iterLeaderType}  figNames{2}{iterFollowerType}]);
%         saveas(figHandles(iterLeaderType, iterFollowerType), [imgFolder 'sev' figNames{1}{iterLeaderType}  figNames{2}{iterFollowerType} '.eps'], 'epsc');
%     end
% end


