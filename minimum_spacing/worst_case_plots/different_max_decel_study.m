%%% Code to study min veh following spaces during LC
%%% Focus is to study how to define time headway when leader and follower
%%% decelerations differ by a large factor

%% Clear variables and plots

close all
clearvars;

%% Parameters
% Simulation time
t = 0:0.01:15;
mps2kph = 3.6;

% Initial states
v0L = 90/mps2kph; % 90km/h = 25m/s
deltaV = 10;
v0array = v0L-deltaV:0.1:v0L+deltaV; % 54km/h to 126km/h
nV0 = length(v0array);

% Perception delays for connected follower
delay1 = 0.3;
delay2 = 0;

% Time for lane change
tLat = 5;

%% Create vehicles
leader = VehicleLongitudinal('P', 'L', 0, v0L, 0, 0);
leader.maxJerk = inf;
leader.brakeFullStop(t);

follower = VehicleRiskMap('P', 'F', 0, 0, v0L, delay1, delay2);
gammaD = 0.85;
rho = 0.9;
follMaxVel = 30; % approx 65 miles per hour
follower.maxBrake = gammaD*leader.maxBrake;
follower.adjustTimeHeadway(leader, follMaxVel, rho);
disp(follower.h)

%% Compute severity
minGap = zeros(1, nV0);
gapACC = zeros(1, nV0);
severity = cell(1, nV0);

maxnGaps = 0;
maxMinSafeGap = 0;
duringLC = 0;

for iterV = 1:nV0
    % Followers brake
    follower.v0 = v0array(iterV);
    follower.brakeFullStop(t);
    
    % Find minimum gaps
    if duringLC
        [minGap(iterV), gapACC(iterV)] = follower.computeFollowingMSSDuringLC(t, leader, tLat);
    else
        [minGap(iterV), gapACC(iterV)] = follower.computeFollowingMSS(leader);
    end
    
    % Simulate for different initial gaps
    initGapArray = 0.1:0.1:minGap(iterV); % the first min gap (from the human follower) is always the largest
    nGaps = length(initGapArray);
    
    if minGap(iterV)>maxMinSafeGap
        maxMinSafeGap = minGap(1, iterV);
        maxnGaps = nGaps;
    end
    
    severity{iterV} = zeros(nGaps, 1);
    
    severity{iterV} = follower.computeSeverity(leader, initGapArray);
    
end

% % Fill the shorter arrays with zeros
for iterV = 1:nV0
%     for iterFtype = 1:nF
    if isempty(severity{iterV})
        severity{iterV} = zeros(maxnGaps, 1);
    else
        severity{iterV}(end:maxnGaps) = 0;
    end
%     end
end

% sevMatrices = zeros(maxnGaps, nV0);
sevMatrices = cell2mat(severity);

%% Plot
% figHandles = zeros(nF, 1);
figHandles = figure; 
hold on;
% colormap('gray')
% xRange = (v0array([1 end])-v0L);
xRange = (v0array([1 end])-v0L)*mps2kph;
yRange = [0 maxMinSafeGap];
imagesc(xRange, yRange, sevMatrices);
set(gca,'YDir','normal')
xlim(xRange);
ylim(yRange);
xlabel('\Delta v(0) [km/h]');
ylabel('g(0) [m]')
c = colorbar;
c.Label.String = '\Delta v [(km/h)]';
c.Label.FontSize = 12;

%%% ACTUAL PLOTS TO BE SAVED
plot((v0array-v0L)*3.6, gapACC, 'r', 'LineWidth', 1.5);
% legend('g*_{E,L}(0)', 'FontSize', 14)
legend('ACC Gap', 'FontSize', 14, 'Location', 'NW')

% Plots for analysis
% plot((v0array-v0L)*mps2kmph, minGap, 'r', 'LineWidth', 1.5);
% plot((v0array-v0L)*mps2kmph, gapACC, '--r', 'LineWidth', 1.5);
% plot((v0array-v0L)*mps2kmph, minGap-gapACC, 'y', 'LineWidth', 1.5);




%% Save

imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/images/';
% imgFolder = 'G:/My Drive/Lane Change/images/';
figNames = {'sev_map_acc_gap'};
% for k = 1:length(figHandles)
%     saveas(figHandles(k), ['figures/' figNames{k}]);
%     saveas(figHandles(k), [imgFolder figNames{k} '.eps'], 'epsc');
% end


