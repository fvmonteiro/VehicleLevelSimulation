%%%% Test to show why there are flat tops on the delta v plot

close all
clearvars;

% Simulation time
t = 0:0.01:15;

% Initial states
v0L = 25;
v0F = 25;

% Perception delays
delay1 = [1, 0.3, 0.3];
delay2 = [1, 0.3, 0];

% Create vehicles and "move" them
leader = VehicleLongitudinal('P', 'L', [0, v0L], [0, 0]);
leader.brakeFullStop(t);

nF = 3;
follower(nF) = VehicleLongitudinal();
for k = 1:nF
    follower(k) = VehicleLongitudinal('P', 'F', [0, v0F], [delay1(k), delay2(k)]);
    follower(k).maxBrake = 0.8*leader.maxBrake;
    if k == nF
        follower(end).comfJerk = follower(end).maxJerk;
    end
    follower(k).brakeFullStop(t);
end

% follower = VehicleLongitudinal('P', 'F', 0, v0F, delay1, delay2);
% autonomousFollower = VehicleLongitudinal('P', 'F', 0, v0F, 0.3, 0.3);
% connectedFollower = VehicleLongitudinal('P', 'F', 0, v0F, 0.3, 0);
% connectedFollower.comfJerk = connectedFollower.maxJerk;

% Find minimum gap
longestMinSafeGap = 0;
for k = 1:nF
    longestMinSafeGap = max(longestMinSafeGap, follower(k).computeFollowingMSS(leader));
end

% Simulate for different initial gaps
initGapArray = 0:0.1:longestMinSafeGap*1.1;
initHeadwayArray = 0:0.01:longestMinSafeGap/v0F;
% initGapArray = [1 10 20 30:5:50 60];
nGaps = length(initGapArray);
nHeadway = length(initHeadwayArray);

minSafeGaps = -ones(nGaps, nF);
collisionTimeIdx = -ones(nGaps, nF);
severity = -ones(nGaps, nF);


for k = 1:nF 
    severity(:, k) = follower(:, k).computeSeverity(leader, initGapArray);
    
    for n = 1:nGaps
        leader.x0 = initGapArray(n);
        [minSafeGaps(n, k), collisionTimeIdx(n, k)] = follower(k).computeFollowingMSS(leader);
    end
end

severityByHeadway = -ones(nHeadway, nF);
for n = 1:nHeadway
    leader.x0 = initHeadwayArray(n)*v0F;
    for k = 1:nF
        [~, ~, severityByHeadway(n, k)] = follower(k).computeFollowingMSS(leader);
    end
end

%% Plots

% Delta v vs g0
figHandles(1) = figure; hold on; grid on;
for k = 1:nF
    plot(initGapArray, (severity(:, k)*3.6), 'LineWidth', 1.5)
    xlabel('g_0 [m]')
    ylabel('\Delta v [km/h]');
end
legend({'human', 'autonomous', 'connected'}, 'FontSize', 14);

% Delta v^2 vs g0
figHandles(2) = figure; hold on; grid on;
for k = 1:nF
    plot(initGapArray, ((severity(:, k)*3.6).^2), 'LineWidth', 1.5)
    xlabel('g_0 [m]')
    ylabel('\Delta v^2 [km/h]^2');
end
legend({'human', 'autonomous', 'connected'}, 'FontSize', 14);

% Severity vs tc
figure; hold on; grid on;
for k = 1:nF
    plot(t(collisionTimeIdx(:, k)), ((severity(:, k)*3.6)), 'LineWidth', 1.5)
    xlabel('t_c [s]')
    ylabel('\Delta v [km/h]');
end
legend({'human', 'autonomous', 'connected'}, 'FontSize', 14);

% Delta v vs headway0
figure; hold on; grid on;
for k = 1:nF
    plot(initHeadwayArray, (severityByHeadway(:, k)*3.6), 'LineWidth', 1.5)
    xlabel('headway [s]')
    ylabel('\Delta v [km/h]');
end
legend({'human', 'autonomous', 'connected'}, 'FontSize', 14);

% imgFolder = 'C:/Users/fvall/Google Drive/Lane Change/minimum_spacing_presentation/';
% imgFolder = 'G:/My Drive/Lane Change/minimum_spacing_presentation/';
% figNames = {'delta_v', 'delta_v_squared'};
% for iterFig = 1:length(figHandles)
%     saveas(figHandles(iterFig), ['figures/' figNames{iterFig} ]);
%     saveas(figHandles(iterFig), [imgFolder figNames{iterFig} '.png']);
% end

% Accelerations
fa = figure;
for k = 1:nF
    fa = follower(k).plotState(t, 'a', fa);
end
legend({'human', 'autonomous', 'connected'});

% Velocities
fv = figure;
for k = 1:nF
    fv = follower(k).plotState(t, 'v', fv);
end
hold on; grid on;
leader.plotState(t, 'v', fv);
legend({'human', 'autonomous', 'connected', 'leader'});

% Collision points
% scatterLegend = cell(1, nGaps);
% hold on;
% for n = 1:100:nGaps
%     collisionTime = zeros(nF, 2);
%     leaderCollisionSpeed = zeros(nF, 1);
%     followerCollisionSpeed = zeros(nF, 1);
%     for k = 1:nF
%         collisionTime(k, :) = t(collisionTimeIdx(n, k));
%         leaderCollisionSpeed(k) = leader.vt(collisionTimeIdx(n, k));
%         followerCollisionSpeed(k) = follower(k).vt(collisionTimeIdx(n, k));
%     end
%     scatter(collisionTime(:), [leaderCollisionSpeed; followerCollisionSpeed], 'fill');
% %     scatterLegend{n} = ['delta v for g(0) = ' num2str(initGapArray(n))];
% end
% legend(['leader speed' 'follower speed' scatterLegend]);

% figure; hold on; grid on;
% plot(initGapArray, t(humanCollisionTimeIdx), 'LineWidth', 1.5)
% xlabel('g_0 [m]')
% ylabel('collision time');