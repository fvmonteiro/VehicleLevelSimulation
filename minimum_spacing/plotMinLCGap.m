function [figHandle] = plotMinLCGap(egoVeh, minGaps, lane)
%plotMinLCGap Summary of this function goes here
%   Detailed explanation goes here

mpsTokph = 3600/1000; %meter per second to kilometer per hour
% otherVehsLocation = fields(minGaps);
nVehs = size(minGaps, 1);
if nVehs~=2
    error('plotMinLCGap function should receive min gaps from two vehicles')
end
% if otherVehsLocation{1}(2) ~= otherVehsLocation{2}(2)
%     warning('Ploting min gaps from different lanes');
% else
%     if otherVehsLocation{1}(2)=='d'
%         lane = 'destination lane';
%     else
%         lane = 'original lane';
%     end
% end

figHandle = figure;
grid on, hold on;
if lane=='d'
    titleStr = 'destination lane';
else
    titleStr = 'original lane';
end

% legStrCell = cell(3, 1);
totalGap = egoVeh.len;
vel = egoVeh.v0*mpsTokph;
% n=1;
for k = 1:nVehs %length(otherVehsLocation)
%     if otherVehsLocation{k}(2)==lane
        gapArray = minGaps(k, :);%.(otherVehsLocation{k});       
%         if otherVehsLocation{k}(1)=='L'
%             legStrCell{n} = 'to leader';
%             n = n+1;
%         else
%             legStrCell{n} = 'to follower';
%             n = n+1;
%         end
        totalGap = totalGap + minGaps(k, :);%.(otherVehsLocation{k});
        plot(vel, gapArray, 'LineWidth', 1.5);
%     end
end
legStrCell = {'to follower', 'to leader', 'total'};

plot(vel, totalGap, 'LineWidth', 1.5);
legend(legStrCell);
% lgd.FontSize = 14;
title(titleStr)
xlabel('v_{ego} [km/h]');
ylabel('min gap [m]');
end

