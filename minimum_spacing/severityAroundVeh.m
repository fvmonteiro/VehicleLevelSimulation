function [severity] = severityAroundVeh(simTime, deltaS, otherVeh, egoVeh)%, tLat, laneWidth)

warning('This function doesn''t seem necessary any longer (7/2/2019) and will probably be deleted soon.')
% if abs(egoVeh.y0-otherVeh.y0) < laneWidth/2
%     lane = 'o';
% else
%     lane = 'd';
% end

% if isempty(otherVeh.location)
    otherVeh.location = 'F';
    [g0F, s2F] = collisionSeverity(simTime, deltaS, egoVeh, otherVeh);
    otherVeh.location = 'L';
    [g0L, s2L] = collisionSeverity(simTime, deltaS, egoVeh, otherVeh);
    % The naming is somewhat counterintuitive here because we want to center
    % around the other vehicle (not the ego)
    severity.gapAhead = g0F;
    severity.sevAhead = s2F;
    severity.gapBehind = g0L;
    severity.sevBehind = flip(s2L);
% else
%     [g0, s2] = collisionSeverity(simTime, deltaS, egoVeh, otherVeh);
%     severity.gap = g0;
%     if strncmpi(otherVeh.location, 'L', 1)
%         s2 = flip(s2);
%     end
%     severity.sev = s2;
% end


% if lane == 'o'
%     severity.minGapAhead = max(g0F);
%     severity.minGapBehind= max(g0L);
% else
%     otherVeh.location = ['F' lane];
%     severity.minGapAhead = minLaneChangeGap(simTime, egoVeh, otherVeh, delay, tLat, 0, laneWidth);
%     otherVeh.location = ['L' lane];
%     severity.minGapBehind= minLaneChangeGap(simTime, egoVeh, otherVeh, delay, tLat, 0, laneWidth);
% end

end