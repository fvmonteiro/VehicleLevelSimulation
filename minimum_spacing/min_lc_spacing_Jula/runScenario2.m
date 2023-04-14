function [MSS, figHandle] = runScenario2(t, tLat, h, relVelRange, otherVehs, mergingVeh, deltaDO)
%runScenario2 Run and plot results for scenario where merging vehicle is 
% already well positioned - it just accelerates (decelerates) to meet the 
% speeds of vehicles on the adjacent lane

% Merging vehicle lateral movement
tAdj = 0;
aLat = sinLatAccel(t, tAdj, tLat, h);

% Longitudinal initial speeds
vm0 = 25; % [m/s] merging vehicle

% Assign speeds
mergingVeh.v0 = vm0;
for k = 1:length(otherVehs)
    switch otherVehs(k).name(1)
        case 'L'
            otherVehs(k).v0 = mergingVeh.v0 - (relVelRange(1):relVelRange(2));
        case 'F'
            otherVehs(k).v0 = mergingVeh.v0 + (relVelRange(1):relVelRange(2));
        otherwise
    end
end

% Duration of longitudinal acceleration phase
tLong = 10;

% Compute MSS: merging vehicle accel is determined by speed in the
% destination lane

for k = 1:length(otherVehs)
    tempStruct = struct(otherVehs(k).name, []);
    switch otherVehs(k).name(2)
        case 'd'
            am = zeros(length(t), length(otherVehs(k).v0));
            am(t<=tLong, :) = repmat((otherVehs(k).v0-vm0)/tLong, sum(t<=tLong), 1);
            mergingVeh.a = am;
            tempStruct = computeMSS(mergingVeh, otherVehs(k), aLat, t);
            MSS.(otherVehs(k).name) = tempStruct.(otherVehs(k).name);
        case 'o' % in this case, M's acceleration is still defined
            % by vehicle's in the destination lane, so we must
            % check for different values of vXd-vXo, X = {L, F}
            for n = 1:length(deltaDO)
                vD = otherVehs(k).v0 - deltaDO(n); % vel at destination lane
                am = zeros(length(t), length(otherVehs(k).v0));
                am(t<=tLong, :) = repmat((vD-vm0)/tLong, sum(t<=tLong), 1);
                mergingVeh.a = am;
                tempStruct(n) = computeMSS(mergingVeh, otherVehs(k), aLat, t);
            end
            MSS.(otherVehs(k).name) = [tempStruct(:).(otherVehs(k).name)];
            
    end
    
end

% Plots

figHandle = figure;

switch length(otherVehs)
    case 1
%         title(['MSS: switching accel: M and ' otherVehs.name])
        subplotDim = [1, 1];
    case 2
        sgtitle('MSS: switching accel')
        subplotDim = [1, 2];
    case {3, 4}
        sgtitle('MSS: switching accel')
        subplotDim = [2, 2];
    otherwise
        error('Too many vehicles being analyzed at once')
end

for k = 1:length(otherVehs)
    
    vOtherVeh = otherVehs(k).v0;
    nameOtherVeh = otherVehs(k).name;
    
    switch nameOtherVeh(1)
        case 'L'
            txt = 'vm - vl';
            deltaV = mergingVeh.v0 - vOtherVeh;
        case 'F'
            txt = 'vf - vm';
            deltaV = vOtherVeh - mergingVeh.v0;
        otherwise
    end
    
%     if length(otherVehs)>1 % assuming you either test a single vehicle or all four
        subplot(subplotDim(1), subplotDim(2), k);
%     end
    plot(deltaV, MSS.(nameOtherVeh));
    grid on;
    xlabel(['Rel long speed [m/s]: ' txt]);
    ylabel('MSS [m]');
    title(['M and ' nameOtherVeh])
    if nameOtherVeh(2)=='o' && length(deltaDO)>1
        legend({sprintf('v_o-v_d = %d', deltaDO(1)), ...
            sprintf('v_o-v_d = %d', deltaDO(2))}, 'Location', 'best')
    end
    
    % indicate safe region
    idx = deltaV == round(max(deltaV)/2);
    text(deltaV(idx), MSS.(nameOtherVeh)(idx), 'Safe Region \leftarrow', ...
        'HorizontalAlignment', 'right')

end

end