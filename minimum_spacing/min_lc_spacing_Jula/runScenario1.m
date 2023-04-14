function [MSS, figHandle] = runScenario1(t, tLat, h, relVelRange,  otherVehs, mergingVeh)
%runScenario1 Run and plots scenario with constants speeds

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

% Const speed
% Merging vehicle lateral movement
tAdj = 0;
aLat = sinLatAccel(t, tAdj, tLat, h);

% Merging vehicle longitudinal movement
am = zeros(length(t), 1);
mergingVeh.a = am;

% Actually compute MSS
MSS = computeMSS(mergingVeh, otherVehs, aLat, t);

%% Plots
figHandle = figure;
sgtitle('MSS: const velocity')
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
    
    subplot(2, 2, k);
    plot(deltaV, MSS.(nameOtherVeh));
    grid on;
    xlabel(['Rel long speed [m/s]: ' txt]);
    ylabel('MSS [m]');
    title(['M and ' nameOtherVeh])
    
    % indicate safe region
%     idx1 = deltaV == round(min(deltaV)/2);
    idx = deltaV == round(max(deltaV)/2);
    
%     text(deltaV(idx1), MSS.(nameOtherVeh)(idx1), '\uparrow Safe Region', ...
%         'Position', [10 10])
    text(deltaV(idx), MSS.(nameOtherVeh)(idx), 'Safe Region \leftarrow', ...
        'HorizontalAlignment', 'right')
    
    
end

end