function [figHandle] = runScenario4(t, tLat, ta, h, relVelRange, otherVehs, ...
    mergingVeh, deltaS0, deltaV0, aAdj)
%runScenario4 Run and plot results for scenario where merging vehicle  
% adjusts its position and speed before and during lane change

ts = t(2)-t(1);

switch length(otherVehs)
    case 1
%         title(['MSS: switching accel: M and ' otherVehs.name])
        subplotDim = [1, 1];
    case 2
        subplotDim = [1, 2];
    case {3, 4}
        subplotDim = [2, 2];
    otherwise
        error('Too many vehicles being analyzed at once')
end

% figHandle = gobjects(length(otherVehs), 1);
[~, figHandle] = runScenario2(t, tLat, h, relVelRange, otherVehs, mergingVeh, 5);
for k = 1:length(otherVehs)
    currentVeh = otherVehs(k);   
    
    switch currentVeh.name(1)
        case 'L'
            signAadj = -1;
        case 'F'
            signAadj = 1;
    end
    
    % Simulating pre-lane change adjustment
    currentVeh.v0 = mergingVeh.v0 + signAadj*deltaV0;
    
    subplot(subplotDim(1), subplotDim(2), k);
    hold on, grid on;
%     figure(currentHandle), hold on, grid on;
%     figure(figHandle(k)), hold on, grid on;
    
    legs = cell(length(aAdj)+1, 1);
    legs{1} = 'MSS';
    for n = 1:length(aAdj)
        longAccel = zeros(length(t), 1);
        longAccel(t<=ta) = signAadj*aAdj(n);
        mergingVeh.a = longAccel ;
        vM = mergingVeh.computeVel(ts);
        
        deltaV = -signAadj*vM + signAadj*currentVeh.v0;
        limit = vM>=0 & deltaV>=-deltaV0;
        deltaV = deltaV(limit);
        deltaS = deltaS0 - cumtrapz(ts, deltaV);
        % Condition to avoid aAdj that would cause collision
        if ~(currentVeh.name(2)=='o' && min(deltaS)<0)
            plot(deltaV, deltaS)
            legs{n+1} = ['aAdj = ' num2str(signAadj*aAdj(n))];
        end
    end
    text(deltaV0, deltaS0, {'\Delta v(0)', '\Delta x(0)'})
    text(deltaV(end), deltaS(end), {'\Delta v(T)', '\Delta x(T)'})
    legs = legs(~cellfun('isempty',legs));
    legend(legs, 'Location', 'best')
end

end

