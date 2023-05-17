classdef VehicleStates
    enumeration
        laneKeeping, longitudinalAdjustment, laneChanging, ...
            cooperating
        % Implicit assumption: vehicle only cooperates if it has not lane
        % change intention
    end
end