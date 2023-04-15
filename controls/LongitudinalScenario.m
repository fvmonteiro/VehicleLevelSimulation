classdef LongitudinalScenario < handle
    %Scenario: class to help run Matlab scenarios

    properties
%         stopTime % total simulation time
%         simTime;
        vehicleArray
    end
    
    properties (SetAccess = private)
        samplingPeriod = 0.1;
        simTime;
    end
    
    methods
%     function obj = LongitudinalScenario(stopTime)
% %         obj.stopTime = stopTime;
% %         obj.simTime = 0:samplingPeriod:stopTime;
%     end

    function setStopTime(obj, stopTime)
        obj.simTime = 0:obj.samplingPeriod:stopTime;
    end
    
%     function [] = runSingleStep(obj, reference)
%         for n = 1:length(obj.vehicleArray)
%             vehicle = obj.vehicleArray(n);
%             vehicle.singleStepUpdate(reference(n));
%         end
%     end
    
    end
end