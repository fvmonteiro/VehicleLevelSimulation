classdef LongitudinalScenario < handle
    %Scenario: class to help run Matlab scenarios

    properties
%         stopTime % total simulation time
%         simTime;
        samplingPeriod = 0.1
    end

    properties (SetAccess = protected)
        vehicleArray
    end

    properties (SetAccess = private)
        simTime
    end
    
    methods
%     function obj = LongitudinalScenario(stopTime)
% %         obj.stopTime = stopTime;
% %         obj.simTime = 0:samplingPeriod:stopTime;
%     end

    function [] = updateSamplingPeriod(obj, value)
        obj.samplingPeriod = value;
        obj.setStopTime(obj.simTime(end));
    end

    function setStopTime(obj, stopTime)
        obj.simTime = 0:obj.samplingPeriod:stopTime;
    end
    
    end
end