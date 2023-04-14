classdef Scenario
    %Scenario: class containing all data necessary for testing certain
    %scenarios
    
    properties
        stopTime % total simulation time
        vehicleArray
%         nVehicles 
        nLanes
    end
    
    properties (SetAccess = private)
        % Constants and default values
        samplingPeriod = 0.01;
        laneWidth = 3.6; % [m]
        accelMin = -3;
        accelMax = 3;
    end
    
    methods
        
    function obj = Scenario(stopTime, nLanes)
        obj.stopTime = stopTime;
%         obj.nVehicles = nVehicles;
        obj.nLanes = nLanes;
    end
    
    function [] = runScenario(obj)
        % Sensing
        for k = 1:obj.samplingPeriod:obj.stopTime
            for veh = obj.vehicleArray
                veh.findLeader(obj.vehicleArray);
            end
        end
        
        % Control
        
        % Update states
    end
    
    end
end