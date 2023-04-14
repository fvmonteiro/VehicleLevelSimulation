classdef Truck < Vehicle
    %Car Parameters for a regular car (passenger vehicle)
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = Truck()
            %Truck Just sets proper values
            obj.name = 'truck';
            % Truck params (using conservative params from Experimental 
            % Measurement of The Stopping Performance of A Tractor-
            % Semitrailer From Multiple Speeds)
            obj.maxAccel = 3;% (this one is not provided in the paper) [m/s^2]
            obj.minAccel = -5; % [m/s^2]
            obj.maxJerk = 40; % (this one is not provided in the paper) [m/s^3]
            timeToMinAccel = 0.5;
            obj.minJerk = obj.minAccel/timeToMinAccel; % [m/s^3]
            obj.reactionTime = 0.2; %[s]
            [obj.h, obj.d0] = obj.safeHeadway();
        end
        
    end
end

