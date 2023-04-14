classdef Car < Vehicle
    %Car Parameters for a regular car (passenger vehicle)
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = Car()
            %Car Just sets proper values
            obj.name = 'passenger vehicle';
            % Standard vehicle params
            obj.maxAccel = 4; %3; % [m/s^2]
            obj.minAccel = -8; %-3; % [m/s^2]
            obj.maxJerk = 40;%75; % (this one is not provided in the paper) [m/s^3]
            obj.minJerk = -75; %-4; % [m/s^3]
            obj.reactionTime = 0.1; %[s]
            [obj.h, obj.d0] = obj.safeHeadway();
        end
        
    end
end

