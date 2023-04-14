classdef VirtualVehicle
    %VirtualVehicle class represents non-existent vehicles which copy some
    %other vehicle's longitudinal position and speed
    % They are used to "force" gap creation by having the same longitudinal
    % position as a vehicle in an adjacent lane.
    
    properties
        originalVehicle
        latPosition
    end
    properties (Dependent)
        position
        velocity
    end
    
    methods
        function obj = VirtualVehicle(vehicle, latPosition)
            %VirtualVehicle Construct an instance of this class
            obj.originalVehicle = vehicle;
            obj.latPosition = latPosition;
        end
        
        function value = get.position(obj)
            value = obj.originalVehicle.position;
        end
        function value = get.velocity(obj)
            value = obj.originalVehicle.velocity;
        end
    end
end

