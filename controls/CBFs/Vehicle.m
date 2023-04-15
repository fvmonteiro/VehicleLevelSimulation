classdef Vehicle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        stateVector % states over entire simulation
    end
    
    methods
        function obj = Vehicle(initialState)
            %VEHICLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.stateVector = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.stateVector + inputArg;
        end
    end
end

