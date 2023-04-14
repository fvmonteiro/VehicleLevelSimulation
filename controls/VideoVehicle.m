classdef VideoVehicle < Vehicle
    %VideoVehicle Class to unify simulink and matlab simulated vehicles
    %keeping only the necessary variables to create videos
    
    properties
        x
        y
        vx
        ax
        psi
        u
        delta
        
        lcState
    end
    
    properties (Dependent)
        x0
        y0
        vx0
    end
    
    methods
        function [] = copySimulinkVehicle(obj, vehicle)
            obj.name = vehicle.name;
            obj.width = vehicle.width;
            obj.simTime = vehicle.simTime;
            obj.x = vehicle.x;
            obj.y = vehicle.y;
            obj.vx = vehicle.vx;
            obj.ax = vehicle.ax;
            obj.psi = vehicle.psi;
            obj.u = vehicle.u;
            obj.delta = vehicle.delta;
            
            obj.lcState = vehicle.lcState;
            if isempty(obj.lcState)
                obj.lcState = timeseries(0, obj.simTime);
            end
        end
        
        function [] = copyLongitudinalModelVehicle(obj, vehicle)
            obj.name = vehicle.name;
            obj.width = 1.8;
            obj.simTime = vehicle.simTime;
            obj.x = vehicle.x;
            obj.y = vehicle.y*ones(length(obj.simTime), 1);
            obj.vx = vehicle.vx;
            obj.ax = vehicle.ax;
            obj.psi = zeros(length(obj.simTime), 1);
            obj.u = vehicle.u;
            obj.delta = zeros(length(obj.simTime), 1);
            
            obj.lcState = timeseries(0, obj.simTime);
        end
    
        %%% GETTERS %%%
        function [value] = get.x0(obj)
           value = obj.x(1); 
        end
        
        function [value] = get.y0(obj)
           value = obj.y(1); 
        end
        
        function [value] = get.vx0(obj)
           value = obj.vx(1); 
        end
    end
end