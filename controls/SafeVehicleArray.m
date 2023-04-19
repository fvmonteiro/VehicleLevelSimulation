classdef SafeVehicleArray < VehicleArray
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    methods
        function obj = SafeVehicleArray(nv)
            %SafeVehicleArray Construct an instance of this class
            %   We just create an array of vehicles
            obj.vehs = SafeVehicle.empty;
            obj.vehs(nv) = SafeVehicle();
        end

        function createVehicles(obj, names, simTime, initialStates, ...
                desiredVelocity, isConnected)
            for n = 1:length(obj.vehs)
                obj.vehs(n) = SafeVehicle(names{n}, simTime, ...
                    initialStates(:, n), isConnected);
                obj.vehs(n).desiredVelocity = desiredVelocity(n);
                if n > 1
                    obj.vehs(n).leader = obj.vehs(n-1);
                end
            end
        end

        function [] = singleStepUpdate(obj, desiredAccelerations)
            if nargin > 1
                m = length(desiredAccelerations);
            else
                m = 0;
            end
            for n = 1:length(obj.vehs)
                if n <= m
                    obj.vehs(n).singleStepUpdate(desiredAccelerations(n));
                else
                    obj.vehs(n).singleStepUpdate();
                end
            end
        end
    end
end