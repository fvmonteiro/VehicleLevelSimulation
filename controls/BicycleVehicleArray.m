classdef BicycleVehicleArray < VehicleArray
    %BicycleVehicleArray Class to easily deal with several Bicycle Vehicles

    methods
        function obj = BicycleVehicleArray(nv)
            %SafeVehicleArray Construct an instance of this class
            %   We just create an array of vehicles
            obj.vehs = BicycleVehicleModel.empty;
            obj.vehs(nv) = BicycleVehicleModel();
        end

        function createVehicles(obj, names, simTime, initialStates, ...
                desiredVelocity, isConnected)
            if length(desiredVelocity) == 1
                desiredVelocity = desiredVelocity * ones(length(obj.vehs));
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n) = BicycleVehicleModel(names{n}, simTime, ...
                    initialStates(:, n), desiredVelocity(n), isConnected);
                if n > 1
                    obj.vehs(n).leader = obj.vehs(n-1);
                end
            end
        end

        function [] = singleStepUpdate(obj, leaderInputs)
            for n = 1:length(obj.vehs)
                if nargin > 1 && n == 1
                    obj.vehs(n).computeInput(leaderInputs);
                else
                    obj.vehs(n).computeInput();
                end
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n).updateStates();
            end
        end
    end
end