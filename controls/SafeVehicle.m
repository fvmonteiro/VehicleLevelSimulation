classdef SafeVehicle < LongitudinalVehicleModel
    %SafeVehicle Longitudinal vehicle that uses CBFs

    properties
        hasLaneChangeIntention = false
        speedCBFparameter = 1
        safeDistCBFparameter = 1
        laneChangeDistCBFparameter = [0.01, 1]
    end

    properties (SetAccess = private)
        isConnected
    end

    methods
        function obj = SafeVehicle(name, simTime, q0, isConnected)
            if nargin == 0
                super_args = {};
            else
                % For now, SafeVehicle only has simple second order 
                % dynamics 
                super_args = {name, 'PV', simTime, q0, 0};
            end

            obj = obj@LongitudinalVehicleModel(super_args{:});
            if nargin > 0
                obj.isConnected = isConnected;
                obj.controller = SafeController(obj);
                % Temp - we can compute those
                obj.h = 1;
                obj.d0 = 1;
                obj.hLC = 2;
            end
        end

        function [] = includeInSimulation(obj, q0, tau)
            includeInSimulation@LongitudinalVehicleModel(obj, q0, tau);
        end

        %%% Methods %%%

        function [u] = computeInput(obj, reference)
            % computeInput Temporary implementation. Sets the next input to
            % be the minimum between reference and the safe acceleration
            % @reference: next step's desired acceleration

            if nargin > 1
                nominalController = reference;
            elseif obj.hasLeader()
                nominalController = max(obj.accelBounds);
            else
                nominalController = 0;
            end
            % u = min(nominalController, obj.computeSafeAccel());
            u = min(nominalController, obj.controller.computeInputs());
            obj.inputs(obj.iterCounter + 1) = u;
        end

        function [value] = errorToDesiredGap(obj, timeHeadway, egoMaxBrake)
            %errorToDesiredGap computes difference between current gap and
            %the desired (collision free) gap

            gap = obj.computeCurrentGapToLeader();
            k = obj.iterCounter;
            egoVel = obj.vx(k);
            leaderVel = obj.leader.vx(k);
            minimumGap = timeHeadway*egoVel + obj.d0 ...
                + egoVel^2/2/egoMaxBrake ...
                - leaderVel^2/2/obj.leader.maxBrake;
            value = gap - minimumGap;
        end

    end

end