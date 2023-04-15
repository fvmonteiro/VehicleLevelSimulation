classdef SafeVehicle < LongitudinalVehicleModel
    %SafeVehicle Longitudinal vehicle that uses CBFs

    properties (Dependent, SetAccess = private)
        maxBrake
    end

    methods
        function obj = SafeVehicle(name, simTime, q0)
            if nargin == 0
                super_args = {};
            else
                % For now, SafeVehicle only has simple second order 
                % dynamics 
                super_args = {name, 'PV', simTime, q0, 0};   
            end

            obj = obj@LongitudinalVehicleModel(super_args{:});
            % Temp - we can compute those
            obj.h = 1;
            obj.d0 = 1;
        end

        %%% Getters %%%
        function value = get.maxBrake(obj)
            value = abs(min(obj.accelBounds));
        end

        function [u] = computeInput(obj, reference)
            % computeInput Temporary implementation. Sets the next input to
            % be the minimum between reference and the safe acceleration
            % @reference: next step's desired acceleration
            u = min(reference, obj.computeSafeAccel());
            obj.inputs(obj.iterCounter + 1) = u;
        end

        %%% CBF related functions %%%
        function [safeAccel] = computeSafeAccel(obj)
            %computeSafeAccel determines the maximum safe control input
            % This must be moved either to a new class or to the controller
            % class

            % We're assuming a linear functions for all class K functions
            alphaSpeed = 1;
            alphaDistance = 1;
            safeAccelSpeed = alphaSpeed*obj.safeSpeedCBF();

            if isempty(obj.leader)
                safeAccelDistance = max(obj.accelBounds);
            else
                egoVelocity = obj.velocity;
                leaderVelocity = obj.leader.velocity;
                leaderAccel = obj.leader.acceleration;
                term1 = obj.maxBrake/(egoVelocity + obj.h*obj.maxBrake);
                term2 = alphaDistance*obj.safeDistanceCBF() ...
                    + leaderVelocity - egoVelocity ...
                    + leaderVelocity*leaderAccel/obj.leader.maxBrake;
                safeAccelDistance = term1 * term2;
            end
            
            comfBrake = max(obj.comfAccelBounds);
            safeAccel = min([safeAccelSpeed, safeAccelDistance, ...
                comfBrake]);
        end

        function [value] = safeDistanceCBF(obj)
            %CBF computes the value of the control barrier function for
            %vehicle following
            % This must be moved either to a new class or to the controller
            % class
            if isempty(obj.leader)
                value = 1;  % anything positive
                return
            end
            gap = obj.leader.position - obj.position - obj.len;
            vel = obj.velocity;
            minimumGap = obj.h*vel + obj.d0 + vel^2/2/obj.maxBrake ...
                - obj.leader.velocity^2/2/obj.leader.maxBrake;
            value = gap - minimumGap;
        end

        function [value] = safeSpeedCBF(obj)
            value = obj.desiredVelocity - obj.velocity;
        end
    end
end