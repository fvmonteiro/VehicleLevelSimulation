classdef SafeVehicle < LongitudinalVehicleModel
    %SafeVehicle Longitudinal vehicle that uses CBFs

    properties
        hasLaneChangeIntention = false
        speedCBFparameter = 1
        safeDistCBFparameter = 1
        laneChangeDistCBFparameter = [0.01, 1]
    end

    properties (SetAccess = private)
        cbfValues % safe speed, safe dist, lane change dist
        isConnected
    end

    properties (Dependent, SetAccess = private)
        maxBrake
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
                % Temp - we can compute those
                obj.h = 1;
                obj.d0 = 1;
                obj.hLC = 2;
            end
        end

        function [] = includeInSimulation(obj, q0, tau)
            includeInSimulation@LongitudinalVehicleModel(obj, q0, tau);
            obj.cbfValues = zeros(length(obj.simTime), 3);
        end

        %%% Getters %%%
        function value = get.maxBrake(obj)
            value = abs(min(obj.accelBounds));
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
            u = min(nominalController, obj.computeSafeAccel());
            obj.inputs(obj.iterCounter + 1) = u;
        end

        %%% CBF related functions %%%
        function [safeAccel] = computeSafeAccel(obj)
            %computeSafeAccel determines the maximum safe control input
            % This must be moved either to a new class or to the controller
            % class

            safeAccel = max(obj.accelBounds);
            safeAccel = min(safeAccel, obj.safeSpeedCBF());
            if obj.hasLeader()
                safeAccel = min(safeAccel, obj.safeDistanceCBF());
                if obj.hasLaneChangeIntention
                    safeAccel = min(safeAccel, ...
                        obj.laneChangingDistanceCBF());
                end
            end
        end

        function [maxAccel] = safeSpeedCBF(obj)
            %safeSpeedCBF computes the CBF that ensures the vehicle
            %respects the maximum velocity constraint
            alpha = obj.speedCBFparameter;
            barrierFunctionValue = ...
                obj.desiredVelocity - obj.velocity;
            obj.cbfValues(obj.iterCounter, 1) = barrierFunctionValue;
            maxAccel = classKappaFunction('linear', barrierFunctionValue, ...
                alpha);
        end

        function [maxAccel] = safeDistanceCBF(obj)
            alpha = obj.safeDistCBFparameter;
            if ~obj.hasLeader()
                maxAccel = max(obj.accelBounds);
            else
                timeHeadway = obj.h;
                minAccel= obj.maxBrake;
                v2v = obj.isConnected && obj.leader.isConnected;
                k = obj.iterCounter;
                egoVelocity = obj.vx(k);
                barrierFunctionValue = obj.errorToDesiredGap(timeHeadway, ...
                    minAccel);
                obj.cbfValues(obj.iterCounter, 2) = barrierFunctionValue;
                term1 = minAccel / (egoVelocity + timeHeadway*minAccel);
                term2 = classKappaFunction('linear', barrierFunctionValue, ...
                    alpha) - egoVelocity;
                if v2v
                    leaderVelocity = obj.leader.vx(k);
                    leaderAccel = obj.leader.ax(k+1); % intended accel at k
                    connectedTerm = leaderVelocity*(1 ...
                    + leaderAccel/obj.leader.maxBrake);
                    term2 = term2 + connectedTerm;
                end
                maxAccel = term1 * term2;
            end
        end

        function [maxAccel] = laneChangingDistanceCBF(obj)
            rho = obj.laneChangeDistCBFparameter(1);
            gamma = obj.laneChangeDistCBFparameter(2);
            if ~obj.hasLeader()
                maxAccel = max(obj.accelBounds);
            else
                timeHeadway = obj.hLC;
                minAccel= abs(min(obj.accelBoundsDuringLC));
                v2v = obj.isConnected && obj.leader.isConnected;
                egoVelocity = obj.velocity;
                barrierFunctionValue = obj.errorToDesiredGap(timeHeadway, ...
                    minAccel);
                obj.cbfValues(obj.iterCounter, 3) = barrierFunctionValue;
                if barrierFunctionValue < 0
                    kappaType = 'exponential';
                    kappaParameters = [rho, gamma];
                else
                    kappaType = 'exponential';
                    kappaParameters = [1/rho, gamma]; %alphaDistance;
                end
                term1 = minAccel / (egoVelocity + timeHeadway*minAccel);
                term2 = classKappaFunction(kappaType, ...
                    barrierFunctionValue, kappaParameters) - egoVelocity;
                if v2v
                    k = obj.iterCounter;
                    leaderVelocity = obj.leader.vx(k);
                    leaderAccel = obj.leader.ax(k+1); % intended accel at k
                    connectedTerm = leaderVelocity*(1 ...
                    + leaderAccel/obj.leader.maxBrake);
                    term2 = term2 + connectedTerm;
                end
                maxAccel = term1 * term2;
            end
        end

%         function [value] = errorToVehicleFollowingGap(obj)
%             %safeVehicleFollowingDistanceBF computes the value of the 
%             % control barrier function for lane changing
%             value = obj.errorToDesiredGap(obj.h, obj.maxBrake);
%         end
% 
%         function [value] = errorToLaneChangingGap(obj)
%             %safeLaneChangeDistanceBF computes the value of the control
%             % barrier function for lane changing
%             value = obj.errorToDesiredGap(obj.hLC, ...
%                 abs(min(obj.accelBoundsDuringLC)));
%         end

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