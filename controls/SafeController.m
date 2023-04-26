classdef SafeController < handle

    properties
        speedCBFparameter = 1
        safeDistCBFparameter = 1
        longAdjustmentStartTime
        longitudinalAdjustmentType = 2 % 1 for Finite Time CBF
                                       % 2 for Time Varying CBF
        laneChangeGapCBFparameter % rho, gamma for Finite Time CBF
                                  % h0 for Time Varying CBF 
        longitudinalAdjustmentStartFlag = false
    end

    properties (SetAccess = private)
        cbfValuesLog % safe speed, safe dist, lane change dist
        egoVehicle
        iterCounter = 1
    end

    methods

        function obj = SafeController(egoVehicle)
            obj.egoVehicle = egoVehicle;
            obj.cbfValuesLog = zeros(length(egoVehicle.simTime), 3);
        end

        function [u] = computeInputs(obj)
            u = obj.computeSafeAccel();
            obj.iterCounter = obj.iterCounter + 1;
        end

        function [] = activateLaneChangingCBF(obj)
            obj.longitudinalAdjustmentStartFlag = true;
            obj.longAdjustmentStartTime = obj.egoVehicle.currentTime;
        end

        %%% During lane keeping and longitudinal adjustments %%%
        function [safeAccel] = computeSafeAccel(obj)
            %computeSafeAccel determines the maximum safe control input
            % This must be moved either to a new class or to the controller
            % class

            safeAccel = max(obj.egoVehicle.accelBounds);
            safeAccel = min(safeAccel, obj.safeSpeedCBF());
            if obj.egoVehicle.hasLeader()
                safeAccel = min(safeAccel, obj.safeDistanceCBF());
                if obj.egoVehicle.hasLaneChangeIntention
                    if obj.longitudinalAdjustmentType == 1
                    longAdjustmentAccel = ...
                        obj.laneChangingGapFiniteTimeCBF();
                    elseif obj.longitudinalAdjustmentType == 2
                        longAdjustmentAccel = ...
                        obj.laneChangingGapTimeVaryingCBF();
                    end
                    safeAccel = min(safeAccel, longAdjustmentAccel);
                end
            end
        end

        function [maxAccel] = safeSpeedCBF(obj)
            %safeSpeedCBF computes the CBF that ensures the vehicle
            %respects the maximum velocity constraint
            alpha = obj.speedCBFparameter;
            barrierFunctionValue = ...
                obj.egoVehicle.desiredVelocity - obj.egoVehicle.velocity;
            obj.cbfValuesLog(obj.iterCounter, 1) = barrierFunctionValue;
            maxAccel = classKappaFunction('linear', barrierFunctionValue, ...
                alpha);
        end

        function [maxAccel] = safeDistanceCBF(obj)
            alpha = obj.safeDistCBFparameter;

            timeHeadway = obj.egoVehicle.h;
            minAccel= obj.egoVehicle.maxBrake;
            leader = obj.egoVehicle.leader;
            v2v = obj.egoVehicle.isConnected && leader.isConnected;
            egoVelocity = obj.egoVehicle.velocity;
            barrierFunctionValue = obj.egoVehicle.errorToDesiredGap(...
                timeHeadway, minAccel);
            obj.cbfValuesLog(obj.iterCounter, 2) = barrierFunctionValue;
            term1 = minAccel / (egoVelocity + timeHeadway*minAccel);
            term2 = classKappaFunction('linear', barrierFunctionValue, ...
                alpha) - egoVelocity;
            if v2v
                leaderVelocity = leader.velocity;
                leaderAccel = leader.acceleration; % intended accel at k
                connectedTerm = leaderVelocity*(1 ...
                    + leaderAccel/leader.maxBrake);
                term2 = term2 + connectedTerm;
            end
            maxAccel = term1 * term2;
        end

        function [maxAccel] = laneChangingGapFiniteTimeCBF(obj)

            timeHeadway = obj.egoVehicle.hLC;
            minAccel= abs(min(obj.egoVehicle.accelBoundsDuringLC));
            leader = obj.egoVehicle.leader;
            v2v = obj.egoVehicle.isConnected && leader.isConnected;
            egoVelocity = obj.egoVehicle.velocity;
            barrierFunctionValue = obj.egoVehicle.errorToDesiredGap( ...
                timeHeadway, minAccel);
            obj.cbfValuesLog(obj.iterCounter, 3) = barrierFunctionValue;
            
            obj.setFiniteTimeCBFParameters(barrierFunctionValue);
            rho = obj.laneChangeGapCBFparameter(1);
            gamma = obj.laneChangeGapCBFparameter(2);

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
                leaderVelocity = leader.vx(k);
                leaderAccel = leader.ax(k+1); % intended accel at k
                connectedTerm = leaderVelocity*(1 ...
                    + leaderAccel/leader.maxBrake);
                term2 = term2 + connectedTerm;
            end
            maxAccel = term1 * term2;
        end

        function [maxAccel] = laneChangingGapTimeVaryingCBF(obj)

            alpha = obj.safeDistCBFparameter;
            obj.setTimeVaryingCBFParameters();

            timeHeadway0 = obj.laneChangeGapCBFparameter;
            timeHeadwayLC = obj.egoVehicle.hLC;
            minAccel= obj.egoVehicle.maxBrakeLaneChanging;
            time = obj.egoVehicle.currentTime;
            exponent = minAccel / obj.egoVehicle.desiredVelocity;
            h1 = exp(exponent * (time - obj.longAdjustmentStartTime)) ...
                + timeHeadway0 - 1;

            if h1 <= timeHeadwayLC
                timeHeadway = h1;
                dhdt = exponent * h1;
            else
                timeHeadway = timeHeadwayLC;
                dhdt = 0;
            end
            
            leader = obj.egoVehicle.leader;
            v2v = obj.egoVehicle.isConnected && leader.isConnected;
            egoVelocity = obj.egoVehicle.velocity;
            barrierFunctionValue = obj.egoVehicle.errorToDesiredGap(...
                timeHeadway, minAccel);
            obj.cbfValuesLog(obj.iterCounter, 3) = barrierFunctionValue;
            term1 = minAccel / (egoVelocity + timeHeadway*minAccel);
            term2 = classKappaFunction('linear', barrierFunctionValue, ...
                alpha) - egoVelocity * (1 + dhdt);
            if v2v
                leaderVelocity = leader.velocity;
                leaderAccel = leader.acceleration; % intended accel at k
                connectedTerm = leaderVelocity*(1 ...
                    + leaderAccel/leader.maxBrake);
                term2 = term2 + connectedTerm;
            end
            maxAccel = term1 * term2;
        end
    end

    methods (Access = private)

        function [] = setFiniteTimeCBFParameters(obj, cbfValue)
            if obj.longitudinalAdjustmentStartFlag == true
                epsilon = 0.1;
                rho = epsilon;
                veh = obj.egoVehicle;
                c = veh.hLC*veh.maxBrakeLaneChanging;
                gamma = c / abs(cbfValue)^rho;
                obj.laneChangeGapCBFparameter = [rho, gamma];
                fprintf("Expected T: %.2f\n", -cbfValue/(1-rho)/c);
                obj.longitudinalAdjustmentStartFlag = false;
            end
        end

        function [] = setTimeVaryingCBFParameters(obj)
            if obj.longitudinalAdjustmentStartFlag == true
                timeHeadwayLK = obj.egoVehicle.h;
                timeHeadwayLC = obj.egoVehicle.hLC;
                maxBrakeLK = obj.egoVehicle.maxBrake;
                maxBrakeLC = obj.egoVehicle.maxBrakeLaneChanging;
                egoSpeed = obj.egoVehicle.velocity;

                safeDistanceCBF = obj.egoVehicle.errorToDesiredGap(...
                    timeHeadwayLK, maxBrakeLK);
                virtualTimeHeadway = timeHeadwayLK ...
                    + egoSpeed * (1/maxBrakeLK - 1/maxBrakeLC) / 2 ...
                    + safeDistanceCBF / egoSpeed;
                obj.laneChangeGapCBFparameter = virtualTimeHeadway;
                expectedTime = egoSpeed / maxBrakeLC ...
                    * log(timeHeadwayLC/virtualTimeHeadway);
                fprintf("Expected T: %.2f\n", expectedTime);
                obj.longitudinalAdjustmentStartFlag = false;
            end
        end

    end
end