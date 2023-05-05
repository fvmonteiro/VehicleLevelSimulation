classdef SafeController < handle

    properties
        speedCBFparameter = 1
        safeDistCBFparameter = 1
        longAdjustmentStartTime
        % TODO: Make it a parameter chosen by the vehicle
        longitudinalAdjustmentType = 1 % 1 for Finite Time CBF
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

        function [accel, beta] = computeInputs(obj)
            switch obj.egoVehicle.maneuverState
                case VehicleStates.laneKeeping
                    accel = obj.computeLaneKeepingSafeAccel();
                case VehicleStates.longitudinalAdjustment
                    accel = obj.computeLongitudinalAdjustmentSafeAccel();
                case VehicleStates.laneChanging
                    accel = obj.computeLaneChangeAccel();
            end
            beta = obj.computeLateralInput();
            obj.iterCounter = obj.iterCounter + 1;
        end

        function [] = activateLaneChangingCBF(obj)
            obj.longitudinalAdjustmentStartFlag = true;
            obj.longAdjustmentStartTime = obj.egoVehicle.currentTime;
        end

        %%% Inputs for each phase %%%
        function [safeAccel] = computeLaneKeepingSafeAccel(obj)
            %computeLaneKeepingSafeAccel determines the maximum safe 
            % acceleration during lane keeping
            safeAccel = max(obj.egoVehicle.accelBounds);
            safeAccel = min(safeAccel, obj.safeSpeedCBF());
            if obj.egoVehicle.hasLeader()
                safeAccel = min(safeAccel, obj.safeDistanceCBF());
            end
        end

        function [safeAccel] = computeLongitudinalAdjustmentSafeAccel(obj)
            %computeLongitudinalAdjustmentSafeAccel determines the maximum 
            % acceleration input during longitudinal adjustment

            safeAccel = obj.computeLaneKeepingSafeAccel();
            if obj.egoVehicle.hasLeader()
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

        function [safeAccel] = computeLaneChangeAccel(obj)
            %computeLaneChangeInputs determines the maximum
            % safe control input during longitudinal adjustment
            ego = obj.egoVehicle;
            ev = ego.computeVelocityReferenceError();
            vRefDot = ego.computeVelocityReferenceDerivative();
            psi0 = 2*ev*vRefDot + classKappaFunction('linear', ev^2, 1);
            psi1 = -2*ev;
            accelCLF = obj.minNormController(psi0, psi1);

            maxLCaccel = max(ego.comfAccelBounds)/2; % arbitrary
            safeAccel = min([accelCLF, maxLCaccel,...
                obj.computeLongitudinalAdjustmentSafeAccel()]);
        end

        function [safeBeta] = computeLateralInput(obj)
            ego = obj.egoVehicle;
            ey = ego.computeLateralPositionError();
            v = ego.velocity;
            theta = ego.orientation;
            yRefDot = ego.computeLateralReferenceDerivative();
            psi0 = 2*ey*(yRefDot - v*sin(theta)) ...
                + classKappaFunction('linear', ey^2, 1);
            psi1 = -2*ey*v*cos(theta);
            betaCLF = obj.minNormController(psi0, psi1);

            betaCBF = obj.lateralPositionCBF();
            safeBeta = min(betaCLF, betaCBF);
        end

%         function [safeAccel, safeBeta] = CLF_CBF_QP(obj)
%             H = [0.01, 0; 0, 0]; % based on He et al 2021, ACC
%             F = [0; 0; 0; 0]; % must have some value for the deltas, right?
%         end
    end

    methods (Access = private)

        function [maxAccel] = safeSpeedCBF(obj)
            %safeSpeedCBF computes the CBF that ensures the vehicle
            %respects the maximum velocity constraint
            alpha = obj.speedCBFparameter;
            barrierFunctionValue = ...
                obj.egoVehicle.computeVelocityReferenceError();
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
            barrierFunctionValue = ...
                obj.egoVehicle.computeErrorToLaneKeepingGap();
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
            minAccel= obj.egoVehicle.maxBrakeLaneChanging;
            leader = obj.egoVehicle.leader;
            v2v = obj.egoVehicle.isConnected && leader.isConnected;
            egoVelocity = obj.egoVehicle.velocity;
            barrierFunctionValue = ...
                obj.egoVehicle.computeErrorToLaneChangeGap();
            obj.cbfValuesLog(obj.iterCounter, 3) = barrierFunctionValue;
            
            obj.setFiniteTimeCBFParameters();
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
                leaderVelocity = leader.velocity;
                leaderAccel = leader.acceleration;
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
            minAccel = obj.egoVehicle.maxBrakeLaneChanging;
            time = obj.egoVehicle.currentTime;
            exponent = minAccel / obj.egoVehicle.desiredVelocity;
            h1 = exp(exponent * (time - obj.longAdjustmentStartTime)) ...
                + timeHeadway0 - 1;

            if h1 <= timeHeadwayLC
                timeHeadway = h1;
                dhdt = exponent ...
                    * exp(exponent * (time - obj.longAdjustmentStartTime));
            else
                timeHeadway = timeHeadwayLC;
                dhdt = 0;
            end
            
            leader = obj.egoVehicle.leader;
            v2v = obj.egoVehicle.isConnected && leader.isConnected;
            egoVelocity = obj.egoVehicle.velocity;
            barrierFunctionValue = obj.egoVehicle.computeErrorToDesiredGap(...
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

        function [maxBeta] = lateralPositionCBF(obj)
            %lateralPositionCBF CBF that prevents vehicle from overshooting
            %the lane change maneuver
            maxOverShoot = 3;
            ey = obj.egoVehicle.computeLateralErrorToDestLane();
            v = obj.egoVehicle.velocity;
            theta = obj.egoVehicle.orientation;

            barrierFunctionValue = ey + maxOverShoot;
            maxBeta = classKappaFunction('linear', barrierFunctionValue, 1) ...
                / v / cos(theta) - tan(theta);
        end

        function [u] = minNormController(~, psi0, psi1)
            %minNormController Computes the minimum norm controller that
            %satisfies the CLF constraints described by psi0 and psi1
            if psi0 > 0
                u = -psi0/psi1;
            else
                u = 0;
            end
        end

        function [] = setFiniteTimeCBFParameters(obj)
            if obj.longitudinalAdjustmentStartFlag == true
                cbfValue = obj.egoVehicle.computeErrorToLaneChangeGap();
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
                maxSpeed = obj.egoVehicle.desiredVelocity;

                safeDistanceCBF = ...
                    obj.egoVehicle.computeErrorToLaneKeepingGap();
                virtualTimeHeadway = timeHeadwayLK ...
                    + maxSpeed * (1/maxBrakeLK - 1/maxBrakeLC) / 2 ...
                    + safeDistanceCBF / maxSpeed;
                obj.laneChangeGapCBFparameter = virtualTimeHeadway;
                expectedTime = maxSpeed / maxBrakeLC ...
                    * log(timeHeadwayLC - virtualTimeHeadway + 1);
                fprintf("Expected T: %.2f\n", expectedTime);
                obj.longitudinalAdjustmentStartFlag = false;
            end
        end

    end
end