classdef SafeController < handle

    properties
        speedCBFparameter = 1
        safeDistCBFparameter = 1
        longAdjustmentStartTime
%         longitudinalAdjustmentType = 1 % 1 for Finite Time CBF
                                       % 2 for Time Varying CBF
        % Finite Time CBF params
        leaderlaneChangeGapCBFparameter % rho, gamma for Finite Time CBF
        leaderRho
        leaderGamma
        destLaneLeaderRho
        destLaneLeaderGamma

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
                    beta = obj.computeLateralInput();
                case VehicleStates.longitudinalAdjustment
                    accel = obj.computeLongitudinalAdjustmentSafeAccel();
                    beta = obj.computeLateralInput();
                case VehicleStates.laneChanging
                    accel = obj.computeLaneChangeAccel();
                    beta = obj.computeLateralInput();
%                     [beta, accel] = obj.computeCombinedInput();
            end
            obj.iterCounter = obj.iterCounter + 1;
        end

        function [] = activateIncreaseGapToLeader(obj)
            obj.setFiniteTimeCBFParameters(obj.egoVehicle.leader);
        end

        function [] = activateCreateGapToDestLaneLeader(obj)
            obj.setFiniteTimeCBFParameters(obj.egoVehicle.destLaneLeader);
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
                realLeaderAccelBound = ...
                    obj.laneChangingGapFiniteTimeCBF(...
                    obj.egoVehicle.leader);
                safeAccel = min(safeAccel, realLeaderAccelBound);
            end
            if obj.egoVehicle.hasDestLaneLeader()
                virtualLeaderAccelBound = ...
                    obj.laneChangingGapFiniteTimeCBF(...
                    obj.egoVehicle.destLaneLeader);
                safeAccel = min(safeAccel, virtualLeaderAccelBound);
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

        function [safeBeta, safeAccel] = computeCombinedInput(obj)

            % NOTE: Easier to solve separately, because then there is no 
            % need to choose weights

            ego = obj.egoVehicle;
            v = ego.velocity;
            theta = ego.orientation;
            ey = ego.computeLateralPositionError();
            ev = ego.computeVelocityReferenceError();
            yRefDot = ego.computeLateralReferenceDerivative();
            vRefDot = ego.computeVelocityReferenceDerivative();

            wv = 0.000001;

            psi0 = 2*ey*(yRefDot - v*sin(theta)) ...
                + classKappaFunction('linear', ey^2, 1) ...
                + 2*wv*ev*vRefDot + classKappaFunction('linear', wv*ev^2, 1);
            psi1 = [-2*ey*v*cos(theta), -2*wv*ev]'; 
            uCLF = obj.minNormController(psi0, psi1);
            betaCLF = uCLF(1);
            accelCLF = uCLF(2);
            betaCBF = obj.lateralPositionCBF();
            safeBeta = min(betaCLF, betaCBF);
            maxLCaccel = max(ego.comfAccelBounds)/2; % arbitrary
            safeAccel = min([accelCLF, maxLCaccel,...
                obj.computeLongitudinalAdjustmentSafeAccel()]);
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

        function [maxAccel] = laneChangingGapFiniteTimeCBF(obj, leader)

            timeHeadway = obj.egoVehicle.hLC;
            minAccel= obj.egoVehicle.maxBrakeLaneChanging;
            v2v = obj.egoVehicle.isConnected && leader.isConnected;
            egoVelocity = obj.egoVehicle.velocity;
            barrierFunctionValue = ...
                obj.egoVehicle.computeErrorToLaneChangeGap(leader);
            obj.cbfValuesLog(obj.iterCounter, 3) = barrierFunctionValue;
            
            %%%% TODO %%%%
            rho = obj.leaderRho;
            gamma = obj.leaderGamma;

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

            timeHeadway0 = obj.leaderlaneChangeGapCBFparameter;
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
            maxOverShoot = 2;
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
                u = -(psi0*psi1)/(psi1'*psi1);
            else
                u = zeros(length(psi1), 1);
            end
        end

        function [minBeta, maxBeta] = ...
                lateralPositionWithInputConstraintsCBF(obj)
            %lateralPositionCBF CBF that prevents vehicle from overshooting
            %the lane change maneuver
            maxOverShoot = -1;
            ey = obj.egoVehicle.computeLateralErrorToDestLane();
            v = obj.egoVehicle.velocity;
            theta = obj.egoVehicle.orientation;

            % With input constraints
            betaBound = 0.01;

            barrierFunctionValue = ey + maxOverShoot ...
                - obj.egoVehicle.lr * (cos(theta + betaBound) + 1) ...
                / betaBound;
            minBeta = betaBound * (1 ...
                - classKappaFunction('linear', barrierFunctionValue, 1)...
                / v / sin(theta));
            maxBeta = -minBeta;

        end

        function [] = setFiniteTimeCBFParameters(obj, leader)
%             if obj.longitudinalAdjustmentStartFlag == true
            cbfValue = obj.egoVehicle.computeErrorToLaneChangeGap(leader);
            epsilon = 0.1;
            rho = epsilon;
            veh = obj.egoVehicle;
            c = veh.hLC*veh.maxBrakeLaneChanging;
            gamma = c / abs(cbfValue)^rho;
            if obj.egoVehicle.currentLane == leader.currentLane
                obj.leaderRho = rho;
                obj.leaderGamma = gamma;
            else
                obj.destLaneLeaderRho = rho;
                obj.destLaneLeaderGamma = gamma;
            end
            fprintf("Expected T: %.2f\n", ...
                veh.currentTime -cbfValue/(1-rho)/c);
%             obj.longitudinalAdjustmentStartFlag = false;
%             end
        end

        % NOT UP TO DATE
        function [] = setTimeVaryingCBFParameters(obj)
            warning('Time Varying CBF code not up to date')
            if obj.longitudinalAdjustmentStartFlag == true
                obj.longAdjustmentStartTime = obj.egoVehicle.currentTime;
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
                obj.leaderlaneChangeGapCBFparameter = virtualTimeHeadway;
                expectedTime = maxSpeed / maxBrakeLC ...
                    * log(timeHeadwayLC - virtualTimeHeadway + 1);
                fprintf("Expected T: %.2f\n", expectedTime);
                obj.longitudinalAdjustmentStartFlag = false;
            end
        end

    end
end