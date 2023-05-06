classdef BicycleVehicleModel < LongitudinalVehicleModel
    %VehicleBicycleModel Vehicle model including longitudinal and lateral
    %movement
    
    % In this model we control linear acceleration 'a' and steering wheel
    % angle 'delta' separately.
    % States:
    % q = [x, y, theta, v]'
    % Inputs:
    % u = [a, delta(beta)]
    % Dynamics
    % dx/dt = v.cos(theta + beta)
    % dy/dt = v.sin(theta + beta)
    % dtheta/dt = v/lr sin(beta)
    % dv/dt = a
    % beta = arctan(lr/(lf+lr) tan(delta))
    properties
        yRefLog
        vRefLog

        laneWidth = 4
    end

    properties (SetAccess = private, Dependent)
        % States and inputs over entire simulation
        y
        theta % vehicle orientation
        beta % angle between veh orientation and veh speed

        % Current values
        lateralPosition
        orientation
    end

    properties (SetAccess = private)
        % ayBounds = [-0.67, 0.67]
        % hLat = 0.5;
        lf = 1 % from c.g. to front axle
        lr = 1.5 % from c.g. to rear axle
        isConnected
        delta % front wheel steering angle
        maneuverState = VehicleStates.laneKeeping
        originLane
        currentLane
        destinationLane
        laneChangeTrajectoryCoeffs
        laneChangeVelocityTrajectoryCoeffs
        laneChangeStartTime
        laneChangeDesiredDuration = 5

        destLaneLeader
        destLaneFollower
    end
    
    methods
        function obj = BicycleVehicleModel(name, simTime, x0, v0,...
                initialLane, desiredVelocity, isConnected)
            %BicycleVehicleModel Construct an instance of this class
            
            % Set variables indices for this class
            obj.statesIdx.x = 1;
            obj.statesIdx.y = 2;
            obj.statesIdx.theta = 3;
            obj.statesIdx.vx = 4;
            
            % Set input indices
            obj.inputsIdx.u = 1;
            obj.inputsIdx.beta = 2;
            
            if nargin > 0
                obj.name = name;
                obj.setVehicleParams('PV');
                obj.simTime = simTime;
                obj.desiredVelocity = desiredVelocity;
                obj.isConnected = isConnected;
                obj.setInitialLaneValues(initialLane);
                q0 = [x0; initialLane * obj.laneWidth; 0; v0];
                obj.includeInSimulation(q0);
                obj.controller = SafeController(obj);
                % Temp - we can compute those
                obj.h = 1;
                obj.d0 = 1;
                obj.hLC = 2;
            else

            end

        end
        
        %%% GETTERS %%%
        function value = get.theta(obj)
            value = obj.states(:, obj.statesIdx.theta);
        end

        function value = get.y(obj)
            value = obj.states(:, obj.statesIdx.y);
        end

        function value = get.beta(obj)
            value = obj.inputs(:, obj.inputsIdx.beta);
        end

        function value = get.lateralPosition(obj)
            value = obj.states(obj.iterCounter, obj.statesIdx.y);
        end

        function value = get.orientation(obj)
            value = obj.states(obj.iterCounter, obj.statesIdx.theta);
        end

        %%%%%%%%%%%%%%%
        
        function [] = setInitialLaneValues(obj, lane)
            obj.originLane = lane;
            obj.currentLane = lane;
            obj.destinationLane = lane;
        end

        function [] = setLaneChangeDirection(obj, value)
            obj.destinationLane = obj.destinationLane + value;
        end

        function [bool] = hasLaneChangeIntention(obj)
            bool = obj.currentLane ~= obj.destinationLane;
        end

        %%% SURROUNDING VEHICLES %%%
        function [bool] = hasDestLaneLeader(obj)
            bool = ~isempty(obj.destLaneLeader);
        end

        function [bool] = hasDestLaneFollower(obj)
            bool = ~isempty(obj.destLaneFollower);
        end

        function [] = analyzeSurroundingVehicles(obj, otherVehicles)
            obj.findLeader(otherVehicles);
            obj.findDestinationLaneLeader(otherVehicles);
        end
        function [] = findLeader(obj, otherVehicles)
            %findLeader recieves an array of vehicles, determines which are
            %on the same lane, then determines which one is longitudinally 
            % closest and makes it the leader.
            closestX = inf;
            leaderIdx = -1;
            for k = 1:length(otherVehicles)
                otherVeh = otherVehicles(k);
                relativeX = otherVeh.position - obj.position;
                if obj.isOnSameLane(otherVehicles(k)) ...
                        && obj.isOtherAhead(otherVeh) ...
                        && relativeX < closestX
                    closestX = relativeX;
                    leaderIdx = k;
                end
            end

            if leaderIdx > 0
                obj.leader = otherVehicles(leaderIdx);
            else
                obj.leader = [];
            end
        end

        function [] = findDestinationLaneVehicles(obj, otherVehicles)
            obj.destLaneLeader = [];
            obj.destLaneFollwer = [];
            if obj.hasLaneChangeIntention()
                closestAhead = inf;
                closestBehind = -Inf;
                for k = 1:length(otherVehicles)
                    otherVeh = otherVehicles(k);
                    relativeX = otherVeh.position - obj.position;
                    if obj.destinationLane == otherVehicles(k).currentLane
                        if obj.isOtherAhead(otherVeh) ...
                                && relativeX < closestAhead
                            closestAhead = relativeX;
                            obj.destLaneLeader = otherVeh;
                        elseif obj.isOtherBehind(otherVeh) ...
                                && relativeX > closestBehind
                            closestBehind = relativeX;
                            obj.destLaneFollower = otherVeh;
                        end
                    end
                end
            end
        end

        %%% UPDATING METHODS %%%
        function [q] = updateStates(obj)
            %singleStepUpdate computes system states after one step given
            %inputs computed in the last step

            % Current states and inputs
            xNow = obj.position;
            yNow = obj.lateralPosition;
            vNow = obj.velocity;
            thetaNow = obj.orientation;
         
            axNow = obj.acceleration;
            betaNow = obj.beta(obj.iterCounter);
            
            % Increments
            sampling = obj.simTime(obj.iterCounter + 1) ...
                - obj.simTime(obj.iterCounter);
            dx = vNow*cos(thetaNow + betaNow) * sampling;
            dy = vNow*sin(thetaNow + betaNow) * sampling;
            dTheta = vNow / obj.lr * sin(betaNow) * sampling;
            dv = axNow*sampling;

            % Update states
            q = [xNow+dx; yNow+dy; thetaNow+dTheta; vNow+dv];
            q(3) = rem(q(3), 2*pi);  % keeps theta within 2pi
            if q(4) < 0 % prevents negative speeds
                q(4) = 0;
            end

            % Increase counter and update states log
            obj.iterCounter = obj.iterCounter + 1;
            obj.states(obj.iterCounter, :) = q;
            obj.delta(obj.iterCounter) = atan(...
                tan(betaNow) * (obj.lf + obj.lr) / obj.lr);
            obj.updateCurrentLane();
            obj.updateManeuverState();
        end

        function [u1, u2] = computeInput(obj, externalInput)
            % Nominal controller
            if nargin > 1
                u1 = externalInput(1);
                u2 = externalInput(2);
            else
                [u1, u2] = obj.controller.computeInputs();
            end
            obj.inputs(obj.iterCounter, :) = [u1, u2];
        end

        %%% TRACKING/ REFERENCE ERRORS %%%
        function [value] = computeErrorToDesiredGap(obj, timeHeadway, ...
                egoMaxBrake)
            %errorToDesiredGap computes the difference between current gap 
            % and the desired (collision free) gap
            gap = obj.computeCurrentGapToLeader();
            egoVel = obj.velocity;
            leaderVel = obj.leader.velocity;
            minimumGap = timeHeadway*egoVel + obj.d0 ...
                + egoVel^2/2/egoMaxBrake ...
                - leaderVel^2/2/obj.leader.maxBrake;
            value = gap - minimumGap;
        end

        function [value] = computeErrorToLaneKeepingGap(obj)
            %errorToLaneKeepingGap computes the difference between current 
            % gap and the safe gap during lane keeping
            value = obj.computeErrorToDesiredGap(obj.h, ...
                obj.maxBrake);
        end

        function [value] = computeErrorToLaneChangeGap(obj)
            %errorToLaneChangeGap computes the difference between current 
            % gap and the lane changing gap
            value = obj.computeErrorToDesiredGap(obj.hLC, ...
                obj.maxBrakeLaneChanging);
        end

        function [ey] = computeLateralPositionError(obj)
            if obj.maneuverState == VehicleStates.laneChanging
                if obj.currentTime <= (obj.laneChangeStartTime ...
                        + obj.laneChangeDesiredDuration)
                    yRef = obj.originLane * obj.laneWidth;
                    c = obj.laneChangeTrajectoryCoeffs;
                    for k = 1:length(c)
                        yRef = yRef ...
                            + c(k)*(obj.currentTime ...
                            - obj.laneChangeStartTime)^(k-1);
                    end
                else
                    yRef = obj.destinationLane * obj.laneWidth;
                end
            else
                yRef = obj.currentLane * obj.laneWidth;
            end
            obj.yRefLog(obj.iterCounter) = yRef;
            ey = yRef - obj.lateralPosition;
        end

        function [ey] = computeLateralErrorToDestLane(obj)
            yDestLane = obj.destinationLane * obj.laneWidth;
            ey = yDestLane - obj.lateralPosition;
        end

        function [yRefDot] = computeLateralReferenceDerivative(obj)
            yRefDot = 0;
            if obj.maneuverState == VehicleStates.laneChanging ...
                    && obj.currentTime <= (obj.laneChangeStartTime ...
                    + obj.laneChangeDesiredDuration)
                c = obj.laneChangeTrajectoryCoeffs;
                for k = 2:length(c)
                    yRefDot = yRefDot ...
                        + (k-1) * c(k)...
                        *(obj.currentTime - obj.laneChangeStartTime)^(k-2);
                end
            else
                yRefDot = 0;
            end
        end

        function [ev] = computeVelocityReferenceError(obj)
            vRef = obj.desiredVelocity;
            if obj.maneuverState == VehicleStates.laneChanging ...
                    && obj.currentTime <= (obj.laneChangeStartTime ...
                    + obj.laneChangeDesiredDuration)
                c = obj.laneChangeVelocityTrajectoryCoeffs;
                vRef = 0;
                for k = 1:length(c)
                    vRef = vRef + c(k)...
                        *(obj.currentTime - obj.laneChangeStartTime)^(k-1);
                end
            end
            obj.vRefLog(obj.iterCounter) = vRef;
            ev = vRef - obj.velocity;
        end

        function [vRefDot] = computeVelocityReferenceDerivative(obj)
            vRefDot = 0;
            if obj.maneuverState == VehicleStates.laneChanging ...
                    && obj.currentTime <= (obj.laneChangeStartTime ...
                    + obj.laneChangeDesiredDuration)
                c = obj.laneChangeVelocityTrajectoryCoeffs;
                for k = 2:length(c)
                    vRefDot = vRefDot + (k-1)*c(k)...
                        *(obj.currentTime - obj.laneChangeStartTime)^(k-2);
                end
            end
        end

    end

    methods (Access = private)

        function [] = includeInSimulation(obj, q0)
            %includeInSimulation sets the vehicle's initial state and
            % initializes state matrix

            % Create all matrices
            n = length(obj.simTime);
            obj.inputs = zeros(n, obj.nInputs);
            obj.delta = zeros(n, 1);
            obj.states = zeros(n, obj.nStates);
            obj.yRefLog = zeros(n, 1);
            obj.vRefLog = zeros(n, 1);
            
            % Initialize
            obj.states(obj.iterCounter, :) = q0;
        end

        function [] = computeLaneChangeTrajectoryCoefficients(obj)
            tlc = obj.laneChangeDesiredDuration;
            Y0 = [obj.currentLane*obj.laneWidth, 0, 0]';
            Yf = [obj.destinationLane*obj.laneWidth, 0, 0]';
            obj.laneChangeTrajectoryCoeffs = findPathCoefficients(...
                tlc, Y0, Yf);
        end

        function [] = computeLaneChangeVelocityTrajectoryCoefficients(obj)
            tlc = obj.laneChangeDesiredDuration;
            V0 = [obj.velocity, 0]';
            if obj.hasDestLaneLeader()
                vd = obj.destLaneLeader.velocity;
            else
                vd = obj.desiredVelocity;
            end
            Vf = [vd, 0]'; 
            obj.laneChangeVelocityTrajectoryCoeffs = findPathCoefficients(...
                tlc, V0, Vf);
        end

        function [] = updateCurrentLane(obj)
            if obj.currentLane ~= obj.destinationLane
                currentLaneY = obj.currentLane * obj.laneWidth;
                desiredLaneY = obj.destinationLane * obj.laneWidth;
                if abs(desiredLaneY - obj.lateralPosition) ...
                        < abs(currentLaneY - obj.lateralPosition)
                    obj.currentLane = obj.destinationLane;
                end
            end
        end

        function [] = updateManeuverState(obj)
            oldState = obj.maneuverState;
            if obj.maneuverState == VehicleStates.laneKeeping
                if obj.hasLaneChangeIntention()
                    obj.controller.activateLaneChangingCBF();
                    obj.maneuverState = ...
                        VehicleStates.longitudinalAdjustment;
                end
            elseif obj.maneuverState == VehicleStates.longitudinalAdjustment
                if obj.computeErrorToLaneChangeGap >= 0
                    obj.computeLaneChangeTrajectoryCoefficients();
                    obj.computeLaneChangeVelocityTrajectoryCoefficients();
                    obj.laneChangeStartTime = obj.currentTime;
                    obj.maneuverState = ...
                        VehicleStates.laneChanging;
                elseif ~obj.hasLaneChangeIntention()
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            else % lane changing
                if obj.isLaneChangeComplete()
                    obj.originLane = obj.destinationLane;
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            end

            if oldState ~= obj.maneuverState
                fprintf("t=%.1f: Transition from %s to %s\n", ...
                    obj.currentTime, oldState, obj.maneuverState);
            end

        end

        function [bool] = isLaneChangeComplete(obj)
            bool = obj.currentTime >= (obj.laneChangeStartTime ...
                    + obj.laneChangeDesiredDuration) ...
                    && obj.computeLateralErrorToDestLane() < 0.1...
                    && obj.orientation < 0.01 ...
                    && obj.beta(obj.iterCounter) < 0.01;
        end

        function [bool] = isOnSameLane(obj, otherVehicle)
            % relativeY = otherVehicles(k).lateralPosition ...
            %         - obj.lateralPosition;
            % bool = abs(relativeY(k)) < (obj.laneWidth/2);
            bool = obj.currentLane == otherVehicle.currentLane;
        end

        function [bool] = isOtherAhead(obj, otherVehicle)
            bool = otherVehicle.position > obj.position;
        end

        function [bool] = isOtherBehind(obj, otherVehicle)
            bool = otherVehicle.position < obj.position;
        end

    end
end

