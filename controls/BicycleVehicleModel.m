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
        
    end

    properties (SetAccess = private, Dependent)
        % States and inputs over entire simulation
        y
        theta % vehicle orientation
        beta % angle between veh orientation and veh speed

        % Initial values
        y0
        theta0

        % Final value
        yf

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
        vehicleRequestingCooperation
        virtualLeader

        yRefLog
        vRefLog
        leaderIdLog
        destLaneLeaderLog
        destLaneFollowerLog
        vehicleRequestingCooperationLog
        virtualLeaderLog
    end

    properties (Constant = true, Hidden = true)
        laneWidth = 4
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

        function value = get.theta0(obj)
            value = obj.states(1, obj.statesIdx.theta);
        end

        function value = get.y0(obj)
            value = obj.states(1, obj.statesIdx.y);
        end

        function value = get.yf(obj)
            value = obj.states(end, obj.statesIdx.y);
        end

        %%%%%%%%%%%%%%%

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

        function [bool] = hasReceivedCooperationRequest(obj)
            bool = ~isempty(obj.vehicleRequestingCooperation);
        end

        function [bool] = hasVirtualLeader(obj)
            bool = ~isempty(obj.virtualLeader);
        end

        function [] = analyzeSurroundingVehicles(obj, otherVehicles)
            obj.findLeader(otherVehicles);
            obj.findDestinationLaneVehicles(otherVehicles);
            obj.findCooperationRequest(otherVehicles);

            obj.updateManeuverState();
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
                if obj.isOnSameLane(otherVeh) ...
                        && obj.isOtherAhead(otherVeh) ...
                        && relativeX < closestX
                    closestX = relativeX;
                    leaderIdx = k;
                end
            end

            if leaderIdx > 0
                obj.leader = otherVehicles(leaderIdx);
                obj.leaderIdLog(obj.iterCounter) = obj.leader.id;
            else
                obj.leader = [];
                obj.leaderIdLog(obj.iterCounter) = 0;
            end
        end

        function [] = findDestinationLaneVehicles(obj, otherVehicles)
            obj.destLaneLeader = [];
            obj.destLaneFollower = [];
            if obj.hasLaneChangeIntention()
                closestAhead = inf;
                closestBehind = -Inf;
                for k = 1:length(otherVehicles)
                    otherVeh = otherVehicles(k);
                    relativeX = otherVeh.position - obj.position;
                    if obj.destinationLane == otherVeh.currentLane
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

            if obj.hasDestLaneLeader()
                obj.destLaneLeaderLog(obj.iterCounter) = ...
                    obj.destLaneLeader.id;
                if obj.isDestLaneLeaderNew()
                    obj.controller.activateCreateGapToDestLaneLeader();
                end
            else
                obj.destLaneLeaderLog(obj.iterCounter) = 0;
            end
        end

        function [] = findCooperationRequest(obj, otherVehicles)
            if obj.hasLeader && obj.leader.isConnected
                closestX = obj.leader.position - obj.position;
            else
                closestX = inf;
            end
            coopRequestIdx = -1;
            for k = 1:length(otherVehicles)
                otherVeh = otherVehicles(k);
                relativeX = otherVeh.position - obj.position;
                if ~obj.isOnSameLane(otherVeh) ...
                        && obj.currentLane == otherVeh.destinationLane ...
                        && obj.isOtherAhead(otherVeh) ...
                        && relativeX < closestX
                    closestX = relativeX;
                    coopRequestIdx = k;
                end
            end

            if coopRequestIdx > 0
                obj.vehicleRequestingCooperation = ...
                    otherVehicles(coopRequestIdx);
                obj.vehicleRequestingCooperationLog(obj.iterCounter) = ...
                    otherVehicles(coopRequestIdx).id;
            else
                obj.vehicleRequestingCooperation = [];
                obj.vehicleRequestingCooperationLog(obj.iterCounter) = 0;
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
            if obj.iterCounter + 1 > length(obj.simTime)
                return
            end
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
                egoMaxBrake, aLeader)
            %errorToDesiredGap computes the difference between current gap 
            % and the desired (collision free) gap
            if nargin < 4
                aLeader = obj.leader;
            end
            gap = obj.computeGapToVehicle(aLeader);
            egoVel = obj.velocity;
            leaderVel = aLeader.velocity;
            minimumGap = timeHeadway*egoVel + obj.d0 ...
                + egoVel^2/2/egoMaxBrake ...
                - leaderVel^2/2/aLeader.maxBrake;
            value = gap - minimumGap;
        end

        function [value] = computeErrorToLaneKeepingGap(obj, aLeader)
            %errorToLaneKeepingGap computes the difference between current 
            % gap and the safe gap during lane keeping
            if nargin < 2
                aLeader = obj.leader;
            end
            value = obj.computeErrorToDesiredGap(obj.h, ...
                obj.maxBrake, aLeader);
        end

        function [value] = computeErrorToLaneChangeGap(obj, aLeader)
            %errorToLaneChangeGap computes the difference between current 
            % gap and the lane changing gap
            if nargin < 2
                aLeader = obj.leader;
            end
            value = obj.computeErrorToDesiredGap(obj.hLC, ...
                obj.maxBrakeLaneChanging, aLeader);
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

            % Preallocate all arrays
            n = length(obj.simTime);
            obj.inputs = zeros(n, obj.nInputs);
            obj.delta = zeros(n, 1);
            obj.states = zeros(n, obj.nStates);
            obj.yRefLog = zeros(n, 1);
            obj.vRefLog = zeros(n, 1);
            obj.leaderIdLog = zeros(n, 1);
            obj.destLaneLeaderLog = zeros(n, 1);
            obj.destLaneFollowerLog = zeros(n, 1);
            obj.virtualLeaderLog = zeros(n, 1);
            obj.vehicleRequestingCooperationLog = zeros(n, 1);

            % Initialize
            obj.states(obj.iterCounter, :) = q0;
        end

        function [] = setInitialLaneValues(obj, lane)
            obj.originLane = lane;
            obj.currentLane = lane;
            obj.destinationLane = lane;
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
                    obj.controller.activateIncreaseGapToLeader();
                    obj.maneuverState = ...
                        VehicleStates.longitudinalAdjustment;
                elseif obj.hasReceivedCooperationRequest()
                    obj.controller.activateCreateGapToIncomingVehicle();
                    obj.maneuverState = VehicleStates.cooperating;
                end
            elseif obj.maneuverState == VehicleStates.longitudinalAdjustment
                if obj.isSafeToStartLaneChange()
                    obj.computeLaneChangeTrajectoryCoefficients();
                    obj.computeLaneChangeVelocityTrajectoryCoefficients();
                    obj.laneChangeStartTime = obj.currentTime;
                    obj.maneuverState = ...
                        VehicleStates.laneChanging;
                elseif ~obj.hasLaneChangeIntention()
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            elseif obj.maneuverState == VehicleStates.laneChanging
                if obj.isLaneChangeComplete()
                    obj.originLane = obj.destinationLane;
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            elseif obj.maneuverState == VehicleStates.cooperating
                if ~obj.hasReceivedCooperationRequest()
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            else
                error("Error: Unknown vehicle state\n" + ...
                    "\tid=%d, t=%.2f", obj.id, obj.currentTime);
            end

            if oldState ~= obj.maneuverState
                fprintf("veh %d, t=%.1f: Transition from %s to %s\n", ...
                    obj.id, obj.currentTime, oldState, obj.maneuverState);
            end

        end

        function [bool] = isSafeToStartLaneChange(obj)
            
            if obj.hasLeader()
                gapErrorToLeader = obj.computeErrorToLaneChangeGap( ...
                    obj.leader);
            else
                gapErrorToLeader = 1;
            end
            if obj.hasDestLaneLeader()
                gapErrorToDestLaneLeader = obj.computeErrorToLaneChangeGap( ...
                    obj.destLaneLeader);
            else
                gapErrorToDestLaneLeader = 1;
            end
            if obj.hasDestLaneFollower()
                gapErrorToDestLaneFollower = ...
                    obj.destLaneFollower.computeErrorToLaneKeepingGap(obj);
            else
                gapErrorToDestLaneFollower = 1;
            end
            bool = gapErrorToLeader >= 0 ...
                && gapErrorToDestLaneLeader >= 0 ...
                && gapErrorToDestLaneFollower >= 0;
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

        function [bool] = isLeaderNew(obj)
            bool = obj.isOtherVehicleNew(obj.leader, obj.leaderIdLog);
        end

        function [bool] = isDestLaneLeaderNew(obj)
            bool = obj.isOtherVehicleNew(obj.destLaneLeader, ...
                obj.destLaneLeaderLog);
        end

        function [bool] = isDestLaneFollowerNew(obj)
            bool = obj.isOtherVehicleNew(obj.destLaneFollower, ...
                obj.destLaneFollowerLog);
        end

        function [bool] = isOtherVehicleNew(obj, otherVehicle, idLog)
             if ~isempty(otherVehicle) && (obj.iterCounter == 0 || ...
                    (idLog(obj.iterCounter - 1) ...
                    ~= otherVehicle.id))
                bool = true;
            else
                bool = false;
            end
        end

    end
end

