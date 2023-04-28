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
        hasLaneChangeIntention = false
        yRefLog
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
        laneChangeTrajectoryCoeffs
        laneChangeStartTime
        laneChangeDesiredDuration = 5
    end
    
    methods
        function obj = BicycleVehicleModel(name, simTime, q0, ...
                desiredVelocity, isConnected)
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

        %%% SETTERS %%%
        function [] = set.hasLaneChangeIntention(obj, value)
%             if value && ~obj.hasLaneChangeIntention
%                 obj.controller.activateLaneChangingCBF();
%             end
            obj.hasLaneChangeIntention = value;
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
            q(3) = rem(q(3), 2*pi);
            if q(4) < 0
                q(4) = 0;
            end

            % Increase counter and update states
            obj.iterCounter = obj.iterCounter + 1;
            obj.states(obj.iterCounter, :) = q;
            obj.delta(obj.iterCounter) = atan(...
                tan(betaNow) * (obj.lf + obj.lr) / obj.lr);
            obj.updateManeuverState();
        end

        function [u1, u2] = computeInput(obj, externalInput)
            % Nominal controller
            if nargin > 1
                u1 = externalInput(1);
                u2 = externalInput(2);
            elseif ~obj.hasLeader()
                u1 = 0;
                u2 = 0;
            else
                [u1, u2] = obj.controller.computeInputs();
            end
            obj.inputs(obj.iterCounter, :) = [u1, u2];
        end

        function [leaderIdx] = findLeaderBasic(obj, otherVehicles)
            %findLeader recieves an array of vehicles, determines which are
            %on the same lane, then determines which one is longitudinally 
            % closest and makes it the leader.
            laneWidth = 3.6; % possibly make this global or pass as paramter
            
            closestX = inf;
            leaderIdx = -1;
            for k = 1:size(otherVehicles)
                relativeX = otherVehicles(k).position - obj.position;
                relativeY = otherVehicles(k).lateralPosition ...
                    - obj.lateralPosition;
                if abs(relativeY(k)) < (laneWidth/2) && relativeX(k) > 0 ...
                        && relativeX(k) < closestX
                    closestX = relativeX(k);
                    leaderIdx = k;
                end
            end

            if leaderIdx > 0
                obj.leader = otherVehicles(leaderIdx);
            else
                obj.leader = [];
            end
        end

        %%% TRACKING/ REFERENCE ERRORS %%%
        function [value] = computeErrorToDesiredGap(obj, timeHeadway, egoMaxBrake)
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
            yRef = 0;
            if obj.currentTime <= (obj.laneChangeStartTime ...
                    + obj.laneChangeDesiredDuration)
                for k = 1:length(obj.laneChangeTrajectoryCoeffs)
                    yRef = yRef + obj.laneChangeTrajectoryCoeffs(k)...
                        *(obj.currentTime - obj.laneChangeStartTime)^(k-1);
                end
            else
                yRef = 4; % Must be dependent on something else
            end

            obj.yRefLog(obj.iterCounter) = yRef;

            ey = yRef - obj.lateralPosition;
        end

        function [ey] = computeLateralErrorToDestLane(obj)
            yDestLane = 4; % TODO: make param, get from somewhere
            ey = yDestLane - obj.lateralPosition;
        end

        function [yRefDot] = computeLateralReferenceDerivative(obj)
            yRefDot = 0;
            if obj.currentTime <= (obj.laneChangeStartTime ...
                    + obj.laneChangeDesiredDuration)
                for k = 2:length(obj.laneChangeTrajectoryCoeffs)
                    yRefDot = yRefDot ...
                        + (k-1) * obj.laneChangeTrajectoryCoeffs(k)...
                        *(obj.currentTime - obj.laneChangeStartTime)^(k-2);
                end
            else
                yRefDot = 0;
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
            obj.states = zeros(n, length(q0));
            obj.yRefLog = zeros(n, 1);
            
            % Initialize
            obj.states(obj.iterCounter, :) = q0;
        end

        function [] = computeLaneChangeTrajectoryCoefficients(obj)
            tlc = obj.laneChangeDesiredDuration;
            Y0 = [0, 0, 0]';
            Yf = [4, 0, 0]';  % TODO: make yd a parameter
            obj.laneChangeTrajectoryCoeffs = findPathCoefficients(...
                tlc, Y0, Yf);
        end

        function [] = updateManeuverState(obj)
            oldState = obj.maneuverState;
            if obj.maneuverState == VehicleStates.laneKeeping
                if obj.hasLaneChangeIntention
                    obj.controller.activateLaneChangingCBF();
                    obj.maneuverState = ...
                        VehicleStates.longitudinalAdjustment;
                end
            elseif obj.maneuverState == VehicleStates.longitudinalAdjustment
                if obj.computeErrorToLaneChangeGap >= 0
                    obj.computeLaneChangeTrajectoryCoefficients();
                    obj.laneChangeStartTime = obj.currentTime;
%                     obj.controller.activateLaneChangingCLFCBF();
                    obj.maneuverState = ...
                        VehicleStates.laneChanging;
                elseif ~obj.hasLaneChangeIntention
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            else % lane changing
                % TODO: condition on end of maneuver
                % obj.laneChangeIntention = false;
                % obj.maneuverState = VehicleStates.laneKeeping;
                if ~obj.hasLaneChangeIntention
                    obj.maneuverState = VehicleStates.laneKeeping;
                end
            end
    
            if oldState ~= obj.maneuverState
                fprintf("t=%.1f: Transition from %s to %s\n", ...
                    obj.currentTime, oldState, obj.maneuverState);
            end

        end

    end
end

