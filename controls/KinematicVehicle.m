classdef KinematicVehicle < Vehicle
    %KinematicVehicle Class used to model emergency braking in a connected
    %environment
    
    properties
        initialState
        samplingTime = 0.01;
        hFilterGain = 0.3;
    end
    
    properties (SetAccess = private)
        gapThresholds
    end
    
    properties (Dependent)
        x0
        vx0
        
        x
        vx
        ax
        u
        
        maxBrake
        fullStopTime  % assumes negative jerk and then max deceleration
    end
    
    methods
        
        function obj = KinematicVehicle(name, type, vx0)
            
            if nargin > 1
                % Set variables indices for this class
                obj.statesIdx.x = 1;
                obj.statesIdx.vx = 2;
                obj.statesIdx.ax = 3;

                obj.name = name;
                obj.setVehicleParams(type);
                obj.initialState = [0, vx0];

                obj.safeHeadwayParams();
                obj.safeHeadwayParamsDuringLC();
            end
        end
        
        function [] = brakeFullStop(obj, finalTime, otherVeh)
            %brakeFullStop defines the vehicle's acceleration to full stop
            % The vehicle's name property defines if this is a leader
            % or a follower. If the instance is the ego vehicle, we need to
            % specify who's the other vehicle: its leader or its follower.
            obj.simTime = 0:obj.samplingTime:finalTime;
            loc = char(obj.name);
            switch upper(loc(1))
                case 'L'
                    obj.worstCaseBrakeLeader();
                case 'F'
                    obj.worstCaseBrakeFollower();
                otherwise % main vehicle
                    otherLoc = char(otherVeh.name);
                    switch upper(otherLoc(1))
                        case 'F' % ego vehicle is leading
                            obj.worstCaseBrakeLeader();
                        case 'L' % ego vehicle is following
                            obj.worstCaseBrakeFollower();
                        otherwise
                            error(['Surrounding vehicle not properly '
                                'provided to ego vehicle.'])
                    end
            end
        end
        
        function acceleration = worstCaseBrakeLeader(obj)
            %worstCaseBrakeLeader Create the leader's worst case braking 
            % acceleration profile
            % Switching points
            jerk = [0, inf];
            accel = [obj.accelBounds(1), obj.accelBounds(1)];
            nPhases = length(jerk);
            switchDelays = -ones(1, nPhases);
            switchDelays(end) = (0-accel(2))/jerk(2);
            
            % acceleration profile over time
            acceleration = obj.kinematicProfileFromSwitchPoints(jerk, ...
                accel, switchDelays);
%             obj.simTime = simTime;
        end
        
        function acceleration = worstCaseBrakeFollower(obj)
            %worstCaseBrakeFollower Create the followers's worst case 
            % braking acceleration profile
            % This function (and the whole class) assumes connected
            % vehicles
            
            % Switching points
            jerk = [0, obj.jerkBounds(1), 0, inf];
            accel = [obj.comfAccelBounds(2), obj.comfAccelBounds(2), ...
                obj.accelBounds(1), obj.accelBounds(1)];
            nPhases = length(jerk);
            
            switchDelays = -ones(1, nPhases);
            
            switchDelays(1) = obj.reactionTime;
            switchDelays(2) = (accel(3)-accel(2))/jerk(2);
            switchDelays(end) = (0-accel(end))/jerk(end);
            
            acceleration = obj.kinematicProfileFromSwitchPoints(jerk, ...
                accel, switchDelays);
%             obj.simTime = simTime;
        end
    
        function [at] = kinematicProfileFromSwitchPoints(obj, jerk, accel, ...
                switchDelays)
            %kinematicProfileFromSwitchPoints Given piecewise constant
            %jerk and the discontinuous points, compute whole kinematic
            %profile
            
            if any(switchDelays==0)
                jerk = jerk(switchDelays~=0);
                accel = accel(switchDelays~=0);
                switchDelays = switchDelays(switchDelays~=0);
            end
            
            nPhases = length(jerk);
            vel = -ones(1, nPhases);
            vel(1) = obj.vx0;
            if vel(1) == 0
                return;
            end
            for k = 2:nPhases
                vel(k) = vel(k-1) + accel(k-1)*switchDelays(k-1) ...
                    + jerk(k-1)/2*switchDelays(k-1)^2;
            end
            if vel(k)<0
                warning(['Case where vehicle achieves full stop early is '...
                    'not coded'])
            end
            switchDelays(end) = (0-vel(end))/accel(end);
            
            % Acceleration
            sampling = 0.01;
            brakeTime = 0:sampling:sum(switchDelays);
            at = zeros(length(brakeTime), 1);
            at(1) = accel(1);
            intervalStart = 0;
            for k = 1:nPhases
                intervalEnd = sum(switchDelays(1:k));
                timeIdx = brakeTime>(intervalStart+sampling/2) ...
                    & brakeTime<=(intervalEnd+sampling/2);
                timeInterval = brakeTime(timeIdx) - intervalStart;
                intervalStart = intervalEnd;
                
                if ~isempty(timeInterval)
                    at(timeIdx) = accel(k) + jerk(k)*timeInterval;
                end
            end
            
            % Speed
            vt = obj.vx0 + cumtrapz(brakeTime, at);
            % Due to rounding errors and time discretization, vt might get
            % negative
            negativeSpeedIdx = find(vt<0, 1);
            if ~isempty(negativeSpeedIdx)
                at(negativeSpeedIdx:end) = 0;
                vt(negativeSpeedIdx:end) = 0;
            end
            
            % Position
            xt = obj.x0 + cumtrapz(brakeTime, vt);
            
            % Save to the object's state
            obj.states = zeros(length(obj.simTime), obj.nStates);
            obj.states(1:length(brakeTime), obj.statesIdx.x) = xt;
            obj.states(length(brakeTime)+1:end, obj.statesIdx.x) = ...
                obj.states(length(brakeTime), obj.statesIdx.x);
            obj.states(1:length(brakeTime), obj.statesIdx.vx) = vt;
            obj.states(1:length(brakeTime), obj.statesIdx.ax) = at;
            
        end
        
        function [minSafeGap, worstCollisionTime] = ...
                computeCollisionFreeGap(obj, leader)
            %computeCollisionFreeGap Computes minimum following distance 
            %that guarantees no collision numerically
            %Solution is found by directly solving the integral for all t 
            %and then looking for the maximum delta S.
            
            if nargin<2
                leader = obj.leader;
            end
            
            if isempty(leader.states)
                error([leader.name ' movement not yet computed'])
            elseif isempty(obj.states)
                error([obj.name ' movement not yet computed'])
            end
            
            leaderDeltaX = leader.x - leader.x0;
            egoDeltaX = obj.x - obj.x0;        
            deltaGap = leaderDeltaX - egoDeltaX;
            [minSafeGap, worstCollisionTimeIdx] = max(-deltaGap);
            
            minSafeGap = max(0, minSafeGap);
            if minSafeGap == 0
                worstCollisionTime = 0;
            else
                worstCollisionTime = obj.simTime(worstCollisionTimeIdx);
            end
        end
        
        function [minSafeGap] = ...
                computeAnalyticalCollisionFreeGap(obj, leader)
            %computeAnalyticalCollisionFreeGap Computes minimum following 
            %distance that guarantees no collision analytically
            if nargin<2
                leader = obj.leader;
            end
            risk = 0;
            minSafeGap = obj.computeSimplifiedRiskyGap(risk, leader);
%             if nargin<2
%                 leader = obj.leader;
%             end
% 
%             gamma = leader.maxBrake/obj.maxBrake;
%             threshold = leader.vx0/(obj.vx0 + obj.lambda1);
%             
%             if gamma >= threshold
%                 minSafeGap = (obj.vx0 + obj.lambda1)^2/2/obj.maxBrake ...
%                     - leader.vx0^2/2/leader.maxBrake ...
%                     + obj.lambda2;
%             elseif gamma < threshold && threshold <= 1
%                 deltaV = obj.vx0 - leader.vx0;
%                 deltaMaxBrake = obj.maxBrake - leader.maxBrake;
%                 minSafeGap = (deltaV + obj.lambda1)^2/2/deltaMaxBrake ...
%                     + obj.lambda2;
%             else
%                 minSafeGap = 0;
%             end
            
        end
        
        function[gap] = computeSimplifiedRiskyGap(obj, risk, leader)
            %computeSimplifiedRiskyGap Computes the gap that would lead 
            % to a collision with severity equal to "risk" were the 
            % worst-case scenario to happen
            
            % The computation is simplified because we don't consider the
            % cases where decreasing gaps decrease severity (when vehicles
            % are close and collide during leader braking phase)
            
            if nargin<3
                leader = obj.leader;
            end

            gamma = leader.maxBrake/obj.maxBrake;
            threshold = leader.vx0/(obj.vx0 + obj.lambda1);
            
            if gamma >= threshold
                gap = (obj.vx0 + obj.lambda1)^2/2/obj.maxBrake ...
                    - leader.vx0^2/2/leader.maxBrake ...
                    + obj.lambda2 - risk^2/2/obj.maxBrake;
            elseif gamma < threshold && threshold <= 1
                deltaV = obj.vx0 - leader.vx0;
                deltaMaxBrake = obj.maxBrake - leader.maxBrake;
                gap = (deltaV + obj.lambda1)^2/2/deltaMaxBrake ...
                    + obj.lambda2 - risk^2/2/deltaMaxBrake;
            else
                gap = 0;
            end
        end
        
        function [severity, collisionTime] = computeSeverity(obj, ...
                g0Array, leader)
            %computeSeverity Numerically computes severity of collision,
            %that is, the deltaV at collision, for the entire x0array
            
            if nargin < 3
                leader = obj.leader;
            end
            
            if isempty(leader.states)
                error([leader.name ' movement not yet computed'])
            elseif isempty(obj.states)
                error([obj.name ' movement not yet computed'])
            end
            
            nX0 = length(g0Array);
            leaderDeltaX = leader.x - leader.x0;
            egoDeltaX = obj.x - obj.x0;
            deltaGap = leaderDeltaX - egoDeltaX;
            severity = zeros(nX0, 1);
            collisionTimeIdx = zeros(nX0, 1);
            
            for n = 1:nX0
                gap = g0Array(n) + deltaGap;
                possibleTimeIdx = find(gap<0, 1);
                if ~isempty(possibleTimeIdx)
                    collisionTimeIdx(n) = possibleTimeIdx;
                    % we compute the highest severity, which can be for the
                    % follower or the leader
                    weight = max(leader.m, obj.m)/(obj.m + leader.m);
                    severity(n) = weight*(obj.vx(collisionTimeIdx(n)) - ...
                        leader.vx(collisionTimeIdx(n)));
                else
                    collisionTimeIdx(n) = length(gap);
                    severity(n) = 0;
                end
            end
            collisionTime = obj.simTime(collisionTimeIdx);
        end
        
        function [severity, x0Array] = computeSeverityProfile(obj, leader)
            %computeSeverityProfile Computes the severity of collision
            %under worst case scenario for all possible unsafe gaps
            
            if nargin < 2
                leader = obj.leader;
            end
            
            minSafeGap = obj.computeCollisionFreeGap(leader);
            spaceSampling = 0.005;
            x0Array = 0:spaceSampling:(minSafeGap*1.2);
            
            severity = obj.computeSeverity(x0Array, leader);
        end
        
        function [severity] = computeAnalyticalSeverity(obj, gap0)
            %computeSeverity Analytically computes severity of collision,
            %that is, the deltaV at collision, for the entire x0array
            leader = obj.leader;
            deltaV0 = leader.vx0 - obj.vx0;
            accel = max(obj.comfAccelBounds);
            gamma = leader.maxBrake/obj.maxBrake;
                            
            if gap0 < obj.gapThresholds(1)
                a = (leader.maxBrake + accel)/2;
                b = -deltaV0;
                c = -gap0;
%                 collisionTime = (-b + sqrt(b^2 -4*a*c))/2/a;
%                 deltaVCollision = deltaV0 ...
%                     - (leader.maxBrake + accel)*collisionTime;
                deltaVCollision = -sqrt(b^2 -4*a*c);
            elseif gap0 < obj.gapThresholds(2)
                tauD = obj.reactionTime;
                minJerk = abs(min(obj.jerkBounds));
                tauJ = (accel + obj.maxBrake)/minJerk;
                % Coefficients of third degree equation
                a = 1;
                b = -3*((accel + leader.maxBrake)/minJerk + tauD);
                c = 6*deltaV0/minJerk + 3*tauD^2;
                d = 6*gap0/minJerk - tauD^3;
                
                collisionTimeCandidates = roots([a, b, c, d]);
                possibleIdx = imag(collisionTimeCandidates)==0 ...
                    & collisionTimeCandidates > tauD ...
                    & collisionTimeCandidates < tauD+tauJ;
                collisionTime = collisionTimeCandidates(possibleIdx);
                if length(collisionTime) ~= 1
                    disp('not enough or too many collision time candidates');
                    [~, collisionTime] = obj.computeSeverity(gap0);
                end
                deltaVCollision = deltaV0 ...
                    - (leader.maxBrake + accel)*collisionTime ...
                    + minJerk/2*(collisionTime - tauD)^2;

            elseif (gamma > leader.vx0/(obj.vx0 + obj.lambda1) ...
                    && gap0 < obj.gapThresholds(3)) ...
                    || (gamma < leader.vx0/(obj.vx0 + obj.lambda1) ...
                    && gap0 < obj.gapThresholds(4))
                % Coefficients of second degree equation
                a = (leader.maxBrake - obj.maxBrake)/2;
                b = obj.lambda1 - deltaV0;
                c = obj.lambda2 - gap0;
%                 if a > 0
%                     collisionTime = (-b + sqrt(b^2-4*a*c))/2/a;
%                 elseif b > 0
%                     if leaderStopTime+tauD+tauJ > -b/a
%                         collisionTime = (-b - sqrt(b^2-4*a*c))/2/a;
%                     else
%                         collisionTime = (-b + sqrt(b^2-4*a*c))/2/a;
%                     end
%                 else
%                     disp('case a<0 and b<0');
%                 end
%                 
%                 deltaVCollision = deltaV0 ...
%                     - (leader.maxBrake - obj.maxBrake)*collisionTime ...
%                     + minJerk/2*tauJ^2 ...
%                     - (accel + obj.maxBrake)*(tauD + tauJ);
                deltaVCollision = -sqrt(b^2-4*a*c);
            elseif gap0 < obj.gapThresholds(4)
                % Coefficients of second degree equation
                a = obj.maxBrake/2;
                b = - (obj.lambda1 + obj.vx0);
                c = gap0 + leader.vx0^2/2/leader.maxBrake - obj.lambda2;
                
%                 collisionTime = (-b - sqrt(b^2-4*a*c))/2/a;
%                 deltaVCollision = deltaV0 ...
%                     + obj.maxBrake*collisionTime ...
%                     + minJerk/2*tauJ^2 ...
%                     - (accel + obj.maxBrake)*(tauD + tauJ) ...
%                     - leader.maxBrake*leaderStopTime;
                deltaVCollision = -sqrt(b^2-4*a*c);
            else
                deltaVCollision = 0; % no collision
            end
            
            weight = max(leader.m, obj.m)/(obj.m + leader.m);
            severity = -deltaVCollision*weight;
        end
        
        function [severity, x0Array] = ...
                computeAnalyticalSeverityProfile(obj, leader)
            %computeSeverityProfile Analytically computes the severity of 
            %collision under worst case scenario for all possible unsafe 
            %gaps
            
            if nargin < 2
                leader = obj.leader;
            end
            
            if isempty(obj.gapThresholds)
                obj.setGapThresholds(leader);
            end
            
            minSafeGap = obj.computeAnalyticalCollisionFreeGap(leader);
            spaceSampling = 0.01;
            x0Array = [0:spaceSampling:minSafeGap minSafeGap];
            severity = zeros(length(x0Array), 1);
            for n = 1:length(x0Array)
                severity(n) = obj.computeAnalyticalSeverity(x0Array(n));
            end
            if isempty(x0Array)
                x0Array = 0;
                severity = 0;
            end
        end
        
        function [severity, gap0Array] = estimateSeverity(obj, rho, leader)
            if nargin < 3
                leader = obj.leader;
            end
            
            gamma = leader.maxBrake/obj.maxBrake;
            Vf = obj.desiredVelocity;
            gammaThreshold = (1 - rho)*Vf/(Vf + obj.lambda1);
            massRatio = max(obj.m, leader.m)/(obj.m + leader.m);
            
            obj.computeTimeHeadway(obj.desiredVelocity, rho);
            gap0Array = [0:0.01:obj.minVFGap0, obj.minVFGap0];
            
            if gamma < gammaThreshold
                alpha1 = rho^2*Vf + 2*rho*obj.lambda1;
                alpha2 = -2*((1 - gamma)*obj.maxBrake);
            else
                alpha1 = (gamma - (1-rho)^2)*Vf/gamma ...
                    + 2*obj.lambda1;
                alpha2 = -2*obj.maxBrake;
            end
            
            squaredDeltaV = alpha1*obj.vx0 + obj.lambda1^2 ...
                + alpha2*(gap0Array - obj.lambda2);
            squaredDeltaV(squaredDeltaV < 0) = 0;
            severity = sqrt(squaredDeltaV)*massRatio;
        end
        
        %%% Setters %%%
        function [] = set.maxBrake(obj, maxBrake)
            obj.accelBounds(1) = -abs(maxBrake);
            obj.safeHeadwayParams();
            if ~isempty(obj.h)
                % Instead of recomputing time headway automatically, which
                % has its issues, we force the user to explicitly
                % recomputed it.
                obj.h = [];
                obj.d0 = [];
%                 warning(['TODO: this block does not work as it should.'...
%                     ' When recomputing the time headway, we have to use '...
%                     'the same rho as before'])
%                 obj.computeTimeHeadway(obj.desiredVelocity);
            end
        end
        
        function [] = setLeaderAndTimeHeadway(obj, leader, rho)
            if nargin < 3
                rho = 0;
            end
            obj.leader = leader;
            if isempty(obj.desiredVelocity)
                freeFlowVelocity = obj.vx0;
            else
                freeFlowVelocity = obj.desiredVelocity;
            end
            obj.computeTimeHeadway(freeFlowVelocity, rho);
            obj.setGapThresholds();
        end
        
        %%% Getters %%%
        function value = get.x0(obj)
            value = obj.initialState(obj.statesIdx.x);
        end
               
        function value = get.vx0(obj)
            value = obj.initialState(obj.statesIdx.vx);
        end
        
        function value = get.u(obj)
            warning(['Kinematic Vehicle class has no inputs. ...'
                'Returning acceleration values instead']);
            value = obj.states(:, obj.statesIdx.ax);
        end
        
        function value = get.x(obj)
            value = obj.states(:, obj.statesIdx.x);
        end
        
        function value = get.vx(obj)
            value = obj.states(:, obj.statesIdx.vx);
        end
        
        function value = get.ax(obj)
            value = obj.states(:, obj.statesIdx.ax);
        end
        
        function value = get.maxBrake(obj)
            value = abs(min(obj.accelBounds));
        end
        
        function value = get.fullStopTime(obj)
            value = (obj.vx0 + obj.lambda1)/obj.maxBrake;
        end
                
    end
    
    methods (Access = private)
        function [] = setGapThresholds(obj, leader)
            
            if nargin < 2
                leader = obj.leader;
            end
            deltaV0 = leader.vx0 - obj.vx0;
            tauD = obj.reactionTime;
            accel = max(obj.comfAccelBounds);
            minJerk = abs(min(obj.jerkBounds));
            tauJ = (accel + obj.maxBrake)/minJerk;
            leaderStopTime = leader.vx0/leader.maxBrake;
            gamma = leader.maxBrake/obj.maxBrake;
            
            obj.gapThresholds(1) = tauD*...
                (tauD/2*(accel + leader.maxBrake) - deltaV0);
            obj.gapThresholds(2) = (tauD + tauJ)*...
                (obj.lambda1 ...
                + (tauD + tauJ)/2*(leader.maxBrake - obj.maxBrake) ...
                - deltaV0) ...
                + obj.lambda2;
            if gamma < leader.vx0/(obj.vx0 + obj.lambda1)
                % Threshold 3 shoud be ignored in this case
                obj.gapThresholds(3) = - 1; 
            else
                obj.gapThresholds(3) = leaderStopTime*...
                (obj.lambda1 ...
                + leaderStopTime/2*(leader.maxBrake - obj.maxBrake) ...
                - deltaV0) ...
                + obj.lambda2;
            end
            obj.gapThresholds(4) = obj.computeAnalyticalCollisionFreeGap();
        end
    end
end

