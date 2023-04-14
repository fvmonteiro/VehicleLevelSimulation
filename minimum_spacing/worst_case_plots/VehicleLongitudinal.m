classdef VehicleLongitudinal < handle
    %Class with vehicles parameters and methods to find safe gaps
    
    properties
        % Initial states
        x0 % [m] initial longitudinal position
        v0 % [m/s] initial longitudinal velocity
        
        % Parameters
        type % P (passenger vehicle), B (bus) or T (truck)
        name % location with respect to ego vehicle. Up to two
        %characters: first must be L or F, second (optional) must be d or o
        mass % [kg]
        len % [m]
        width % [m]
        timeToBrake % [s] time taken to notice leader started braking 
        timeToEmergencyBrake = 0 % [s] time taken to notice leader started 
        % emergency braking 
        freeFlowVel = 30; % [m/s]
        maxAccel % [m/s^2] 
        maxBrake % [m/s^2] (abs value)
        maxJerk % [m/s^3] maximum negative jerk (abs value)
        maxPosJerk = inf; % [m/s^3] used to avoid a(t) from going 
        % instantaneously to zero
        comfBrake % [m/s^2]
        comfJerk % % [m/s^3] comfortable negative jerk (abs value)
        
        % ACC params (dependent on other params, but quicker to keep a copy)
        lambda1 % lambda1/maxDecel is the time headway of the exact min gap
        % solution assuming equal decelerations and speeds
        lambda2 % constant term of the exact min gap solution assuming equal
        % decelerations and speeds
        h % time headway for ACC given difference in decelerations and 
        % assumed difference in speeds and max speed
        d0 % distance at standstill for ACC
        altH % using for tests
        altD0 % using for tests
        
    end
    properties (SetAccess = protected)
        % Parameters
        
%         positionIdx = 1;
%         velocityIdx = 2;
                
        % Dynamical model and states
        nStates = 2;
        A % state transition matrix
        B % input matrix
        currentState
        deltaXt % [m/s] longitudinal position variation during whole 
        % simulation
        vt % [m/s^2] longitudinal velocity during whole simulation
        at % [m/s^2] longitudinal acceleration during whole simulation
    end
    
    properties (Dependent)
%         minACCGap
%         position
%         velocity
    end
    
    methods
        function obj = VehicleLongitudinal(type, name, q0, delay)
            %Vehicle Construct an instance of this class           
            
            if nargin==0
                return % no parameter constructor
            end
            % Setting parameters
            g = 9.8;
            obj.type = type;
            switch obj.type
                case 'P' % passenger vehicle
                    obj.mass = 2000;
                    obj.len = 5; % [m]
                    obj.width = 1.8; 
                    obj.maxAccel = 3; % [m/s^2]
                    obj.maxBrake = 8; %0.8*g;% [m/s^2]
                    obj.maxJerk = 50; % [m/s^3]
                    obj.comfBrake = 0.5;
                    obj.comfJerk = 10;
                case 'B' % bus
                    obj.mass = 13000;
                    obj.len = 13; % [m]
                    obj.width = 2.4;
                    obj.maxAccel = 2; % [m/s^2]
                    obj.maxBrake = 0.4*g; % [m/s^2]
                    obj.maxJerk = 40; % [m/s^3]
                    obj.comfBrake = 0.5;
                    obj.comfJerk = 5;
                case 'T' % truck
                    obj.mass = 18000;
                    obj.len = 18; % [m]
                    obj.width = 2.4;
                    obj.maxAccel = 2; % [m/s^2]
                    obj.maxBrake = 3; %0.3*g; % [m/s^2]
                    obj.maxJerk = 30; % [m/s^3]
                    obj.comfBrake = 0.5;
                    obj.comfJerk = 5;
                otherwise
                    error('Unknown vehicle type')
            end
            obj.name = name;
            obj.x0 = q0(1);
            obj.v0 = q0(2);
            obj.timeToBrake = delay(1);
            if length(delay)>1
                obj.timeToEmergencyBrake = delay(2);
            end
            if (obj.timeToEmergencyBrake == 0)
                obj.comfJerk = obj.maxJerk;
                totalDelay = obj.timeToBrake;
            else
                obj.comfJerk = 10;
                totalDelay = obj.timeToBrake + ...
                    (obj.comfBrake+obj.maxAccel)/obj.comfJerk + ...
                    obj.timeToEmergencyBrake;
            end
            
%             [obj.lambda1, obj.lambda2] = obj.safeHeadwayParams();
            [obj.lambda1, obj.lambda2] = obj.computeHeadwayParams(...
                totalDelay, obj.maxAccel, obj.maxBrake, obj.maxJerk);
            
            % Creating system matrices
            obj.A = zeros(obj.nStates);
            obj.A(1:end-1, 2:end) = eye(obj.nStates-1);
            obj.B = zeros(obj.nStates, 1);
            obj.B(end) = 1;
        end
        
        function [] = zeroAccel(obj, simTime)
            obj.at = zeros(length(simTime), 1);
            obj.vt = obj.v0*ones(length(simTime), 1);
            obj.deltaXt = obj.v0*simTime';
        end
        
        function [] = brakeFullStop(obj, simTime, otherVeh)
            %brakeFullStop defines the vehicle's acceleration to full stop
            % The vehicle's location property defines if this is a leader
            % (which brakes at max brake instantaneously) or this it is a
            % follower (which has a delay and a negative jerk period). If
            % the instance is the ego vehicle, we need to specify who's
            % the other vehicle: its leader or its follower.
            loc = char(obj.name);
            switch upper(loc(1))
                case 'L'
                    obj.leaderBrakeFullStop(simTime);
                case 'F'
                    obj.followerBrakeFullStop(simTime);
                otherwise % main vehicle
                    otherLoc = char(otherVeh.name);
                    switch upper(otherLoc(1))
                        case 'F' % ego vehicle is leading
                            obj.leaderBrakeFullStop(simTime);
                        case 'L' % ego vehicle is following
                            obj.followerBrakeFullStop(simTime);
                        otherwise
                            error(['Surrounding vehicle not properly '
                                'provided to ego vehicle.'])
                    end
            end
        end
        
        function [] = leaderBrakeFullStop(obj, simTime)
            %leaderBrakeFullStop Create the leader's worst case braking 
            % acceleration profile
            
            % Switching points
            jerk = [-obj.maxJerk, 0, inf];
            accel = [0, -obj.maxBrake, -obj.maxBrake];
            nPhases = length(jerk);
            switchDelays = -ones(1, nPhases);
            switchDelays(1) = (accel(2)-accel(1))/jerk(1);
            switchDelays(end) = (0-accel(3))/jerk(3);

            % Actual profile over time
            obj.kinematicProfileFromSwitchPoints(jerk, accel, ...
                switchDelays, simTime);

        end
        
        function [] = followerBrakeFullStop(obj, simTime)
            %followerAccelProfile Create the followers's worst case 
            % braking acceleration profile
            
            % Switching points
            if (obj.timeToEmergencyBrake == 0)
                obj.comfJerk = obj.maxJerk;
            end
            jerk = [0, -obj.comfJerk, 0, -obj.maxJerk, 0, obj.maxPosJerk];
            accel = [obj.maxAccel, obj.maxAccel, -obj.comfBrake, ...
                -obj.comfBrake, -obj.maxBrake, -obj.maxBrake];
            nPhases = length(jerk);
            
            switchDelays = -ones(1, nPhases);
            
            switchDelays(1) = obj.timeToBrake;
            switchDelays(2) = (accel(3)-accel(2))/jerk(2);
            switchDelays(3) = obj.timeToEmergencyBrake;
            switchDelays(4) = (accel(5)-accel(4))/jerk(4);
            switchDelays(end) = (0-accel(end))/jerk(end);
            
            obj.kinematicProfileFromSwitchPoints(jerk, accel, ...
                switchDelays, simTime);
            
        end
        
        function [] = recomputeHeadwayParams(obj)
            if (obj.timeToBrake==0) || (obj.timeToEmergencyBrake== 0)
                totalDelay = obj.timeToBrake + obj.timeToEmergencyBrake;
            else
                totalDelay = obj.timeToBrake + ...
                    (obj.comfBrake+accel)/obj.comfJerk + ...
                    obj.timeToEmergencyBrake;
            end
            [obj.lambda1, obj.lambda2] = obj.computeHeadwayParams(...
                totalDelay, obj.maxAccel, obj.maxBrake, obj.maxJerk);
%             if ~isempty(obj.lambda1) && ~isempty(obj.lambda2)
%                 [hNew, d0New] = obj.safeHeadwayParams();
%             end
        end
        
        function [] = safeHeadwayParams(obj)
            %safeHeadwayParams computes the minimum safe following distance
            %considering the case where the leader brakes with maximum 
            %force and main vehicle is travelling with maximum acceleration
            
            % Code expects: obj.maxBrake>0 and obj.maxJerk>0
            
            accel = obj.maxAccel;
            brake = obj.maxBrake;
            jerk = obj.maxJerk;
            
            if (obj.timeToBrake==0) || (obj.timeToEmergencyBrake== 0)
                delay = obj.timeToBrake + obj.timeToEmergencyBrake;
            else
                delay = obj.timeToBrake + ...
                    (obj.comfBrake+accel)/obj.comfJerk + ...
                    obj.timeToEmergencyBrake;
            end
            
            [obj.lambda1, obj.lambda2] = obj.computeHeadwayParams(delay,...
                accel, brake, jerk);
            
%             obj.lambda1 = lambda1;
%             obj.lambda2 = lambda2;
        end
        
        function [hNew] = adjustTimeHeadway(obj, leader, maxVel, rho)
            warning(['[12/14/2020] Deprecated function name: change to '...
                'computeHeadway. Be careful with new definition of rho: '...
                'vl >= (1-rho)ve'])
            % Adjusting to new [12/14/20] notation
            if rho<=1
                rho = 1 - rho;
            else
                rho = 1 - 1/rho;
            end
            hNew = obj.computeHeadway(leader, maxVel, rho);
        end
        
        function [] = computeHeadway(obj, leader, maxVel, rho)
            %computeHeadway computes time headway based on expected
            %difference of max braking from ego and leading vehicles and on
            %expected maximum relative velocity
            % gamma =  leader.maxBrake/ego.maxBrake
            % rho = (ve-vl)/ve
            obj.safeHeadwayParams();
            if nargin<4
                print('No provided rho; computing it from vL and vE...')
                rho = max((obj.v0-leader.v0)/obj.v0, 0);
            end
            gamma = leader.maxBrake/obj.maxBrake;
            gammaThreshold = (1-rho)*maxVel/(maxVel+obj.lambda1);
            if gamma<=gammaThreshold
                obj.h = rho/obj.maxBrake/(1-gamma)*(rho*maxVel/2+obj.lambda1);
                obj.d0 = obj.lambda1^2/(2*obj.maxBrake*(1-gamma)) ...
                    + obj.lambda2;
            elseif gamma>=(1-rho)^2
                obj.h = obj.lambda1/obj.maxBrake + ...
                    (gamma-(1-rho)^2)*maxVel/(2*obj.maxBrake*gamma);
                obj.d0 = obj.lambda1^2/2/obj.maxBrake + obj.lambda2;
            else
                obj.h = obj.lambda1/obj.maxBrake;
                obj.d0 = obj.lambda1^2/2/obj.maxBrake + obj.lambda2;
            end
        end
        
        function [] = alternativeTimeHeadway(obj, simTime, leader, velFactor)
            %alternativeTimeHeadway Method computes time headway by solving
            %the collision free integral at some max vel and then find a 
            %headway to overestimate that
            
            copyVeh = VehicleLongitudinal(obj.type, [obj.name '_copy'], ...
                [0, 0], [obj.timeToBrake, obj.timeToEmergencyBrake]);
            copyVeh.maxBrake = obj.maxBrake;
            copyVeh.followerBrakeFullStop(simTime);
            obj.altD0 = max(0, copyVeh.computeFollowingMSS(leader));
            
            maxVel = leader.v0*velFactor;
            copyVeh.v0 = maxVel;
%             copyVeh = VehicleLongitudinal(obj.type, [obj.name '_copy'], ...
%                 [0, maxVel], [obj.timeToBrake, obj.timeToEmergencyBrake]);
%             copyVeh.maxBrake = obj.maxBrake;
            copyVeh.followerBrakeFullStop(simTime);
            minSafeGapMaxVel = copyVeh.computeFollowingMSS(leader);
            obj.altH = (minSafeGapMaxVel-obj.altD0)/(maxVel);
        end
        
        function minGap = alternativeVehFollGap(obj, simTime, leader, velFactor)
            obj.alternativeTimeHeadway(simTime, leader, velFactor);
            minGap = obj.altH*obj.v0 + obj.altD0;
        end
                       
        function [minSafeGap, collisionTimeIdx] = ...
                computeFollowingMSS(obj, leader)
            %computeFollowingMSS Computes minimum following distance 
            %similar to Kanaris work.
            %Solution is found by directly solving the integral for all t 
            %and then looking for the maximum delta S.
            
            if isempty(leader.at)
                error([leader.name ' movement not yet computed'])
            elseif isempty(obj.at)
                error([obj.name ' movement not yet computed'])
            end
                       
            deltaGap = leader.deltaXt - obj.deltaXt;
            [minSafeGap, collisionTimeIdx] = max(-deltaGap);
            
            minSafeGap = max(0, minSafeGap);
            
             % Just to check analytical computations
%              tOpt = (obj.v0-leader.v0 + obj.lambda1)...
%                  /(obj.maxBrake-leader.maxBrake);        
%             analyticalGap = obj.analyticalFollowingMSS(leader); %, ...
%                 obj.maxBrake, obj.lambda1, obj.lambda2);
%             if abs(minSafeGap-analyticalGap)>0.1
%                 disp(obj.v0)
%                 disp(minSafeGap-analyticalGap)
%                 warning('Simulation and analytical results don''t match')
%             end
            
        end
        
        
        function [minSafeGap, gapTstop] = analyticalFollowingMSS(obj, leader)
%                 myMaxBrake, lambda1, lambda0)
            gamma = leader.maxBrake/obj.maxBrake;
            rho = (obj.v0-leader.v0)/obj.v0;
            gammaThreshold = (1-rho)*obj.v0/(obj.v0+obj.lambda1);
            
            gapTstop = obj.v0^2/2/obj.maxBrake - ...
                    leader.v0^2/2/leader.maxBrake + ... % non linear terms
                    obj.lambda1/obj.maxBrake*obj.v0 + ... % linear term
                    obj.lambda1^2/2/obj.maxBrake + obj.lambda2; % constant
            gapTstop = max(gapTstop, 0);
            if gamma>=gammaThreshold
                minSafeGap = gapTstop;
            else
                deltaV = obj.v0 - leader.v0;
                deltaBrake = (obj.maxBrake - leader.maxBrake);
                tauJ = (obj.maxAccel+obj.maxBrake)/obj.maxJerk;
                tauD = obj.timeToBrake;
                c = -(obj.maxAccel+obj.maxBrake)*(tauD^2+tauD*tauJ+tauJ^2/3)/2;
                lambda0 = obj.lambda1^2/2/deltaBrake + c;
                minSafeGap =  deltaV^2/2/deltaBrake + ...
                    obj.lambda1/deltaBrake*deltaV + lambda0;
            end
            
        end
        
        function [vehFollGap] = vehFollGap(obj)
            %vehFollGap Computes and returns the constant time headway 
            %vehicle following gap
            
            if ~isempty(obj.h)
                vehFollGap = obj.h*obj.v0 + obj.d0;
            else
                error('Safe headway not computed')
%                 vehFollGap = obj.lambda1*obj.v0 + obj.lambda2;
            end            
        end
        
        function [severity, collisionTimeIdx] = computeSeverity(obj, ...
                leader, x0array)
            nX0 = length(x0array);
            
            deltaGap = leader.deltaXt - obj.deltaXt;
            severity = zeros(nX0, 1);
            collisionTimeIdx = zeros(nX0, 1);
            
            for n = 1:nX0
                gap = x0array(n) + deltaGap;
                possibleTimeIdx = find(gap<0, 1);
                if ~isempty(possibleTimeIdx)
                    collisionTimeIdx(n) = possibleTimeIdx;
                    % we compute the highest severity, which can be for the
                    % follower or the leader
                    weight = max(leader.mass, obj.mass)/(obj.mass+leader.mass);
                    severity(n) = weight*(obj.vt(collisionTimeIdx(n)) - ...
                        leader.vt(collisionTimeIdx(n)));
                else
                    collisionTimeIdx(n) = length(gap);
                    severity(n) = 0;
                end
            end
        end
        
        %%% Plot functions %%%
        function fig = plotKinematics(obj, simTime)
            fig = figure;
            x = [obj.at, obj.vt, obj.deltaXt];
            for k = 1:3
                subplot(3, 1, k);
                plot(simTime, x(:, k));
                grid on;
            end
        end
        
        function fig = plotState(obj, simTime, state, fig)
            if nargin<=3
                fig = figure;
                maxX = 0;
                oldylim = [0 0];
            else
                figure(fig);
                hold on;
                maxX = xlim;
                maxX = maxX(2);
                oldylim = ylim;
            end
            
            switch state
                case 'a'
                    state = obj.at;
                    stateLabel = 'a [m/s^2]';
                case 'v'
                    state = obj.vt;
                    stateLabel = 'v [m/s]';
                case 'x'
                    state = obj.deltaXt;
                    stateLabel = '\Delta x [m]';
                otherwise
                    error('unknown state');
            end
            
%             delayIdx = round((obj.timeToBrake+obj.timeToEmergencyBrake)...
%               /(simTime(2)-simTime(1)));
            stopIdx = find(obj.vt==0, 1);
            plot(simTime(1:stopIdx-1), state(1:stopIdx-1), 'LineWidth', 2);
%             plot(simTime, state, 'LineWidth', 1.5);
            xlabel('t [s]');
            ylabel(stateLabel);
            xlim([0, max(maxX, round(simTime(stopIdx)+0.5))]);
            newylim = [min(oldylim(1), min(state)-0.5), ...
                max(oldylim(2), max(state)+0.5)];
            ylim(newylim);
            grid on;
            hold off;
        end
        
        %%% Modifying parameters %%%
        function [] = setNewMaxBrake(obj, newValue)
            obj.maxBrake = newValue;
            obj.recomputeHeadwayParams();
        end
        
        function [] = setNewMaxAccel(obj, newValue)
            obj.maxAccel = newValue;
            obj.recomputeHeadwayParams();
        end
        
    end
    
    methods (Access = protected)
        
        function [lambda1, lambda2] = computeHeadwayParams(~, delay, ...
                accel, brake, jerk)
            tauJ = (accel+brake)/jerk;
            lambda1 = (accel+brake)*(delay+tauJ/2);
            lambda2 = -(accel+brake)*(delay^2+delay*tauJ+tauJ^2/3)/2;
            
            % [14/12/2020] Checking if new computations are correct
            lambda1OLD = delay + tauJ ...
                + 1/brake*(accel*delay + accel*tauJ-1/2*jerk*tauJ^2);
            if abs(lambda1/brake-lambda1OLD)>0.001
                error('New lambda1 computation is wrong.')
            end
            lambda2OLD = 1/2*accel*delay^2 + accel*delay*tauJ ...
                + 1/2*accel*tauJ^2 - 1/6*jerk*tauJ^3 + ...
                1/(2*brake)*(accel*delay + accel*tauJ - 1/2*jerk*tauJ^2)^2;
            if abs((lambda1^2/2/brake+lambda2)-lambda2OLD)>0.001
                error('New lambda2 computation is wrong.')
            end
        end
        
        function [] = kinematicProfileFromSwitchPoints(obj, jerk, accel,...
                switchDelays, simTime)
            %kinematicProfileFromSwitchPoints Given piecewise constant
            %jerk and the discontinuous points, compute whole kinematic 
            %profile
            
            maxPosJerkIndicator = isinf(obj.maxPosJerk);
            
            if ~maxPosJerkIndicator
%                 jerk = [jerk obj.maxPosJerk];
%                 accel = [accel -obj.maxBrake];
%                 switchDelays(end+1) = (0-accel(end))/jerk(end);
                error('Code is not adapted to finite positive jerk')
            end
            
            if any(switchDelays==0)
                jerk = jerk(switchDelays~=0);
                accel = accel(switchDelays~=0);
                switchDelays = switchDelays(switchDelays~=0);
            end
            
            nPhases = length(jerk);
            vel = -ones(1, nPhases);
            obj.at = zeros(length(simTime), 1);
            obj.vt = zeros(length(simTime), 1);
            obj.deltaXt = zeros(length(simTime), 1);
            
            vel(1) = obj.v0;
            if vel(1) == 0
                return;
            end
            for k = 2:nPhases
                vel(k) = vel(k-1) + accel(k-1)*switchDelays(k-1) ...
                    + jerk(k-1)/2*switchDelays(k-1)^2;
            end
            switchDelays(end) = (0-vel(end))/accel(end);
%             for k = 2:nPhases-1
%                 vel(k) = vel(k-1) + accel(k-1)*switchDelays(k-1) ...
%                   + jerk(k-1)/2*switchDelays(k-1)^2;
%             end
%             vel(end)= 0-(accel(end)*switchDelays(end) 
%               + jerk(end)/2*switchDelays(end)^2);
%             switchDelays(end-1) = (vel(end)-vel(end-1))/accel(end-1);

            
            obj.at(1) = accel(1);
            obj.vt(1) = vel(1);
            intervalStart = 0;
            lastDeltaX = 0;
            sampling = simTime(2)-simTime(1);
            for k = 1:nPhases
                intervalEnd = sum(switchDelays(1:k));
                timeIdx = simTime>(intervalStart+sampling/2) ...
                    & simTime<=(intervalEnd+sampling/2);
                timeInterval = simTime(timeIdx) - intervalStart;
                intervalStart = intervalEnd;
                
                if ~isempty(timeInterval)
                    obj.at(timeIdx) = accel(k) + jerk(k)*timeInterval;
                    obj.vt(timeIdx) = vel(k) + accel(k)*timeInterval + ...
                        1/2*jerk(k)*timeInterval.^2;

                    % Check if vehicle prematurely achieves full stop
                    stopIdx = find(obj.vt(timeIdx)<=0, 1);
                    
                    if isempty(stopIdx) || k==length(switchDelays)
                        obj.at(timeIdx) = accel(k) + jerk(k)*timeInterval;
                        obj.deltaXt(timeIdx) = lastDeltaX ...
                            + vel(k)*timeInterval ...
                            + 1/2*accel(k)*timeInterval.^2 ...
                            + 1/6*jerk(k)*timeInterval.^3;
                        lastDeltaX = obj.deltaXt(find(timeIdx, 1, 'last'));
                    else
                        timeIdxStart = find(timeIdx, 1);
                        obj.at(timeIdxStart+stopIdx-1:end) = 0;
                        obj.vt(timeIdxStart+stopIdx-1:end) = 0;
                        truncatedTimeInterval = timeInterval(1:stopIdx);
                        obj.deltaXt(timeIdx(1:timeIdxStart+stopIdx-1)) = ...
                            lastDeltaX + vel(k)*truncatedTimeInterval ...
                            + 1/2*accel(k)*truncatedTimeInterval.^2 ...
                            + 1/6*jerk(k)*truncatedTimeInterval.^3;
                        obj.deltaXt(timeIdxStart+stopIdx:end) = ...
                            obj.deltaXt(timeIdxStart+stopIdx-1);
                        return;

                    end
                end
            end
            
            obj.deltaXt(find(timeIdx, 1, 'last')+1:end) = lastDeltaX;
        end
        
    end
    
end

