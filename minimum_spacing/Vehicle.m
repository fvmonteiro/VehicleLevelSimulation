classdef Vehicle < handle
    %Vehicle Class to hold vehicle parameters in MSS for lane changes
    % Note: we might want to test several scenarios at once, so v0 can be a
    % vector - the class methods should be able to handle that.
    
    properties
        % Initial states
        x0 % [m] initial longitudinal position
        y0 % [m] initial lateral position
        v0 % [m/s] initial longitudinal velocity
        
        % Vehicle following parameters - by default, they are determined by
        % the vehicle parameters, but can be set to user defined values
        h
        d0
        % increased h and d0 during lane change
        hLc 
        d0Lc
        
        % Accel bounds
        maxAccel % [m/s^2] maximum longitudinal acceleration
        maxBrake % [m/s^2] maximum longitudinal deceleration (abs value)
        maxJerk % [m/s^3] maximum jerk (abs value, equal in both direction)
        %%% TODO: decide if comfortable values are the same for all
        %%% vehicles types or if they vary too
        comfAccel % [m/s^2] comfortable longitudinal acceleration
        comfBrake % [m/s^2] comfortable longitudinal deceleration (abs value)
        comfJerk % [m/s^3] comfortable jerk (abs value, equal in both direction)
        maxAccelLC
        maxBrakeLC
        
        % Other
        reactionTime % [s] time taken to notice leader started emergency braking 
        location % location with respect to ego vehicle. Up to two
        %characters: first must be L or F, second (optional) must be d or o
    end
    properties (SetAccess = protected)
        % Parameters
        type % P (passenger vehicle), B (bus) or T (truck)
        mass % [kg]         
        len % [m]
        width % [m]
        
        positionIdx = 1;
        velocityIdx = 2;
                
        % Dynamical model and states
        nStates = 2;
        A % state transition matrix
        B % input matrix
        currentState
%         xt % [m/s] longitudinal position during whole simulation
%         vt % [m/s^2] longitudinal velocity during whole simulation
        at % [m/s^2] longitudinal acceleration during whole simulation
        aLat % [m/s^2] lateral acceleration during whole simulation
    end
    
    properties (Dependent)
        minACCGap
        position
        velocity
    end
    
    methods
        function obj = Vehicle(type, location, x0, y0, v0, reactionTime)
            %Vehicle Construct an instance of this class           
            
            % Setting parameters
            g = 9.8;
            obj.type = type;
            switch obj.type
                case 'P' % passenger vehicle
                    obj.mass = 1500;
                    obj.len = 5; % [m]
                    obj.width = 1.8; 
                    obj.maxAccel = 4; % [m/s^2]
                    obj.maxBrake = 8; %0.8*g;% [m/s^2]
                    obj.maxJerk = 50; % [m/s^3]
                case 'B' % bus
                    obj.mass = 13000;
                    obj.len = 13; % [m]
                    obj.width = 2.4;
                    obj.maxAccel = 2; % [m/s^2]
                    obj.maxBrake = 0.4*g; % [m/s^2]
                    obj.maxJerk = 40; % [m/s^3]
                case 'T' % truck
                    obj.mass = 18000;
                    obj.len = 18; % [m]
                    obj.width = 2.4;
                    obj.maxAccel = 2; % [m/s^2]
                    obj.maxBrake = 3; %0.3*g; % [m/s^2]
                    obj.maxJerk = 30; % [m/s^3]
                otherwise
                    error('Unknown vehicle type')
            end
            obj.location = location;
            obj.x0 = x0;
            obj.y0 = y0;
            obj.v0 = v0;
            obj.comfAccel = 1.5; % obj.maxAccel/2;
            obj.comfBrake = 1.5; % obj.maxBrake/2;
            obj.comfJerk = 15; % obj.maxJerk/2;
            obj.maxAccelLC = obj.maxAccel;
            obj.maxBrakeLC = obj.maxBrake/3;
            obj.reactionTime = reactionTime;
            
            [obj.h, obj.d0] = obj.safeHeadwayParams();
            duringLaneChange = true;
            [obj.hLc, obj.d0Lc] = obj.safeHeadwayParams(duringLaneChange);
%             obj.currentState(obj.positionIdx) = obj.x0;
%             obj.currentState(obj.velocityIdx) = obj.v0;
            
            % Creating system matrices
            obj.A = zeros(obj.nStates);
            obj.A(1:end-1, 2:end) = eye(obj.nStates-1);
            obj.B = zeros(obj.nStates, 1);
            obj.B(end) = 1;
        end
                
        function [q] = singleStepUpdate(obj, accel, sampling, q0)
            %singleStepUpdate computes system states after one step given
            %input accel; the optional parameter x0 can be used for testing
            %purposes
            if nargin<4
                q0 = obj.currentState;
            end
            q = q0 + (obj.A*q0 + obj.B*accel)*sampling;
            obj.currentState = q;
        end
        
        function v = computeVel(obj, simTime)
            deltaT = simTime(2)-simTime(1);
            v = zeros(length(simTime), length(obj.v0));
            
            q = zeros(obj.nStates, 1);
            q(obj.positionIdx) = obj.x0;
            for n = 1:length(obj.v0)
                q(obj.velocityIdx) = obj.v0(n);
                for k = 1:length(simTime)
                    q = obj.singleStepUpdate(obj.at(k, n), deltaT, q);
                    v(k, n) = max(0, q(obj.velocityIdx));
                end
            end
                      
%             v = obj.v0 + cumtrapz(simTime, obj.at);

            % numerical integration may introduce small errors that lead to
            % final non-zero speeds at situations where this is necessary.
            % We deal with it in a not very elegant way...
            for n = 1:length(obj.v0)
                if round(v(end, n))==0
                    v(v(:, n)==v(end,n),n) = 0;
                end
            end
        end
        
        function vLat = computeVelLat(obj, simTime)
            vLat = cumtrapz(simTime, obj.aLat);
        end
        
        function [] = zeroAccel(obj, simTime)
            obj.at = zeros(length(simTime), length(obj.v0));
        end
        
        function [] = delayAccel(obj, simTime, t0)
            %delayAccel Shifts the acceleration start time to t0 (it's zero
            %by default)
            t0Idx = find(simTime==t0, 1);
            if (obj.at(end-t0Idx)~=0)
                warning('Acceleration time shift overflows - final acceleration is no longer zero')
            end
            newAt = [zeros(t0Idx, 1); obj.at(1:end-t0Idx)];
            obj.at = newAt;
        end
        
        function [accel] = singleStepACC(obj, controlParams, leader)
            q0 = [obj.position; obj.velocity];
            Ks = controlParams(1);
            Kv = controlParams(2);
            
            if ~isempty(leader)
                leaderPosition = leader.position;
                leaderVelocity= leader.velocity;
                positionRef = leaderPosition-obj.minACCGap-obj.len;
                velocityRef = leaderVelocity;
%                 ref = [leaderPosition; leaderVelocity] - [obj.len; 0];
            else
                positionRef = q0(1);
                velocityRef = obj.v0; %obj.desiredVelocity;
            end
            
            % If we receive Kacc
%           accel = controlParams*(ref-x0-[obj.g0;0]) - controlParams(1)*obj.tg*ref(2);
%           If we receive Ks, Kv
            accel = Ks*(positionRef - q0(1))+ Kv*(velocityRef-q0(2));
            accel = obj.saturateAccel(accel);
        end
                
        function [] = brakeFullStop(obj, simTime, otherVeh, duringLaneChange)
            %brakeFullStop defines the vehicle's acceleration to full stop
            % The vehicle's location property defines if this is a leader
            % (which brakes at max brake instantaneously) or this is a
            % follower (which has a delay and a negative jerk period). If
            % the instance is the ego vehicle, we need to specify who's
            % the other vehicle: its leader or its follower.
            loc = char(obj.location);
            switch upper(loc(1))
                case 'L'
                    obj.leaderBrakeFullStop(simTime);
                case 'F'
                    obj.followerBrakeFullStop(simTime);
                otherwise % main vehicle
                    otherLoc = char(otherVeh.location);
                    switch upper(otherLoc(1))
                        case 'F'
                            obj.leaderBrakeFullStop(simTime);
                        case 'L'
                            if nargin<4
                                duringLaneChange = false;
                            end
                            obj.followerBrakeFullStop(simTime, duringLaneChange);
                        otherwise
                            error('Surrounding vehicle not properly provided to ego vehicle.')
                    end
            end
        end
        
        function [] = leaderBrakeFullStop(obj, simTime)
            nV0 = length(obj.v0);
            tToFullStop = obj.v0/obj.maxBrake;
            obj.at = zeros(length(simTime), nV0);
            for n = 1:nV0
                obj.at(simTime<=tToFullStop(n), n) = -obj.maxBrake;
            end
            
        end
        
        function [] = followerBrakeFullStop(obj, simTime, duringLaneChange)
            %followerAccelProfile Create the follower's acceleration profile
            
            deltaT = simTime(2)-simTime(1);
            timeMargin = deltaT/2;
            
            delay = obj.reactionTime;
            
            actualMaxBrake = obj.maxBrake;
            actualMaxAccel = obj.maxAccel;
            if nargin>2 && duringLaneChange
                actualMaxBrake = obj.maxBrakeLC;
                actualMaxAccel = obj.maxAccelLC;
            end
            
            obj.at = zeros(length(simTime), length(obj.v0));
            for n = 1:length(obj.v0)
                % 1st interval (accelerating)
                obj.at(simTime<=delay, n) = actualMaxAccel;
                
                % 2nd interval (negative jerk towards max decel)
                tToMaxBrake = delay +  ...
                    (-actualMaxBrake - actualMaxAccel)/(-obj.maxJerk) + timeMargin; % [s] time to reach max brake
                timeInterval = simTime>delay & simTime<=tToMaxBrake;
                obj.at(timeInterval, n) = ...
                    actualMaxAccel - obj.maxJerk*(simTime(timeInterval)-delay);
                
                % 3rd interval
                vAtFullBrake = obj.v0(n) + actualMaxAccel*(tToMaxBrake) ...
                    - 1/2*obj.maxJerk*(tToMaxBrake-delay)^2; % vel at beginning of interval
                tToFullStop = tToMaxBrake + vAtFullBrake/actualMaxBrake + timeMargin;
                
                obj.at(simTime>tToMaxBrake & simTime<=tToFullStop, n) = -actualMaxBrake;
            end
            
        end
        
        function [] = sinLatAccel(obj, simTime, tAdj, tLat, h)
            %sinLatAccel Creates sinusoidal profile for lateral acceleration
            % t: simulation time
            % tAdj: maneuver start time (longitudinal adjustment time) [s]
            % tLat: lane change duration [s]
            % h: total lateral displacement [m]
            
            obj.aLat = zeros(length(simTime), length(tAdj));
            for n = 1:length(tAdj)
                nonZeroIndices = simTime>=tAdj(n) & simTime<=(tLat+tAdj(n));
                obj.aLat(nonZeroIndices, n) = 2*pi*h/(tLat^2) * sin(2*pi/tLat * (simTime(nonZeroIndices)-tAdj(n)));
            end
            
        end
        
        function [h, d0] = safeHeadwayParams(obj, duringLaneChange)
            %safeHeadwayParams computes the minimum safe following distance considering the
            %case where the leader brakes with maximum force and main vehicle is
            %travelling with maximum acceleration
            % Code expects: obj.maxBrake>0 and obj.maxJerk>0
            
            accel = obj.maxAccel;
            brake = obj.maxBrake;
            
            if nargin>1
                if duringLaneChange==true
                    brake = obj.maxBrakeLC;
                    accel = obj.maxAccelLC;
                end
            end
            
            delay = obj.reactionTime;
            t1 = (accel+brake)/obj.maxJerk;
            h = delay + t1 + ...
                1/brake*(accel*delay + accel*t1-1/2*obj.maxJerk*t1^2);
            
            d0 = 1/2*accel*delay^2 + accel*delay*t1 + 1/2*accel*t1^2 - 1/6*obj.maxJerk*t1^3 + ...
                1/(2*brake)*(accel*delay + accel*t1 - 1/2*obj.maxJerk*t1^2)^2;
            
        end
        
        function [minSafeGap] = computeFutureFollowingMSS(obj, simTime, other, futureV, duringLaneChange)
            tempV0 = obj.v0;
            obj.v0 = futureV;
            minSafeGap = obj.computeFollowingMSS(simTime, other, duringLaneChange);
            obj.v0 = tempV0;
        end
        
        function [minSafeGap] = computeFollowingMSS(obj, simTime, other, duringLaneChange)
            %computeFollowingMSS Computes minimum following distance similar to Kanaris
            %work.
            %   Solution is found by directly solving the integral for all t and then
            %   looking for the maximum delta S.
            
            % Default param value
            if nargin<4
                duringLaneChange = 0;
            end
            
            % Save accel profile before simulating full brake
            tempMyAccel = obj.at;
            tempOtherAccel = other.at;
            
            % Simulated acceleration profiles
            obj.brakeFullStop(simTime, other, duringLaneChange);
            other.brakeFullStop(simTime);
                       
            objIsLeader = strncmpi(other.location, 'F', 1);
            
            % Minimum safety spacing
            [nVel, nVelIdx] = max([length(obj.v0), length(other.v0)]);
            minSafeGap = zeros(nVel, 1);
            
            v = obj.computeVel(simTime);
            vOther = other.computeVel(simTime);
            for n = 1:nVel
                % Too lazy coding?
                if nVelIdx == 1
                    objVelIdx = n;
                    otherVelIdx = 1;
                else
                    objVelIdx = 1;
                    otherVelIdx = n;
                end
                
                if objIsLeader
                    deltaV = v(:, objVelIdx) - vOther(:, otherVelIdx);
                    lowerSafeBound = other.h*other.v0(otherVelIdx) + other.d0;
                else
                    deltaV = vOther(:, otherVelIdx) - v(:, objVelIdx);
                    lowerSafeBound = obj.hLc*obj.v0(objVelIdx) + obj.d0Lc;
                end
%                 deltaV = (-1)^objIsLeader*vM(:, n) - (-1)^objIsLeader*vOther;
                deltaGap = cumtrapz(simTime, deltaV);
                
                minSafeGap(n, :) = max(max(-deltaGap), lowerSafeBound);
%                 minSafeGap(n, :) = max(-deltaGap);
            end
            
            % We return the actual acceleration to the vehicles
            obj.at = tempMyAccel;
            other.at = tempOtherAccel;
            
        end
        
        function [minSafeGap, s2] = futureLongCollisionSeverity(obj, simTime, deltaS, other, futureV)
            tempV0 = obj.v0;
            obj.v0 = futureV;
            [minSafeGap, s2] = longCollisionSeverity(obj, simTime, deltaS, other);
            obj.v0 = tempV0;
        end
        
        function [minSafeGap, s2] = longCollisionSeverity(obj, simTime, deltaS, other)
            %collisionSeverity Computes collision severity for equally spaced different
            %initial gaps
            
            % Save accel profile before simulating full brake
            tempMyAccel = obj.at;
            tempOtherAccel = other.at;
            
            % Acceleration profiles
            obj.brakeFullStop(simTime, other);
            other.brakeFullStop(simTime);
            
            otherLoc = char(other.location);
            objIsLeader = (upper(otherLoc(1))=='F')*2-1; %+1 or -1
            
            % Minimum safety spacing           
            minSafeGap = obj.computeFollowingMSS(simTime, other); %analyticalFollowingMSS(obj, other);
            % above line is not an efficient way of dealing with the problem, but it's
            % easy :)
            g0range = 0:deltaS:max(minSafeGap);
            
            vM = obj.computeVel(simTime);
            vOther = other.computeVel(simTime);
            s2 = zeros(length(g0range), length(obj.v0));
            for n = 1:length(obj.v0)
                
                deltaV = objIsLeader*vM(:, n) - objIsLeader*vOther;
                deltaGap = cumtrapz(simTime, deltaV);
%                 delta = (-1)^objIsLeader*cumtrapz(simTime, vM(:, n)) - ...
%                     (-1)^objIsLeader*cumtrapz(simTime, vOther);
                
                if max(-deltaGap)>0 %at least one g0 will result in collision
                    collisionTimeIdx = length(deltaGap)*ones(length(g0range), 1);
                    for k = 1:length(g0range)
                        tempIdx = find(g0range(k)+deltaGap<=0, 1);
                        if ~isempty(tempIdx) % there is collision
                            collisionTimeIdx(k) = tempIdx;
                        end
                    end

                    s2(:, n) = other.mass/(other.mass+obj.mass)*abs(deltaV(collisionTimeIdx)); %.^2;
                end
                
            end
            
            % We return the actual acceleration to the vehicles
            obj.at = tempMyAccel;
            other.at = tempOtherAccel;
            
        end
        
        % Simplified function which ignores jerk limits
        function [] = laneChangeLongVelAdjustment(obj, simTime, tLong, desiredVel, aComf)
            warning('Oversimplified function without jerk');
            obj.at = zeros(length(simTime), length(obj.v0));
            for n = 1:length(obj.v0)
                accel = min(aComf, (desiredVel - obj.v0(n))/tLong);
                obj.at(simTime<=tLong, n) = accel;
            end
        end
        
        function [] = longVelAdjustmentDuringLC(obj, simTime, leaderDest, tLat)
            % just to shorten code lines
            ac = obj.comfAccel;
            bc = obj.comfBrake;
            jc = obj.comfJerk;
            
            if isnumeric(leaderDest)
                leaderV0 = leaderDest;
            else
                leaderV0 = leaderDest.v0;
            end
            
            obj.at = zeros(length(simTime), length(obj.v0));
            
            for n = 1:length(obj.v0)
                relVel = leaderV0 - obj.v0(n);
                
                if relVel<=0 % ego veh is fater than future leader
                    accel = min(roots([1/jc -tLat -relVel]));
                    if accel <= bc % we can achieve the speed within tLat
                        t1 = accel/jc;
                        obj.at(simTime<=t1, n) = -jc*simTime(simTime<=t1);
                        obj.at(simTime>t1 & simTime<=tLat-t1, n) = -accel;
                        obj.at(simTime>tLat-t1 & simTime<=tLat, n) = -accel + ...
                            jc*(simTime(simTime>tLat-t1 & simTime<=tLat)-tLat+t1);
                    else
                        accel = bc;
                        t1 = accel/jc;
                        obj.at(simTime<=t1, n) = -jc*simTime(simTime<=t1);
                        obj.at(simTime>t1 & simTime<=tLat, n) = -accel;
                        obj.at(simTime>tLat & simTime<=tLat+t1, n) = -accel + ...
                            jc*(simTime(simTime>tLat & simTime<=tLat+t1)-tLat);
                    end
                    
                else % ego is slower than future leader
                    accel = min(roots([-1/jc tLat -relVel]));
                    if accel <= ac% we can achieve the speed within tLat
                        t1 = accel/jc;
                        obj.at(simTime<=t1, n) = jc*simTime(simTime<=t1);
                        obj.at(simTime>t1 & simTime<=tLat-t1, n) = accel;
                        obj.at(simTime>tLat-t1 & simTime<=tLat, n) = accel - ...
                            jc*(simTime(simTime>tLat-t1 & simTime<=tLat)-tLat+t1);
                    else
                        accel = ac;
                        t1 = accel/jc;
                        obj.at(simTime<=t1, n) = jc*simTime(simTime<=t1);
                        obj.at(simTime>t1 & simTime<=tLat, n) = accel;
                        obj.at(simTime>tLat & simTime<=tLat+t1, n) = accel - ...
                            jc*(simTime(simTime>tLat & simTime<=tLat+t1)-tLat);
                    end
                end
            end
        end
        
        %%% NOTE: this function is not well coded (it might not get used
        %%% anyway)
        function [tAdj] = longVelAdjustment(obj, simTime, leaderDest, leaderOrig, tLat, laneWidth)
            
            % just to shorten code lines
            ac = obj.comfAccel;
            bc = obj.comfBrake;
            jc = obj.comfJerk;
            
            tAdj = zeros(length(obj.v0), 1);
            obj.at = zeros(length(simTime), length(obj.v0));
            
            deltaVBounds = obj.maxComfDeltaVDuringLaneChange(tLat);
            deltaVMargin = 0;%1; %[m/s], approx 3.6km/h
                       
            for n = 1:length(obj.v0)
                relVel = leaderDest.v0 - obj.v0(n);

                if (relVel>=min(deltaVBounds)-deltaVMargin && ...
                        relVel<=max(deltaVBounds)+deltaVMargin) % no need for pre maneuver adjustment
                    
                    tAdj(n) = 0;
                    
                    if relVel<=0 % ego veh is fater than future leader
                        accel = min(roots([1/jc -tLat -relVel]));
                        if accel <= bc % we can achieve the speed within tLat
                            t1 = accel/jc;
                            obj.at(simTime<=t1, n) = -jc*simTime(simTime<=t1);
                            obj.at(simTime>t1 & simTime<=tLat-t1, n) = -accel;
                            obj.at(simTime>tLat-t1 & simTime<=tLat, n) = -accel + ...
                                jc*(simTime(simTime>tLat-t1 & simTime<=tLat)-tLat+t1);
                        else
                            accel = bc;
                            t1 = accel/jc;
                            obj.at(simTime<=t1, n) = -jc*simTime(simTime<=t1);
                            obj.at(simTime>t1 & simTime<=tLat, n) = -accel;
                            obj.at(simTime>tLat & simTime<=tLat+t1, n) = -accel + ...
                                jc*(simTime(simTime>tLat & simTime<=tLat+t1)-tLat);
                        end
                        
                    else % ego is slower than future leader
                        accel = min(roots([-1/jc tLat -relVel]));
                        if accel <= ac% we can achieve the speed within tLat
                            t1 = accel/jc;
                            obj.at(simTime<=t1, n) = jc*simTime(simTime<=t1);
                            obj.at(simTime>t1 & simTime<=tLat-t1, n) = accel;
                            obj.at(simTime>tLat-t1 & simTime<=tLat, n) = accel - ...
                                jc*(simTime(simTime>tLat-t1 & simTime<=tLat)-tLat+t1);
                        else
                            accel = ac;
                            t1 = accel/jc;
                            obj.at(simTime<=t1, n) = jc*simTime(simTime<=t1);
                            obj.at(simTime>t1 & simTime<=tLat, n) = accel;
                            obj.at(simTime>tLat & simTime<=tLat+t1, n) = accel - ...
                                jc*(simTime(simTime>tLat & simTime<=tLat+t1)-tLat);
                        end
                    end
                    
                else % necessary pre-maneuver adjustment
                    if relVel<=0
                        % brake till relVel = min(deltaVBounds)
                        totalTime = bc/jc - relVel/bc;
                        tAdj(n) = totalTime - tLat;
                        
                        accel = bc;
                        t1 = accel/jc;
                        obj.at(simTime<=t1, n) = -jc*simTime(simTime<=t1);
                        obj.at(simTime>t1 & simTime<=totalTime-t1, n) = -accel;
                        obj.at(simTime>totalTime-t1 & simTime<=totalTime, n) = -accel + ...
                            jc*(simTime(simTime>totalTime-t1 & simTime<=totalTime)-totalTime+t1);
                    else
                        % Can we just accelerate? (if Lo is already going
                        % faster than ego) NOT FINISHED
                        jerks = [-jc; 0; jc; 0; 0; -jc];
                        
                        egoVeh = obj.copyWithSingleV0(n);
                        % IMPORTANT ASSUMPTION: in this scenario ego veh
                        % speed always equals the of Lo and they start at a
                        % minimum safe distance from each other
                        leaderOrig.v0 = egoVeh.v0; 
                        gapInit = egoVeh.computeFollowingMSS(simTime, leaderOrig);
                        
                        vTadj = leaderDest.v0 - ac*tLat + 1/2*ac^2/jc;
                        egoVeh.sinLatAccel(simTime, 0, tLat, laneWidth)
                        egoVeh.at = zeros(length(simTime), 1);
                        egoVeh.at(simTime<tLat-ac/jc) = ac;
                        egoVeh.at(simTime>=tLat-ac/jc & simTime<tLat) = ac - ...
                            jc*(simTime(simTime>=tLat-ac/jc & simTime<tLat) - (tLat-ac/jc));
                        gapFinal = egoVeh.computeFutureFollowingMSS(simTime, leaderOrig, vTadj) + ...
                            computeLaneChangeMSS(egoVeh, leaderOrig, simTime, tLat);
                        desiredDeltaGap = gapFinal - gapInit;
                        
                        % Supposed to be the proper solution
                        deltaT(1) = bc/jc; %time from zero to comf brake
                        deltaT(6) = ac/jc; %time from comf accel to zero
                        deltaT(3) = deltaT(1) + deltaT(6); %time from comf brake to comf accel
                        deltaT(5) = tLat - deltaT(6);
                        fun = @(x)egoVeh.decelAndAccelTimes(jerks(1:4), deltaT([1 3]), desiredDeltaGap, vTadj, leaderOrig.v0, x);
                        [sol, ~, ~ ,~] = fsolve(fun, [1 1]);
                        deltaT([2 4]) = sol;
                        tAdj(n) = sum(deltaT(1:4));
%                         td = deltaT(1)*2+sol(1);
                        
                        initTime = 0;
                        finalTime = 0;
                        a0 = 0;
                        for k = 1:length(jerks)
                            finalTime = finalTime+deltaT(k);
                            timeIdx = simTime>=initTime ...
                                & simTime<=finalTime;
                            obj.at(timeIdx, n) = a0 + jerks(k)*(simTime(timeIdx)-initTime);
                            initTime = finalTime;
                            a0 = obj.at(find(timeIdx, 1, 'last'), n);
                        end                     
                        
                        % Solution ignoring jerk constraints
%                         alpha2 = ac*bc/2/(ac+bc);
%                         alpha1 = leaderOrig.v0 - vTadj + ac*(vTadj-egoVeh.v0)/(ac+bc);
%                         alpha0 = -desiredDeltaGap + (vTadj-egoVeh.v0)^2/2/(ac+bc);
%                         
%                         tAdjCandidates = roots([alpha2 alpha1 alpha0]);
%                         tAdj2 = tAdjCandidates(tAdjCandidates>0);
%                         td2 = (ac*tAdj2 - (vTadj-egoVeh.v0))/(ac+bc);
                        
                    end
                    
                end
            end
            
            
        end
        
        
        % Dependent variables get methods
        function value = get.minACCGap(obj)
            value = obj.d0 + obj.h*obj.velocity;
        end
        
        function value = get.position(obj)
            value = obj.currentState(obj.positionIdx);
        end
        
        function value = get.velocity(obj)
            value = obj.currentState(obj.velocityIdx);
        end
    end
    
    methods (Access = private)
        function deltaVBounds = maxComfDeltaVDuringLaneChange(obj, tLat)            
%             maxDeltaV = obj.comfAccel*tLat - obj.comfAccel^2/obj.comfJerk;
%             minDeltaV = -obj.comfBrake*tLat + obj.comfBrake^2/obj.comfJerk;
            maxDeltaV = obj.comfAccel^2/(2*obj.comfJerk) + obj.comfAccel*(tLat - 2*obj.comfAccel/obj.comfJerk);
            minDeltaV = -obj.comfBrake^2/(2*obj.comfJerk) - obj.comfAccel*(tLat - 2*obj.comfBrake/obj.comfJerk);
            deltaVBounds = [minDeltaV, maxDeltaV];
        end
        
        function [F] = decelAndAccelTimes(obj, jerks, nonZeroJerkTimes, desiredDeltaGap, vEgoTadj, v0Lo, var)
            
            deltaT(1) = nonZeroJerkTimes(1); %time from zero to comf brake
            deltaT(2) = var(1);
            deltaT(3) = nonZeroJerkTimes(2); %time from comf accel to zero
            deltaT(4) = var(2);
%             deltaT(5) = nonZeroJerkTimes(3); %time from comf brake to comf accel
            
            tAdj = sum(deltaT);
            
            deltaXLo = v0Lo*tAdj;
            
            deltaXEgo = 0;
            v = zeros(length(jerks)+1, 1);
            a = zeros(length(jerks)+1, 1);
            v(1) = obj.v0;
            a(1) = 0;
            for k = 1:length(jerks)
                deltaXEgo = deltaXEgo + v(k)*deltaT(k)+a(k)*deltaT(k)^2/2 + jerks(k)*deltaT(k)^3/6;
                v(k+1) = v(k) + a(k)*deltaT(k) + jerks(k)*deltaT(k)^2/2;
                a(k+1) = a(k) + jerks(k)*deltaT(k);
            end
            
            F(1) = v(end) - vEgoTadj;
            deltaGap = deltaXLo - deltaXEgo;
            F(2) = deltaGap - desiredDeltaGap;
            
        end
        
        function copyVeh = copyWithSingleV0(obj, v0Idx)
            copyVeh = Vehicle(obj.type, obj.location, obj.x0, obj.y0, obj.v0(v0Idx), obj.reactionTime);
        end
    end
end

