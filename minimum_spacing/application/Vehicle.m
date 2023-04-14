classdef Vehicle < handle
    %Vehicle Class contains vehicles parameters and functions to compute
    %MSS
    
    properties
        % Initial states
        x0 % [m] initial longitudinal position
        v0 % [m/s] initial longitudinal velocity
        
        % Parameters
        type % P (passenger vehicle), B (bus) or T (truck)
        mass % [kg]         
        len % [m]
        width % [m]
        maxAccel % [m/s^2] maximum longitudinal acceleration
        maxBrake % [m/s^2] maximum longitudinal deceleration (abs value)
        maxJerk % [m/s^3] maximum jerk (abs value, equal in both direction)
        comfBrake
        comfJerk
        timeToBrake % [s] time taken to notice leader started braking 
        timeToEmergencyBrake % [s] time taken to notice leader started emergency braking 
        name % location with respect to ego vehicle. Up to two
        %characters: first must be L or F, second (optional) must be d or o
    end
    properties (SetAccess = protected)
        % Parameters
        
        positionIdx = 1;
        velocityIdx = 2;
                
        % Dynamical model and states
        nStates = 2;
        A % state transition matrix
        B % input matrix
        currentState
        deltaXt % [m/s] longitudinal position variation during whole simulation
        vt % [m/s^2] longitudinal velocity during whole simulation
        at % [m/s^2] longitudinal acceleration during whole simulation
    end
    
    properties (Dependent)
%         minACCGap
        position
        velocity
    end
    
    methods
        function obj = Vehicle(type, name, x0, v0, delay1, delay2)
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
            obj.name = name;
            obj.x0 = x0;
            obj.v0 = v0;
            obj.timeToBrake = delay1;
            obj.timeToEmergencyBrake = delay2;
            obj.comfBrake = 2;
            obj.comfJerk = 10;
            
            % Creating system matrices
            obj.A = zeros(obj.nStates);
            obj.A(1:end-1, 2:end) = eye(obj.nStates-1);
            obj.B = zeros(obj.nStates, 1);
            obj.B(end) = 1;
        end
                
                               
        function [] = brakeFullStop(obj, simTime)%, otherVeh)
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
                otherwise %
                    error('Unexpected vehicle name.')
            end
        end
        
        function [] = leaderBrakeFullStop(obj, simTime)
            
            % Switching points
            jerk = [-obj.maxJerk, 0];
            accel = [0, -obj.maxBrake];
            nPhases = length(jerk);
            switchDelays = -ones(1, nPhases);
            
            switchDelays(1) = (accel(2)-accel(1))/jerk(1);
            
            % Actual profile over time
            obj.kinematicProfileFromSwitchPoints(jerk, accel, switchDelays, simTime);

        end
        
        function [] = followerBrakeFullStop(obj, simTime)
            %followerAccelProfile Create the follower's acceleration profile
            
            % Switching points
            jerk = [0, -obj.comfJerk, 0, -obj.maxJerk, 0];
            accel = [obj.maxAccel, obj.maxAccel, -obj.comfBrake, -obj.comfBrake, -obj.maxBrake];
            nPhases = length(jerk);
            
            switchDelays = -ones(1, nPhases);
            
            switchDelays(1) = obj.timeToBrake;
            switchDelays(2) = (accel(3)-accel(2))/jerk(2);
            switchDelays(3) = obj.timeToEmergencyBrake;
            switchDelays(4) = (accel(5)-accel(4))/jerk(4);
            
            obj.kinematicProfileFromSwitchPoints(jerk, accel, switchDelays, simTime);
            
        end
        
        function [] = kinematicProfileFromSwitchPoints(obj, jerk, accel, switchDelays, simTime)
            %kinematicProfileFromSwitchPoints Given piecewise constant
            %jerk and the discontinuous points, compute whole kinematic 
            %profile
            
            %%%%% TODO: check at every step if vel(k) = 0
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
                vel(k) = vel(k-1) + accel(k-1)*switchDelays(k-1) + jerk(k-1)/2*switchDelays(k-1)^2;
            end
            switchDelays(end) = (0-vel(end))/accel(end);
            
            obj.at(1) = accel(1);
            obj.vt(1) = vel(1);
            intervalStart = 0;
            lastDeltaX = 0;
            sampling = simTime(2)-simTime(1);
            for k = 1:length(switchDelays)
                intervalEnd = sum(switchDelays(1:k));
                timeIdx = simTime>(intervalStart+sampling/2) & simTime<=(intervalEnd+sampling/2);
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
                        obj.deltaXt(timeIdx) = lastDeltaX + vel(k)*timeInterval + ...
                            1/2*accel(k)*timeInterval.^2 + 1/6*jerk(k)*timeInterval.^3;
                        lastDeltaX = obj.deltaXt(find(timeIdx, 1, 'last'));
                    else
                        timeIdxStart = find(timeIdx, 1);
                        obj.at(timeIdxStart+stopIdx-1:end) = 0;
                        obj.vt(timeIdxStart+stopIdx-1:end) = 0;
                        truncatedTimeInterval = timeInterval(1:stopIdx);
                        obj.deltaXt(timeIdx(1:timeIdxStart+stopIdx-1)) = lastDeltaX + vel(k)*truncatedTimeInterval + ...
                            1/2*accel(k)*truncatedTimeInterval.^2 + 1/6*jerk(k)*truncatedTimeInterval.^3;
                        obj.deltaXt(timeIdxStart+stopIdx:end) = obj.deltaXt(timeIdxStart+stopIdx-1);
                        return;
%                         lastDeltaX = obj.deltaXt(find(timeIdx, 1, 'last'));
                    end
                end
            end
            
            obj.deltaXt(find(timeIdx, 1, 'last')+1:end) = lastDeltaX;
        end       
       
        function [minSafeGap, collisionTimeIdx, severity] = computeFollowingMSS(obj, leader)
            %computeFollowingMSS Computes minimum following distance similar to Kanaris
            %work.
            %   Solution is found by directly solving the integral for all t and then
            %   looking for the maximum delta S.
            
            % Min safe gap
            deltaGap = leader.deltaXt - obj.deltaXt;
            minSafeGap = -min(deltaGap);
            % Collision instant
            gap = leader.x0 + deltaGap;
            collisionTimeIdx = find(gap<=0, 1);
            if ~isempty(collisionTimeIdx)
                severity = obj.vt(collisionTimeIdx) - leader.vt(collisionTimeIdx);
            else
                severity = 0;
            end
            
        end
        
        function fig = plotKinematics(obj, simTime)
            fig = figure;
            x = [obj.at, obj.vt, obj.deltaXt];
            for k = 1:3
                subplot(3, 1, k);
                plot(simTime, x(:, k));
            end
        end
        
                
        function value = get.position(obj)
            value = obj.currentState(obj.positionIdx);
        end
        
        function value = get.velocity(obj)
            value = obj.currentState(obj.velocityIdx);
        end
    end
    
end

