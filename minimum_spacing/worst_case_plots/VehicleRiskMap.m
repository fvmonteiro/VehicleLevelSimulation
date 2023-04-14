classdef VehicleRiskMap < VehicleLongitudinal
    %VehicleRiskMap The class inherits VehicleLongitudinal and includes
    %analysis for simplified lane change trajectories to create a risk map
    %around the vehicle
    
    properties
        y0 % [m] initial lateral position
        
        % Parameters during lane change
        maxAccelLC
        maxBrakeLC
        avgBrakeLC
        lambda1LC
        d0LC
        hLC
        
    end
    
    properties (SetAccess = protected)
        vLat % [m/s] velocity acceleration during whole simulation
        aLat % [m/s^2] lateral acceleration during whole simulation
    end
    
    methods
        function obj = VehicleRiskMap(type, name, q0, delay)
            obj@VehicleLongitudinal(type, name, q0([1,3]), delay);
            obj.y0 = q0(2);
            obj.maxAccelLC = 0;
            
            switch obj.type
                case 'P' % passenger vehicle
                    obj.maxBrakeLC = obj.maxBrake/2;
                case 'B' % bus
                    error('Bus lane change properties not yet coded');
                case 'T' % truck
                    obj.maxBrakeLC = 2;
                otherwise
                    error('Unknown vehicle type')
            end
            
%             obj.safeHeadwayParamsDuringLC();
            
        end
        
        %%% MAYBE CREATE computeFollowingMSSDuringLC function?
%         function [minSafeGap, analyticalGap] = computeFollowingMSS(obj, leader, simTime)
%             
%             leader.brakeFullStop(simTime, obj); 
%             obj.brakeFullStop(simTime, leader);
%             
%             [minSafeGap, analyticalGap] = computeFollowingMSS@VehicleLongitudinal(obj, leader);
%             
%         end
        
        function [collisionFreeGap, vehFollGap] = ...
                computeFollowingMSSDuringLC(obj, simTime, leader, accelLC)
            
            %%% The two cases below will yield if the vehicle comes to a
            %%% complete stop within tLat. For the current simulation
            %%% parameters, this is the case.
            % Limited braking only during lane change
%             obj.followerBrakeFullStopDuringLC(simTime, tLat);
%             [minSafeGap, ~] = obj.computeFollowingMSS(leader);
%             obj.avgBrakeLC = obj.computeAvgBrakeForLC(simTime, tLat);
            
            if nargin<4
                warning(['[11/4/20] This function has a new parameter '...
                    'which will become mandatory'])
                obj.maxAccelLC = 0;
            else
                if accelLC
                    obj.maxAccelLC = sign(leader.v0-obj.v0)*abs(obj.comfBrake);
                else
                    obj.maxAccelLC = 0;
                end
            end
            
            % Limited braking till full stop
            obj.avgBrakeLC = obj.maxBrakeLC;
            followerBrakeFullStopAvgLC(obj, simTime);
            collisionFreeGap = obj.computeFollowingMSS(leader);
            
            obj.setHeadwayDuringLC(leader);
            vehFollGap = obj.hLC*obj.v0 + obj.d0LC;

%             collisionFreeGap = max(collisionFreeGap, vehFollGap);
            
            % For comparison
%             analyticalGap = obj.analyticalFollowingMSS(leader, obj.avgBrakeLC, obj.lambda1LC, obj.d0LC);
        end
        
        function [lambda1LC, d0] = safeHeadwayParamsDuringLC(obj)
            %safeHeadwayParamsDuringLC is like safeHeadwayParams but it
            %uses accel and brake values during lane change.
            
            % Code expects: obj.maxBrake>0 and obj.maxJerk>0
            
            accel = obj.maxAccelLC;
            brake = obj.maxBrakeLC;
%             brake = obj.avgBrakeLC;
            jerk = obj.maxJerk;
            if (obj.timeToBrake==0) || (obj.timeToEmergencyBrake==0)
                delay = obj.timeToBrake + obj.timeToEmergencyBrake;
            else
                delay = obj.timeToBrake + (obj.comfBrake+accel)/obj.comfJerk + obj.timeToEmergencyBrake;
            end
            
            [lambda1LC, d0] = obj.computeHeadwayParams(delay, accel, brake, jerk);
            
            obj.lambda1LC = lambda1LC;
            obj.d0LC = d0;
            
        end
        
        function [] = setHeadwayDuringLC(obj, leader)
            obj.safeHeadwayParamsDuringLC();
            brakeDuringLC = obj.maxBrakeLC;
            gammaD = brakeDuringLC/leader.maxBrake;
            gammaV = 1; % we assume equal speeds during lane changing
            lcVel = obj.v0;
            obj.hLC = obj.lambda1LC + (1-gammaD*gammaV^2)*lcVel/(2*brakeDuringLC);
        end
        
        function [] = followerBrakeFullStopDuringLC(obj, simTime, tLat)
            %followerBrakeFullStopDuringLC Create the follower's 
            % acceleration profile during lane change - we assume connected
            % vehicles only
            
            if obj.timeToEmergencyBrake >0 % not a connected vehicle
                error('Cannot yet simulate a non-connected vehicle lane change')
            end
            
            % Switching points
            % Veh brakes with Dlc for tLC seconds, then goes to D
            jerk = [0, -obj.maxJerk, 0, -obj.maxJerk, 0, obj.maxPosJerk];
            accel = [obj.maxAccelLC, obj.maxAccelLC, -obj.maxBrakeLC, -obj.maxBrakeLC, -obj.maxBrake, -obj.maxBrake];
            nPhases = length(jerk);
            
            switchDelays = -ones(1, nPhases);
            
            switchDelays(1) = obj.timeToBrake;
            switchDelays(2) = (accel(3)-accel(2))/jerk(2);
            switchDelays(3) = tLat - sum(switchDelays(1:2));
            switchDelays(4) = (accel(5)-accel(4))/jerk(4);
            switchDelays(end) = (0-accel(end))/jerk(end);
            
            obj.kinematicProfileFromSwitchPoints(jerk, accel, switchDelays, simTime);
            
        end
        
        function [] = followerBrakeFullStopAvgLC(obj, simTime)
            %followerBrakeFullStopDuringLC Create the follower's 
            % acceleration profile during lane change - we assume connected
            % vehicles only
            
            if obj.timeToEmergencyBrake >0 % not a connected vehicle
                error('Cannot yet simulate a non-connected vehicle lane change')
            end
            
%             obj.avgBrakeLC = obj.computeAvgBrakeForLC(simTime, tLat);
            
            % Switching points
            % Veh brakes with mean(Dlc)
            jerk = [0, -obj.maxJerk, 0, obj.maxPosJerk];
            accel = [obj.maxAccelLC, obj.maxAccelLC, -obj.avgBrakeLC, -obj.avgBrakeLC];
            nPhases = length(jerk);
            
            switchDelays = -ones(1, nPhases);
            
            switchDelays(1) = obj.timeToBrake;
            switchDelays(2) = (accel(3)-accel(2))/jerk(2);
            switchDelays(end) = (0-accel(end))/jerk(end);
            
            obj.kinematicProfileFromSwitchPoints(jerk, accel, switchDelays, simTime);
            
        end
        
        function severityByDistance = computeSeverityByDistance(obj, ...
                leader, minSafeGap, deltaS)
            % Same as running computeSeverity from parent class, but the
            % possible x0 of the leader are determined internally
            
            initGap = 0:deltaS:ceil(minSafeGap);
            
            severityByDistance = obj.computeSeverity(leader, initGap);
            
        end
        
        function [] = sinLatAccel(obj, simTime, tAdj, tLat, latDeltaS)
            %sinLatAccel Creates sinusoidal profile for lateral acceleration
            % t: simulation time
            % tAdj: maneuver start time (longitudinal adjustment time) [s]
            % tLat: lane change duration [s]
            % latDeltaS: total lateral displacement [m]
            
            obj.aLat = zeros(length(simTime), length(tAdj));
            for n = 1:length(tAdj)
                nonZeroIndices = simTime>=tAdj(n) & simTime<=(tLat+tAdj(n));
                obj.aLat(nonZeroIndices, n) = 2*pi*latDeltaS/(tLat^2) * sin(2*pi/tLat * (simTime(nonZeroIndices)-tAdj(n)));
            end
            
            obj.vLat = cumtrapz(simTime, obj.aLat);
            
        end
        
        function accelDuringLC(obj, simTime, leaderV0)
            % Comf accel to adjust to the maneuver
            accelSign = (leaderV0-obj.v0);
            accel = accelSign*abs(obj.comfBrake);
            jerk = accelSign*abs(obj.comfJerk);
            t1 = accel/jerk;
            deltaV = leaderV0 - obj.v0;
            deltaT = deltaV/accel - 2*t1;
            t3 = t1+deltaT;
            at = [simTime(simTime<=t1)*jerk, ...
                ones(1, sum(simTime>t1 & simTime<=t3))*accel, ...
                -(simTime(simTime>t3 & simTime<t3+t1)-t3)*jerk];
            if ~isempty(at)
                at(end) = 0;
            end
            obj.at = [at, zeros(1,length(simTime)-length(at))];
            obj.vt = obj.v0 + cumtrapz(simTime, obj.at)';
        end
        
        function deltaG = computeGapVariationDuringLC(obj, simTime, ...
                otherVeh, tLat)
            % # of relative speeds being tested (we assume all vehicles other than the
            % ego have the same number of possible initial speeds)
            
            vehName = char(otherVeh.name);
            
            switch upper(vehName(1))
                case 'L'
                    vl = otherVeh.vt;
                    vf = obj.vt;
                case 'F'
                    vl = obj.vt;
                    vf = otherVeh.vt;
                otherwise
                    error('unknown vehicle')
            end
            
            deltaS = cumtrapz(simTime, vf-vl); % relative position
            
            switch lower(vehName(2))
                case {'d', 'r', 'l'} % destination, right, or left
                    intervalIdx = 1;
                case 'o'
                    intervalIdx = 2;
                otherwise
                    error('unknown vehicle')
            end
            
            tc = obj.crossingTime(simTime, otherVeh);
            
            interval = [0 tLat];
            interval(intervalIdx) = tc;
            indices = simTime>=interval(1) & simTime<=interval(2);
            deltaG = max(deltaS(indices));
            
        end
        
        function avgAccel = computeAvgBrakeForLC(obj, simTime, tLat)
            obj.followerBrakeFullStopDuringLC(simTime, tLat);
            deltaX = obj.deltaXt(end);
            
            options = optimoptions('fsolve','Display','off');
            fun = @(x)obj.deltaXEquationForAvgAccel(deltaX, x);
            x0 = (obj.maxBrake+obj.maxBrakeLC)/2;
            [sol, ~, exitflag, ~] = fsolve(fun, x0, options);
            if (exitflag<=0)
                error('[Min time and fuel] No solution found.')
            end
            avgAccel = sol;
        end
        
    end
    
    methods (Access = protected)
        function [tc] = crossingTime(obj, simTime, otherVeh)
            %crossingTime Find the time after which the merging vehicle might collide
            % with others on the adjacent lane
            
%             vLat = obj.computeVelLat(simTime);
            vEgo = obj.vt;
            
            yCenter = obj.y0 + cumtrapz(simTime, obj.vLat);
            yFrontLeft = yCenter + (obj.width/2)*vEgo./sqrt(obj.vLat.^2+vEgo.^2);
            
            y = yFrontLeft;
            vehName = char(otherVeh.name);
            switch vehName(1)
                case 'L'
                    % do nothing
                case 'F'
                    y = y - obj.len*obj.vLat./sqrt(obj.vLat.^2+vEgo.^2); %y rear left
                otherwise
                    error('unknown vehicle')
            end
            
            switch vehName(2)
                case {'d', 'l'} % d: destination or l: left
                    yCrossing = otherVeh.y0-otherVeh.width/2;
                case 'o'
                    yCrossing = otherVeh.y0+otherVeh.width/2;
                    y = y - obj.width*vEgo./sqrt(obj.vLat.^2+vEgo.^2); % y right
                otherwise
                    error('unknown vehicle')
            end
            
            [~, idx] = min(abs(y-yCrossing)); % we can code it this way because
            % we know there's a single crossing point.
            
            tc = simTime(idx);
            
        end
        
    end
    
    methods (Access = private)
        function [f] = deltaXEquationForAvgAccel(obj, deltaX1, var)
            brake = var;
            
            accel = obj.maxAccelLC;
            jerk = obj.maxJerk;
            v0 = obj.v0;
            
            delay = obj.timeToBrake + obj.timeToEmergencyBrake;
            t1 = (accel+brake)/jerk;
            
            lambda = delay + t1 + 1/brake*(accel*delay + accel*t1-1/2*jerk*t1^2);
            d0 = 1/2*accel*delay^2 + accel*delay*t1 + 1/2*accel*t1^2 - 1/6*jerk*t1^3 + ...
                1/(2*brake)*(accel*delay + accel*t1 - 1/2*jerk*t1^2)^2;
            
            f = deltaX1 - (v0^2/2/brake + lambda*v0 + d0);
        end
    end
    
    methods (Static)
        function [newVeh] = copyVehicle(oldVeh)
            newVeh = VehicleRiskMap(oldVeh.type, oldVeh.name, [oldVeh.x0, oldVeh.y0, oldVeh.v0], [oldVeh.timeToBrake, oldVeh.timeToEmergencyBrake]);
        end
    end
    
end