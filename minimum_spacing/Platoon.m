classdef Platoon < handle % not sure if inherit from Vehicle
    %Platoon Set of vehicles
    %   Detailed explanation goes here
    
    properties
        vehicles
    end
    properties (Dependent)
        leader
        last
        len
        v0
    end
    
    methods
        function obj = Platoon(type, x0, y0, v0, reactionTime, interPlatoonReactionTime, nVehs)
            %Platoon Create a uniform platoon of vehicles
            %   Parameters should represent state of the platoon leader
            vehicles = Vehicle.empty(nVehs, 0);
            vehicles(1) = Vehicle(type, 'P1', x0, y0, v0, reactionTime);
            x = x0;
            for k = 2:nVehs
                vehicles(k) = Vehicle(type, ['P' num2str(k)], 0, y0, v0, interPlatoonReactionTime);
                [h, d0] = vehicles(k).safeHeadwayParams();
                x = x - vehicles(k-1).len - h*vehicles(k).v0 - d0;
                vehicles(k).x0 = x;
            end
            
            obj.vehicles = vehicles;
        end
        
        function leader = get.leader(obj)
            leader = obj.vehicles(1);
        end
        
        function last = get.last(obj)
            last = obj.vehicles(end);
        end
        
        function len = get.len(obj)
            % NOTE: ignoring transient! We are analyzing only cases where
            % the platoon is assumed to be at steady state
            len = obj.vehicles(1).len;
            for k = 2:length(obj.vehicles)
                [h, d0] = obj.vehicles(k).safeHeadwayParams(); % we're allowing vehicles to not be perfectly homogeneous
                len = len + h*obj.vehicles(k).v0 + d0 + obj.vehicles(k).len;
            end
            
            % or just:
            % len = obj.leader.x0 - (obj.last.x0 - obj.last.len);
            
        end
        
        function v0 = get.v0(obj)
            v0 = obj.leader.v0;
        end
    end
end

