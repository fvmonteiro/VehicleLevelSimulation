classdef Vehicle < handle
    %Vehicle class contains only vehicle data placeholders
    
    properties (SetAccess = protected)
        name
        maxAccel % [m/s^2]
        minAccel % [m/s^2]
        maxJerk % [m/s^3]
        minJerk % [m/s^3]
        reactionTime %[s]
        h %[s]
        d0 % [m]
    end
    
    methods
        function obj = Vehicle()
            %Vehicle empty constructor
        end
        
        function [h, d0] = safeHeadway(obj)
            %safeHeadway computes the minimum safe following distance considering the
            %case where the leader brakes with maximum force and main vehicle is
            %travelling with maximum acceleration
            
            % Detailed explanation in safeHeadway function file in the
            % simulink_simulations folder
            
            T = obj.reactionTime;
            t1 = (obj.minAccel-obj.maxAccel)/obj.minJerk;
            h = T + t1 - 1/obj.minAccel*(obj.maxAccel*T + obj.maxAccel*t1+1/2*obj.minJerk*t1^2);
            d0 = 1/2*obj.maxAccel*T^2 + obj.maxAccel*T*t1 + + 1/2*obj.maxAccel*t1^2 + 1/6*obj.minJerk*t1^3 - ...
                1/(2*obj.minAccel)*(obj.maxAccel*T + obj.maxAccel*t1+ 1/2*obj.minJerk*t1^2)^2;
            
        end
        
        function [gap] = minGap(obj, v, vl, bl)
            %minGap computes minimum distances from vehicle to a leader
            
            bf = abs(obj.minAccel);
            % if the leader is "faster" (depends on vehicles' braking
            % capabilities), we ignore the quadractic term
            gap = obj.h*v+obj.d0 + ...
                (v.^2/bf-vl.^2/bl>0).*(v.^2/bf-vl.^2/bl)/2; 
            
        end
        
        
    end
end

