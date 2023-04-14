classdef Vehicle < handle
    %Vehicle Class describing vehicles
    
    properties (SetAccess = private)
        id
        x0
        len
        xt
    end
    
    methods
        function obj = Vehicle(id, x0, length)
            %Vehicle Construct an instance of this class
            obj.id= id;
            obj.x0 = x0;
            obj.len = length;
        end
      
        function [x] = veh_dynamics(obj, acc, t)
        %veh_dynamics Computes vehicle movement given an acceleration
            x = zeros(3, length(t));
            x(3, :) = acc;
            x(2, :) = obj.x0(2) + cumtrapz(t, acc);
            x(1, :) = obj.x0(1) + cumtrapz(t, x(2,:));
            obj.xt = x;
        end
        
        function [s] = init_position(obj)
            s = obj.x0(1);
        end
        
        function [v] = speed(obj)
            v = obj.x0(2);
        end
        
    end
end

