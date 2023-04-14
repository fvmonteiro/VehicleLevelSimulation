classdef Vehicle < handle
    %Vehicle Class to hold vehicle parameters in MSS for lane changes
    
    properties
        width
        len
        y0
        v0
        a
    end
    properties (SetAccess = protected)
        name
    end
    
    methods
        function obj = Vehicle(name, width, len, y0, v0, accel)
            %Vehicle Construct an instance of this class
            obj.width = width;
            obj.len = len;
            obj.name = name;
            obj.y0 = y0;
            obj.v0 = v0;
            obj.a = accel;
        end
        
        function v = computeVel(obj, ts)
            v = obj.v0 + cumtrapz(ts, obj.a);
        end
        
    end
end

