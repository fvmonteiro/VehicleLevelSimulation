classdef Lane < handle
    %Lane Class representing a single lane
        
    properties (SetAccess = private)
        id % lane unique id
        vlim % lane speed limit
        veh % id of vehicles in this lane
    end
    
    methods
        function obj = Lane(id, vlim)
            %Lane Construct an instance of this class
            if nargin>0
                obj.id = id;
                obj.vlim = vlim;
                obj.veh = [];
            end
        end
                       
        function [] = add_veh(obj, veh)
            obj.veh = [obj.veh veh];
%             return_obj = obj;
        end
        
        function v = speed(obj)
            v = zeros(length(obj), 1);
            for k = 1:length(obj)
                if isempty(obj(k).veh)
                    v(k) = obj(k).vlim;
                else
                    veh_speeds = [obj(k).veh.speed];
                    v(k) = min([obj(k).vlim veh_speeds]);
                end
            end
        end
        
    end
end

