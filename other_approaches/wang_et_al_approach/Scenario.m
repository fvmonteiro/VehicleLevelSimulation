classdef Scenario
    %Scenario Class contains data on lanes and vehicles in the scenario
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        lanes % (object) lanes in the road
        vehicles % (object) non-controlled vehicles
        Tp % prediction time
        lane0 % initial lane of autonomous vehicle
    end
    
    methods
        function obj = Scenario(lanes,vehicles, Tp, lane0)
            %Scenario Construct an instance of this class
            obj.lanes = lanes;
            obj.vehicles = vehicles;
            obj.Tp = Tp;
            if nargin>3
                obj.lane0 = lane0;
            else
                obj.lane0 = length(lanes);
            end
        end
        
        function lanes = lane_by_id(obj, lane_id)
            lanes = cell(1, length(lane_id));
            for l = 1:length(lane_id)
                lanes{l} = obj.lanes([obj.lanes.id]==lane_id(l));
            end
            lanes = [lanes{:}];
        end
%         
%         function vehicles = get.vehicles(obj)
%             vehicles = obj.vehicles;
%         end
        
    end
end

