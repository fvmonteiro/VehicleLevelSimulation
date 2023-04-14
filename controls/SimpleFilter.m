classdef SimpleFilter < handle
    %SimpleFilter Contains methods for some basic measurement filtering
    %   Detailed explanation goes here
    
    properties
        type
        previousValue
        minValue
        maxValue
        minVariation
        maxVariation
        coeff
%         deltaT
    end
    
    methods
        function obj = SimpleFilter(type, params)
            %SimpleFilter Defines filter type and sets relevant parameters
            obj.type = type;
            switch type
                case 'valueAndRateSaturation'
                    obj.previousValue = params(1);
                    obj.minValue = params(2);
                    obj.maxValue = params(3);
                    obj.minVariation = params(4);
                    obj.maxVariation = params(5);
                case 'variationFilter'
                    obj.previousValue = params(1);
                    obj.minVariation = params(2);
                    obj.maxVariation = params(3);
                    obj.coeff = params(4);
                otherwise
                    error('Unknwon filter type')
            end
            
        end
        
        function filteredValue = runFilter(obj, newValue, deltaT)
            
            switch obj.type
                case 'variationFilter'
                    variation = obj.coeff*(newValue - obj.previousValue);
                    if variation <= obj.minVariation*deltaT
                        variation = obj.minVariation*deltaT;
                    elseif variation >= obj.maxVariation*deltaT
                        variation = obj.maxVariation*deltaT;
                    end
                    filteredValue = obj.previousValue + variation;
                case 'valueAndRateSaturation'
                    filteredValue = newValue;
                    variation = filteredValue-obj.previousValue;
                    if variation<= obj.minVariation*deltaT
                        filteredValue = obj.previousValue + obj.minVariation*deltaT;
                    elseif variation >= obj.maxVariation*deltaT
                        filteredValue = obj.previousValue + obj.maxVariation*deltaT;
                    end
                    filteredValue = max(min(filteredValue, obj.maxValue), obj.minValue);
            end
            
            obj.previousValue = filteredValue;
        end
    end
end

