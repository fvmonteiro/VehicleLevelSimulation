classdef (Abstract) VehicleArray < handle
    %PLATOON Abstract class for platoons of vehicles
    %   Detailed explanation goes here
    
    properties
        vehs
        leader
        hasSpecialLeader = false;
        nv
        velRatio = 1; % expected vLeader/vFollower in the array
        vehFollCtrlType = 'ACC PID' % default no comms
    end
    
    properties (Dependent, Hidden)
        len
        states
        inputs
        vehNames
        
        nStates
        
        nInputs
    end
    
    methods (Abstract)
        
        createVehicles(obj, v0)
        
    end
    
    methods
            
        function setDesiredVel(obj, desVel)
            if obj.hasSpecialLeader
                obj.leader.desiredVelocity = desVel;
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n).desiredVelocity = desVel;
            end
        end
        
        function [] = setSimTime(obj, simTime)
            if obj.hasSpecialLeader
                obj.leader.simTime = simTime;
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n).simTime = simTime;
            end
        end
        
        function [] = setActuatorLagForAllVehicles(obj, tau)
            for n = 1:obj.nv
                obj.vehs(n).tau = tau;
            end
        end
        
        function [] = addComms(obj)
            obj.vehFollCtrlType = 'CACC PD';
        end
        
        function [] = shiftVehArray(obj, xShift)
            % shiftVehArray Shifts the longitudinal positions of all
            % vehicles in the array by a value xShift
            for n = 1:obj.nv
                obj.vehs(n).x0 = obj.vehs(n).x0 + xShift;
            end
        end
        
        %%% GETTERS %%%
        
        function value = get.vehNames(obj)
            value = {obj.vehs.name};
        end
        
        function [nv] = get.nv(obj)
            if obj.hasSpecialLeader
                nv = length(obj.vehs) + 1;
            else
                nv = length(obj.vehs);
            end
        end
        
        function [value] = get.len(obj)
            value = 0;
            if obj.hasSpecialLeader
                value = value + obj.leader.len;
                value = value + obj.vehs(1).minVFGap0;
            end
            value = value + obj.vehs(1).len;
            for n = 2:length(obj.vehs)
                value = value + obj.vehs(n).len + obj.vehs(n).minVFGap0;
            end
            
        end
        
        function [matrix] = get.states(obj)
            %%% TODO: won't work when there is specialLeader
            if obj.hasSpecialLeader
                error('Function not yet coded for Coop Platoon Leader')
            end
            [r, c] = size(obj.vehs(1).states);
            matrix = zeros(r, c, obj.nv);
            for n = 1:length(obj.vehs)
                matrix(:, :, n) = obj.vehs(n).states;
            end
        end
        
        function [matrix] = get.inputs(obj)
            %%% TODO: won't work when there is specialLeader
            if obj.hasSpecialLeader
                error('Function not yet coded for Coop Platoon Leader')
            end
            [r, c] = size(obj.vehs(1).inputs);
            matrix = zeros(r, c, obj.nv);
            for n = 1:length(obj.vehs)
                matrix(:, :, n) = obj.vehs(n).inputs;
            end
        end
        
        function value = get.nStates(obj)
            value = obj.vehs(1).nStates;
        end
        
        function value = get.nInputs(obj)
            value = obj.vehs(1).nInputs;
        end

        function [requestedVeh] = getVehByName(obj, requestedNames)
        %getVehByName returns a Vehicle object array
            
            % Allow argument to be a string (or array of characters)
            if ~iscell(requestedNames)
                requestedNames = {requestedNames};
            end

            vehIdx = 0;
            for n = 1:length(requestedNames)
                newIdx = strcmpi(obj.vehNames, requestedNames{n});
                if sum(newIdx)==0
                    warning('Vehicle %s not found in this array', ...
                        requestedNames{n});
                end
                vehIdx = vehIdx | newIdx;
            end
            requestedVeh = obj.vehs(vehIdx);
        end

        function [egoVehicle] = getEgoVehicle(obj)

            % In simulation with a single lane changing vehicle, the
            % vehicle is usualy called E or ego. In simulations with 
            % platoons, the vehicles are called p1, p2, ..., pN
            egoVehicle = obj.getVehByName('E');
            if isempty(egoVehicle)
                egoVehicle = obj.getVehByName('ego');
                if isempty(egoVehicle)
                    egoVehicle = obj.getVehByName('p1');
                end
            end
            if (isempty(egoVehicle))
                error(["No ego vehicle found.\nOnly vehicles in array are" ...
                    obj.vehNames])
            end
        end
        
        %%% PLOT FUNCTIONS %%%
        function fig = plotStatesAllVehs(obj, statesToPlot)
            fig = obj.plotStates(1:obj.nv, statesToPlot);
        end
        
        function fig = plotStates(obj, chosenVehNames, statesToPlot)
            %plotStates plots state of vehicles in platoon. Both arguments
            %can be string/char or cells of those
            
            if isnumeric(chosenVehNames)
                chosenVehs = obj.vehs(chosenVehNames);
            else
                chosenVehs = obj.getVehByName(chosenVehNames);
            end
                        
            fig = figure;
            for n = 1:length(chosenVehs)
                fig = chosenVehs(n).plotStates(statesToPlot, fig, ...
                    chosenVehs(n).leader);
            end
        end
    end
end