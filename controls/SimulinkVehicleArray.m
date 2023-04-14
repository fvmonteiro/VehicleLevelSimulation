classdef SimulinkVehicleArray < VehicleArray
    %SimulinkVehicleArray 
    %   Detailed explanation goes here
    
    properties
        simName
        lcStrategy = 'synchronous'
    end
    
    properties (Dependent)
        simTime
        lcState
    end
       
    methods
        function obj = SimulinkVehicleArray(nv, simName, vehNames, ...
                vehType, lcStrategy)
            %VehicleArray Construct an instance of this class
            
            %   We just create an array of vehicles
            if nargin > 0
                % Create all objects
                obj.simName = simName;
                
                if ~iscell(vehNames)
                    baseName = vehNames;
                    vehNames = cell(nv, 1);
                    for n = 1:nv
                        vehNames{n} = [baseName num2str(n)];
                    end
                end
                
                if ~iscell(vehType)
                    type = vehType;
                    vehType = cell(nv, 1);
                    for n = 1:nv
                        vehType{n} = type;
                    end
                end
                
                % Optional parameter
                if nargin>4
                    obj.lcStrategy = lcStrategy;
                end
                
                obj.vehs = SimulinkVehicle.empty;
                obj.vehs(nv) = SimulinkVehicle();
                for n = obj.nv:-1:1
                    obj.vehs(n) = SimulinkVehicle(vehNames{n}, vehType{n},...
                        obj.simName, obj.lcStrategy);
                end
            end

        end
        
        %%% Create platoon functions %%%
        
        function [] = createHomogeneousSSPlatoon (obj, v0, y0, maxVel, ...
                vehFollCtrlParams, velCtrlPoles)
           %createHomogeneousSSPlatoon Create a platoon with vehicles at 
           %steady-state, that is, all at the same speed at their desired 
           %distances
            obj.createVehicles(v0, y0, maxVel, vehFollCtrlParams, ...
                velCtrlPoles, 1, 0);
        end
        
        function [] = createEquallySpacedVehArray(obj, v0, y0, maxVel, ...
                vehFollCtrlParams, velCtrlPoles, rho)
            obj.createVehicles(v0, y0, maxVel, vehFollCtrlParams, ...
                velCtrlPoles, rho, 0);
        end
        
        function [] = createRandomlySpacedVehArray(obj, v0, y0, maxVel, ...
                vehFollCtrlParams, velCtrlPoles, lambdaExtraGap)
            obj.createVehicles(v0, y0, maxVel, vehFollCtrlParams, 1, ...
                velCtrlPoles, lambdaExtraGap);
        end
        
        function [] = createVehicles(obj, v0, y0, maxVel, vehFollParams, ...
                velCtrlPoles, rho, lambdaExtraGap)
                        
            initState = [0; y0; v0; 0; 0]; %[x, y, vx, ax, psi]
            extraGap = exprnd(lambdaExtraGap, obj.nv, 1);
            
            if size(vehFollParams, 1) == 1
                % only one set of paramters for the whole array
                vehFollParams = repmat(vehFollParams, obj.nv, 1);
            end
            if size(velCtrlPoles, 1) == 1
                % only one set of poles for the whole array
                velCtrlPoles = repmat(velCtrlPoles, obj.nv, 1);
            end
            
            if rho<1
                warning('Setting distances smaller than the minimum safe gap')
            end
                       
            ssGap = 0;
            for n = obj.nv:-1:2
                initState = initState + [ssGap; 0; 0; 0; 0];
                obj.vehs(n).leader = obj.vehs(n-1);
                obj.createSingleVehicle(obj.vehs(n), initState, maxVel,...
                    vehFollParams(n, :), velCtrlPoles(n, :));
                ssGap = (rho+extraGap(n))*obj.vehs(n).minVFGap0 ...
                    + obj.vehs(n).len;
            end
            initState = initState + [ssGap; 0; 0; 0; 0];
            obj.createSingleVehicle(obj.vehs(1), initState, maxVel,...
                vehFollParams(1, :), velCtrlPoles(1, :));
                
        end
        
        %%% Setting parameters for the whole platoon %%%
        
        function [] = setLatControllers(obj, laneChangeVel, latParams)
            if any(size(latParams, 1)==1)
                % only one set of parameters for the whole array
                latParams = repmat(latParams, obj.nv, 1);
            end
            
            for n = 1:obj.nv
                obj.vehs(n).setLatController(laneChangeVel, latParams(n, :));
            end
        end
        
        function [] = setSimParams(obj)
            for n = 1:obj.nv
                obj.vehs(n).setSimParams();
            end
        end
        
        function [] = setSimLateralParams(obj)
            for n = 1:obj.nv
                obj.vehs(n).setSimLateralParams();
            end
        end
        
        function [] = simplifyVehicles(obj, names)
            %simplifyVehicles Sets the property isSimple of the chosen
            %vehicles as true.
            % "Simple" vehicles are used to speed simulation
            if ~iscell(names)
                names = {names};
            end
            for n = 1:length(names)
                obj.getVehByName(names{n}).isSimple = true;
            end
        end
        
        
        %%% OTHER HELPER METHODS %%%
        
        function [lcVehArray] = getLaneChangingVehicleArray(obj)
            
            platoonIdx = contains(obj.vehNames, 'p','IgnoreCase',true) | ...
                contains(obj.vehNames, 'E', 'IgnoreCase',true);
            if sum(platoonIdx)==0
                warning('No lane changing vehicles in this array');
            end
            lcVehArray = SimulinkVehicleArray.createArrayFromVehicles(...
                obj.vehs(platoonIdx));
        end
        
        %%% GETTERS %%%

        function value = get.lcState(obj)
            value = [obj.vehs.lcState];
        end
        
        function value = get.simTime(obj)
            value = obj.vehs(1).simTime;
        end
        
        %%% PLOT FUNCTIONS %%%
         
        function [fig] = plotSimErrors(obj, chosenVehNames, errorsToPlot, ...
                baseFig)
            % plotSimErrors plots the errors obtained directly from the
            % Simulink simulation
            
            if nargin<3
                errorsToPlot = SimulinkVehicle.simErrorNames;
            end
            
            if ~iscell(chosenVehNames)
                chosenVehNames = {chosenVehNames};
            end
            
            if nargin>3
                fig = obj.getVehByName(chosenVehNames{1}).plotSimErrors(...
                    errorsToPlot, baseFig);
            else
                fig = obj.getVehByName(chosenVehNames{1}).plotSimErrors(...
                    errorsToPlot);
            end
            for n = 2:length(chosenVehNames)
                fig = obj.getVehByName(chosenVehNames{n}).plotSimErrors(...
                    errorsToPlot, fig);
            end
             
        end
        
        function [fig] = plotManeuverStates(obj, showTime)
            
            if nargin<2
                showTime = true;
            end
            
            fig = figure;
            lineStyles = {'-.', '-', '--', '-.'};
            vehicles = obj.vehs(~[obj.vehs.isSimple]);
            for k = 1:length(vehicles)
                vehicles(k).plotManeuverStates(showTime, fig, ...
                    'LineWidth', 0.5+0.5*k, ...
                    'LineStyle', lineStyles{rem(k, length(lineStyles))+1});
            end
            
        end
        
    end
    
    methods (Access = private)
        function [] = createSingleVehicle(obj, veh, initState, maxVel, ...
                vehFollParams, velCtrlPoles)
            veh.initialState = initState;
            if contains(veh.name, 'L','IgnoreCase',true) % leaders
                veh.desiredVelocity = veh.vx0;
            else
                veh.desiredVelocity = maxVel;
            end
            veh.computeTimeHeadway(maxVel, obj.velRatio);
            veh.computeTimeHeadway(maxVel, 0.05, true);
            veh.setController(obj.vehFollCtrlType, vehFollParams);
            veh.setVelController(velCtrlPoles);
        end
    end
    
    methods (Static)
        function vehArray = createArrayFromVehicles(vehicles)
            vehArray = SimulinkVehicleArray();
            vehArray.simName = vehicles(1).simulationName;
            vehArray.lcStrategy = vehicles(1).platoonLCStrategy;
            vehArray.vehs = vehicles;
            vehArray.vehFollCtrlType = vehicles(1).controller.type;
        end
    end

    % [3/24/21]: made vehNames a mandatory parameter
%     methods (Static, Access = private)
%         % Used to know total number of vehicles in simulation
%         function value = setgetTotalVehs(newTotalVehs)
%             persistent totalVehs;
%             if isempty(totalVehs)
%                 totalVehs = 0;
%             end
%             if nargin
%                 totalVehs = newTotalVehs;
%             end
%             value = totalVehs;
%         end
%     end

end

