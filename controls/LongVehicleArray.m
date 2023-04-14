classdef LongVehicleArray < VehicleArray
    
    properties
        vehBaseName = 'veh'
        leaderNumber = 1
    end
    
    methods
        function obj = LongVehicleArray(nv)
            %LongVehicleArray Construct an instance of this class
            %   We just create an array of vehicles
            if nargin>0
                if nv>1
                    obj.vehs = LongitudinalVehicleModel.empty;
                    obj.vehs(nv) = LongitudinalVehicleModel();
                else
                    warning("[LongVehicleArray] Single vehicle platoon.")
                    obj.vehs = LongitudinalVehicleModel();
                end
            end

        end
                
        function [] = createVehicles(obj, v0, maxVel, simTime, ...
                vehFollCtrlParams, velCtrlPoles, tau, rho, lambdaExtraGap)
            
            extraGap = exprnd(lambdaExtraGap, obj.nv, 1);
            
            if size(vehFollCtrlParams, 1)==1
                % only one set of parameters  for the whole array
                vehFollCtrlParams = repmat(vehFollCtrlParams, obj.nv, 1);
            end
            
            if rho<1
                warning('Setting distances smaller than the minimum safe gap')
            end
            
            % Create all objects
            for n = 1:obj.nv-obj.hasSpecialLeader
                obj.vehs(n) = LongitudinalVehicleModel([obj.vehBaseName ...
                    num2str(obj.nv - n + obj.leaderNumber)], simTime);
            end
            
            % Include vehicles in simulation: assign leaders, compute
            % headways etc
            if tau > 0
                initState = zeros(3, 1);
            else
                initState = zeros(2, 1);
            end
            initState(2) = v0;
            for n = 1:obj.nv-obj.hasSpecialLeader-1
                obj.vehs(n).includeInSimulation(initState, tau);
                obj.vehs(n).leader = obj.vehs(n+1);
                obj.vehs(n).computeTimeHeadway(maxVel, obj.velRatio);
                obj.vehs(n).setController(obj.vehFollCtrlType,...
                    vehFollCtrlParams(n, :));
                obj.vehs(n).setVelController(velCtrlPoles);
                ssGap = (rho+extraGap(n))*obj.vehs(n).minVehFollGap ...
                    + obj.vehs(n).len;
                initState(1) = initState(1) + ssGap;
            end
            obj.vehs(n+1).includeInSimulation(initState, tau);
            obj.vehs(n+1).computeTimeHeadway(maxVel, obj.velRatio);
            obj.vehs(n+1).setController(obj.vehFollCtrlType,...
                vehFollCtrlParams(n, :));
            obj.vehs(n+1).setVelController(velCtrlPoles);
                
            % Platoon leader has index 1, last vehicle has index nv
            obj.vehs = flip(obj.vehs);
        end
        
        function [] = createHomogeneousSSPlatoon (obj, v0, maxVel, ...
                simTime, vehFollCtrlParams, velCtrlPoles, tau)
            %createHomogeneousSSPlatoon Create a platoon with vehicles at 
            %steady-state, that is, all at the same speed at their desired 
            % distances
            obj.createVehicles(v0, maxVel, simTime, vehFollCtrlParams, ...
                velCtrlPoles, tau, 1, 0);
        end
               
        function [] = createEquallySpacedVehArray(obj, v0, maxVel, ...
                simTime, polesACC, rho)
            obj.createVehicles(v0, maxVel, simTime, polesACC, 0, rho, 0);
        end
        
        function [] = createRandomlySpacedVehArray(obj, v0, simTime, ...
                polesACC, lambdaExtraGap)
            obj.createVehicles(v0, maxVel, simTime, polesACC, ...
                0, 1, lambdaExtraGap);
        end
        
        function [] = resetStates(obj)
            if obj.hasSpecialLeader
                obj.leader.resetStates();
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n).resetStates();
            end
        end
        
        function [] = setLateralPosition(obj, y)
            for n = 1:length(obj.vehs)
                obj.vehs(n).y = y;
            end
        end
               
        function [] = singleStepUpdate(obj)
                        
%             q = zeros(obj.vehs(1).nStates, length(obj.vehs));
            for n = length(obj.vehs):-1:1 % order is very important
                obj.vehs(n).singleStepUpdate();
            end
            
        end
        
        function [] = simulateOverTime(obj)
            %Simulates the vehicle array movement over the entire
            %simulation time of its vehicles.
            
            simTime = obj.vehs(1).simTime;
            for k = 2:length(simTime)
                obj.singleStepUpdate();
            end
            
        end
        
        function [] = simulateWithConstantSpeed(obj)
            %Simulates the vehicle array movement over the entire
            %simulation time of its vehicles assuming all of them keep
            %their initial speed
            simTime = obj.vehs(1).simTime;
            for n = 1:obj.nv
                obj.vehs(n).x = obj.vehs(n).x0 + obj.vehs(n).vx0*simTime;
                obj.vehs(n).vx = obj.vehs(n).vx0;
                obj.vehs(n).ax =0;
                obj.vehs(n).u = 0;
            end
        end
        
        function [u] = computeInputs(obj, controllerParams)

            u = zeros(length(obj.vehs), 1);
            if nargin>1
                u(1) = obj.vehs(1).computeInput(controllerParams);
                n0 = 2;
            else
                n0 = 1;
            end
            
            for n = n0:length(obj.vehs)
                u(n) = obj.vehs(n).computeInput();
            end
        end
        
        function [leader, vehArray] = splitLeader(obj)
            leader = obj.vehs(1);
            vehArray = LongVehicleArray(obj.nv-1);
            vehArray.vehs = obj.vehs(2:obj.nv);
        end
        
        function [affectedVeh, accelCost, timeToSS] = estimateAdjLaneCosts(obj, ...
                gamma, adjLaneVehArray)
            if obj.hasSpecialLeader
                [affectedVeh, accelCost, timeToSS] = obj.leader.estimateAdjLaneCosts(...
                    gamma, adjLaneVehArray, obj.len);
            else
                [affectedVeh, accelCost, timeToSS] = obj.vehs(1).estimateAdjLaneCosts(...
                    gamma, adjLaneVehArray, obj.len);
            end
            
        end
       

    end
end

