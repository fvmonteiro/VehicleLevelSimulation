classdef ACCPlatoon < LongVehicleArray
    %ACCPLATOON Platoon of Vehicles with longitudinal control only
    %   Detailed explanation goes here
       
    methods
        function obj = ACCPlatoon(nv)
            %ACCPLATOON Construct an instance of this class
            %   We just create an array of vehicles and set the special
            %   leader
            obj = obj@LongVehicleArray(nv-1);

            obj.hasSpecialLeader = true;
            
%             if nv>1
%                 obj.vehs = LongitudinalVehicleModel();
%                 obj.vehs(nv-1, 1) = LongitudinalVehicleModel();
%             else
%                 warning("[ACCPlatoon] Single vehicle platoon.")
%             end

        end
        
        function [] = createHomogeneousSSPlatoon (obj, v0, maxVel, simTime, polesACC)
           % Create a platoon with homogeneous vehicles at steady-state,
           % that is, all at the same speed at their desired distances
           createHomogeneousSSPlatoon@LongVehicleArray(obj, v0, maxVel, simTime, polesACC);
           
           if size(polesACC, 2)>2 % 2 poles -> PD ; 3 poles -> PID
               ctrType = 'ACC PID';
           else
               ctrType = 'ACC PD';
           end
%            nf = length(obj.vehs); % followers: nv-1
%            
%            if size(polesACC, 1)==1 % only one set of poles for the whole platoon
%                polesACC = repmat(polesACC, obj.nv, 1);
%            end
%            
%            initState = [0; v0];
%            % All vehicle gaps are initialized at the ACC steady state
%            for n = 1:nf
%                obj.vehs(n) = LongitudinalVehicleModel(initState);
%                obj.vehs(n).setController('ACC PD', deltaT, polesACC(n, :));
%                ssGap = obj.vehs(n).minACCGap + obj.vehs(n).len;
%                initState = initState + [ssGap; 0];
%            end
%            obj.vehs = flip(obj.vehs);
           
           % Create the platoon special leader
           obj.leader = CoopPlatoonLeader(simTime, obj.vehs);
           % Assign the special leader to the first vehicle in the array
           obj.vehs(1).leader = obj.leader;
           obj.vehs(1).adjustTimeHeadway(maxVel, obj.velRatio);
           obj.vehs(1).setController(ctrType, polesACC(1, :));
           
           
           leaderInitState = [obj.vehs(1).position + obj.vehs(1).minACCGap + obj.vehs(1).len; ...
               v0];
           Ks = obj.vehs(1).controller.K(1);
           Kv = obj.vehs(1).controller.K(2);
           obj.leader.includeInSimulation(leaderInitState, Ks, Kv)
           
           
           
%            for n = 2:nf
%                obj.vehs(n).leader = obj.vehs(n-1);
%            end
        end
        
        function [u] = computeInputs(obj, controllerParams)

            u = zeros(obj.nv, 1);
            if nargin>1
                u(1) = obj.leader.computeInput(controllerParams);
            else
                u(1) = obj.leader.computeInput();
            end
            
            u(2:end) = computeInputs@LongVehicleArray(obj);
%             for n = 2:obj.nv
%                 accel(n) = obj.vehs(n-1).computeInput();
%             end

        end
        
        function [q] = singleStepUpdate(obj, reference)

            q = zeros(obj.vehs(1).nStates, obj.nv);
            if nargin>1
                obj.leader.singleStepUpdate(reference);
            else
                obj.leader.singleStepUpdate();
            end
            q(:, 1) = [obj.leader.position; obj.leader.velocity];
            q(:, 2:end) = singleStepUpdate@LongVehicleArray(obj);
            
        end
       
    end
end

