classdef BicyclePlatoon < VehicleArray
    %BicyclePlatoon Platoon of bicycle model vehicles
    %   Detailed explanation goes here
    
    properties (Dependent)
        currentState
    end
    
    methods
        function obj = BicyclePlatoon(nv)
            %BicyclePlatoon Construct an instance of this class
            %   We just create an array of vehicles
            if nv>1
                obj.vehs = BicycleVehicleModel();
                obj.vehs(nv, 1) = BicycleVehicleModel();
            else
                warning("[BicyclePlatoon] Single vehicle platoon?")
            end
        end
        
        function [] = createHomogeneousSSPlatoon (obj, v0, polesACC)
            % Create a platoon with homogeneous vehicles at steady-state,
            % that is, all at the same speed at their desired distances
            
            initState = [0; 0; v0; 0];
            % All vehicle gaps are initialized at the ACC steady state
%             obj.vehs(1) = BicycleVehicleModel(initState);
            for n = 1:obj.nv
                obj.vehs(n) = BicycleVehicleModel(initState);
                obj.vehs(n).setController('ACC PD', polesACC);
                ssGap = obj.vehs(n).minACCGap + obj.vehs(n).len;
                initState = initState + [ssGap; 0; 0; 0];
            end
            % Platoon leader has index 1, last vehicle has index nv
            obj.vehs = flip(obj.vehs);
            
           for n = 2:obj.nv
               obj.vehs(n).leader = obj.vehs(n-1);
           end
        end
        
        %%% NOTE: looks like this type of function should belong in the
        %%% controller class
        function [u] = computeInputs(obj, controlType, controlParams, ref)
%             Kpx = controlParams(1);
%             Kdx = controlParams(2);
            Kpy = controlParams(1);
            Kdy = controlParams(2);
            
            p0 = obj.currentState;
            accel = zeros(1, obj.nv);
            delta = zeros(1, obj.nv);
            %%%% Initially, only follow the leader strategy is coded; after
            %%%% getting this one to work, we'll figure out the others
            platoonLeader = obj.vehs(1);
            switch controlType
                case 'Longitudinal State Feedback'
                    %                     K = controlParams;
                    x = p0(platoonLeader.positionIdx, 1);
                    v = p0(platoonLeader.velocityIdx, 1);
                    xRef = ref(1);
                    vxRef = ref(2);
                    % TODO: change to use existing function
                    accel(1) = [Kpx, Kdx]*([xRef;vxRef] - [x;v]);
                    accel(1) = max(platoonLeader.acc_bounds(1), min(platoonLeader.acc_bounds(2), accel));
                    delta(1) = 0;
                case 'ACC'
                    accel(1) = platoonLeader.singleStepACC([Kpx, Kdx]);
                    delta(1) = 0;
                case 'Longitudinal and Lateral'
                    %%%% WON'T WORK WITH MORE COMPLICATED STRATEGIES
                    yRef = ref(1);
                    vyRef = ref(2);
                    [axRef, ayRef] = platoonLeader.computeAccelRefs(controlParams, [yRef; vyRef]);
                    [accel(1), delta(1)] = platoonLeader.computeInputsFromAccelRef(axRef, ayRef);
                otherwise
                    error([sprintf('Not a valid controller choice. Try: \n')...
                        sprintf('Longitudinal State Feedback \n ACC \n')...
                        sprintf('Longitudinal and Lateral.')])
            end
            
            for n = 2:obj.nv
                precedingVeh = obj.vehs(n-1);
                currentVeh = obj.vehs(n);
                yRef = precedingVeh.latPosition;
                vyRef = precedingVeh.velocity*sin(precedingVeh.orientation);
                [axRef, ayRef] = currentVeh.computeAccelRefs(controlParams, [yRef; vyRef]);
                [accel(n), delta(n)] = currentVeh.computeInputsFromAccelRef(axRef, ayRef);
            end
            
            u = [accel; delta];
            
        end
        
        function [q] = singleStepUpdate(obj, inputs, sampling)
            %singleStepUpdate function updates positions of all platoon
            %vehicles
            
            q = zeros(obj.vehs(1).nStates, obj.nv);
            for n = 1:obj.nv
                q(:, n) = obj.vehs(n).singleStepUpdate(inputs(:, n), sampling);
            end
        end
        
        function [qPlatoon, uPlatoon] = followTheLeaderTrajectory(obj, reference, controllerParams, t)
            % References
            xRef = reference(:, 1);
            vRef = reference(:, 2);
            yRef = reference(:, 3);
            vyRef = reference(:, 4);
            % Controller parameters
            Kacc = controllerParams(1:2);
            Kp = controllerParams(3);
            Kd = controllerParams(4);
            
%             nv = length(obj.vehs);
            % 3D output matrix with all vehicles' trajectories
            qPlatoon = zeros(length(t), length(obj.vehs(1).initState), obj.nv);
            % 3D output matrix with all vehicles' inputs
            uPlatoon = zeros(length(t), 2, obj.nv);
            
            % First vehicle has no longitudinal position reference
            [q, u] = obj.vehs(1).computeMovement(xRef, vRef, yRef, vyRef, [0 Kacc(2)], Kp, Kd, t);
            qPlatoon(:, :, 1) = q;
            uPlatoon(:, :, 1) = u;
            for k = 2:obj.nv
                % Update the references 
                yRef = q(:, 2);
                vRef = q(:, 3); % simplifying assumption: v ~ vx
                vyRef = vRef.*sin(q(:, 4));
                xRef = q(:, 1) - obj.vehs(k-1).len - obj.vehs(k).g0 - obj.vehs(k).tg*vRef;
                % Compute trajectory
                [q, u] = obj.vehs(k).computeMovement(xRef, vRef, yRef, vyRef, Kacc, Kp, Kd, t);
                qPlatoon(:, :, k) = q;
                uPlatoon(:, :, k) = u;
            end
        end
        
        function [state] = get.currentState(obj)
            state = [obj.vehs.currentState];
        end

    end
end

