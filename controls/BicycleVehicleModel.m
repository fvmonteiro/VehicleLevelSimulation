classdef BicycleVehicleModel < LongitudinalVehicleModel
    %VehicleBicycleModel Vehicle model including longitudinal and lateral
    %movement
    
    %%%% TODO: change comments accordingly after changing methods
    % In this model we control linear acceleration 'a' and steering wheel
    % angle 'delta' separately. 
    % States:
    % q = [x, y, v, theta]'
    % Inputs:
    % u = [a, delta]
    % Dynamics
    % dq = [v.cos(theta), v.sin(theta), a, v/l . tan(delta)]'
    properties
        ayBounds = [-0.67, 0.67]
        hLat = 0.5;
    end  
    properties (Access = protected)
        latPositionIdx = 2;
        orientationIdx = 4;
        currentSpeeds % xdot, ydot and thetadot in vehicle reference frame
    end
    properties (Dependent)
        latPosition
        orientation
    end

    
    methods
        function obj = BicycleVehicleModel(q0)
            %BicycleVehicleModel Construct an instance of this class
            obj.velocityIdx = 3;
            obj.nInputs = 2;
            
            if nargin>0 % User defined q0
                obj.initialState = q0;
                obj.currentState = q0;
                obj.currentSpeeds = [obj.velocity*cos(obj.orientation); 
                    obj.velocity*sin(obj.orientation); 0];
            else % Default values for q0 = [0, 0, 0, 0]'
                obj.currentState = [0;0;0;0];
            end
            
            % We assume the initial speed to be the desired one just for
            % simplicity - it can be changed during simulation
            obj.desiredVelocity = obj.velocity;
            
            % Check if model is loaded and load it otherwise
%             if ~bdIsLoaded(obj.simulinkModelName)
%                 load_system(obj.simulinkModelName);
%             end
%             
%             % Set simulink simulation parameters
%             set_param('simVehicleModel', 'StartTime', '0',...
%                 'SaveOutput', 'on', 'OutputSaveName', 'outputs', ...
%                 'ExternalInput', '[t, accel, delta]',...
%                 'SolverType', 'Fixed-Step');
%             set_param('simVehicleModel/Gain1', 'Gain', num2str(obj.m/2));
%             vehicleBlockHandle = getSimulinkBlockHandle('simVehicleModel/Vehicle Body 3DOF Single Track',true);
%             set_param(vehicleBlockHandle, 'm', num2str(obj.m), ...
%                 'a', num2str(obj.lf),'b', num2str(obj.lr))

        end
        
       
        function [leaderIdx] = findLeaderBasic(obj, myPosition, otherPositions)
            %findLeader recieves an array of vehicles, determines which are
            %on the same lane, then determines which one is longitudinally 
            % closest and makes it the leader.
            laneWidth = 3.6; % possibly make this global or pass as paramter
            
            relativeX = otherPositions(1, :) - myPosition(1);
            relativeY = otherPositions(2, :) - myPosition(2);
            closestX = inf;
            leaderIdx = -1;
            for k = 1:size(otherPositions, 2)
                if abs(relativeY(k))<(laneWidth/2) && relativeX(k)>0
                    if relativeX(k)<closestX
                        closestX = relativeX(k);
                        leaderIdx = k;
                    end
                end
            end
            
            % Future improvement: to include curvature check findLeadCar
            % function in the ACC example from Automated Driving Toolbox
            
        end
        
        function [ay] = singleStepALC(obj, controlParams, ref)
            %%% TODO: change to use current state and leader ref
            % Adaptive lateral control (same control rule as ACC)
            y = obj.latPosition;
            v = obj.velocity;
            theta = obj.orientation;
            vy = v*sin(theta);
%             ay = controlParams*(ref-y) - controlParams(1)*obj.hLat*ref(2);
            % or
            Kp = controlParams(1);
            Kd = controlParams(2);
            ay = Kp*(ref(1)-y(1)-obj.hLat*vy) + Kd*(ref(2)-vy);
            ay = max(obj.ayBounds(1), min(obj.ayBounds(2), ay));
        end
        
        function [ax, ay] = computeAccelRefs(obj, controlParams, latRef)
            ax = obj.singleStepACC();
            ay = obj.singleStepALC(controlParams, latRef);
        end
        
        function [accel, delta] = computeInputsFromAccelRef(obj, axRef, ayRef)
            %computeInputsFromAccRef: computes vehicle inputs acceleration
            %and wheel steering angle from inertial x and y acceleration
            %references
            
            v = obj.velocity;
            theta = obj.orientation;
            accel = axRef/cos(theta); % in the vehicle reference
%             accel = (cos(theta)*axRef+sin(theta)*ayRef); %actual d||v||/dt
            delta = atan2(obj.len*(ayRef - accel*sin(theta)), v^2*cos(theta));

        end
        
        function [q] = singleStepUpdate(obj, inputs, sampling, q0)
            %singleStepUpdate computes system states after one step given
            %inputs accel and delta; the optional parameter q0 can be used 
            %for testing purposes
            
            % States
            if nargin<4
                x = obj.position;
                y = obj.latPosition;
                v = obj.velocity;
                theta = obj.orientation;
            else
                x = q0(obj.positionIdx);
                y = q0(obj.latPositionIdx);
                v = q0(obj.velocityIdx);
                theta = q0(obj.orientationIdx);
            end
             
            % Inputs
            accel = inputs(1);
            delta = inputs(2);
            % Compute back the references? (probably not yet a good model
            % representation) to compute the true velocity updates
            
            % Increments
            dx = v*cos(theta)*sampling;
            dy = v*sin(theta)*sampling;
            dv = accel*sampling;
            dTheta = v/obj.len*tan(delta)*sampling;
            % Return vectors
            q = [x+dx; y+dy; v+dv; theta+dTheta];
            obj.currentState = q;
        end
        
        %%% NOT A GOOD APPROACH - too slow
%         function [states] = singleStepSimulinkUpdate(obj, inputs, sampling, q0)
%             %singleStepSimulinkeUpdate uses the simulink block Vehicle Body
%             %3DOF Single Track to compute the vehicle's state given input
%             %accel.
%             
%             if nargin<4
%                 x0 = obj.position + obj.lr;
%                 y0 = obj.latPosition;
%                 v0 = obj.velocity;
%                 theta0 = obj.orientation;
%             else
%                 x0 = q0(obj.positionIdx) + obj.lr;
%                 y0 = q0(obj.latPositionIdx);
%                 v0 = q0(obj.velocityIdx);
%                 theta0 = q0(obj.orientationIdx);
%             end
%             
%             % Set simulink model paramters            
%             xdot0 = obj.currentSpeeds(1);
%             ydot0 = obj.currentSpeeds(2);
%             thetadot0 = obj.currentSpeeds(3);
%             t = 0;
%             accel = inputs(1);
%             delta = inputs(2);
%             set_param('simVehicleModel', 'StopTime', num2str(sampling), ...
%                 'FixedStep', num2str(sampling));
%             vehicleBlockHandle = getSimulinkBlockHandle('simVehicleModel/Vehicle Body 3DOF Single Track',true);
%             % Set proper initial conditions
%             set_param(vehicleBlockHandle, ...
%                 'X_o', num2str(x0), 'xdot_o', num2str(xdot0),...
%                 'Y_o', num2str(y0), 'ydot_o', num2str(ydot0),...
%                 'psi_o', num2str(theta0), 'r_o', num2str(thetadot0));
%             
%             % Run simulink model
%             simOut = sim('simVehicleModel','ReturnWorkspaceOutputs','on', ...
%                 'SrcWorkspace','current');
%             % Get states of our model
%             x = simOut.outputs{obj.positionIdx}.Values.Data(2);
%             y = simOut.outputs{obj.latPositionIdx}.Values.Data(2);
%             v = simOut.outputs{obj.velocityIdx}.Values.Data(2);
%             theta = simOut.outputs{obj.orientationIdx}.Values.Data(2);
%             obj.currentState = [x-obj.lr; y; v; theta];
%             % Get speeds to properly reinitalize the model next iteration
%             xdot = simOut.outputs.get('xdot').Values.Data(2);
%             ydot = simOut.outputs.get('ydot').Values.Data(2);
%             thetadot = simOut.outputs.get('omega').Values.Data(2);
%             obj.currentSpeeds = [xdot; ydot; thetadot];
%             
%             % Function return value
%             states = obj.currentState;
%         end
        
        function value = get.latPosition(obj)
            value = obj.currentState(obj.latPositionIdx);
        end
        
        function value = get.orientation(obj)
            value = obj.currentState(obj.orientationIdx);
        end
           
        
        %%% FUNCTIONS BELOW THIS COMMENT WILL BE DELETED IN THE NEAR FUTURE
        function [q, u] = singleStepUpdateOLD(obj, controlType, controlParams, sampling, ref)
            %singleStepMovement computes states one step ahead given the
            %type of applied control, reference, and controller parameters
            
%             if nargin==5
%                 % References
%                 xRef = ref(1);
%                 vxRef = ref(2);
%                 yRef = ref(3);
%                 vyRef = ref(4);
%             end
            
            % States
            x = obj.position;
            y = obj.latPosition;
            v = obj.velocity;
            theta = obj.orientation;
            
            switch controlType
                case 'Longitudinal State Feedback'
                    K = controlParams;
                    xRef = ref(1);
                    vxRef = ref(2);
                    % TODO: change to use existing function
                    accel = K*([xRef;vxRef] - [x;v]);
                    accel = max(obj.acc_bounds(1), min(obj.acc_bounds(2), accel));
                    delta = 0;
                case 'ACC'
                    Kacc = controlParams;
                    accel = obj.singleStepACC(Kacc);
                    delta = 0;
                case 'Longitudinal and Lateral'
                    Kacc = controlParams(1:2);
                    Kp = controlParams(3);
                    Kd = controlParams(4);
                    %%%% WON'T WORK WITH MORE COMPLICATED STRATEGIES
                    yRef = ref(1);
                    vyRef = ref(2);
                    ayRef = obj.singleStepALC([Kp Kd], ...
                        [yRef; vyRef]);
                    accel = obj.singleStepACC(Kacc);
                    delta = atan2(obj.len*(ayRef - accel*sin(theta)), v^2*cos(theta));
                otherwise
                    error([sprintf('Not a valid controller choice. Try: \n')... 
                        sprintf('Longitudinal State Feedback \n ACC \n')...
                        sprintf('Longitudinal and Lateral.')])
            end
            
            % Increments
            dx = v*cos(theta)*sampling;
            dy = v*sin(theta)*sampling;
            dv = accel*sampling;
            dTheta = v/obj.len*tan(delta)*sampling;
            % Return vectors
            u = [accel; delta];
            q = [x+dx; y+dy; v+dv; theta+dTheta];
            obj.currentState = q;

        end
                
        function [q, u] = computeMovement(obj, x_ref, vx_ref, y_ref, vy_ref, Kacc, Kp, Kd,  t)
            %compute_movement Summary of this method goes here
            %   Detailed explanation goes here
            
            warning('This function will be removed')
            controlType = 'Longitudinal and Lateral';
            
            % Setting proper dimensions (to make calling the function
            % easier)
            x_ref = obj.scalarToVector(x_ref, length(t));
            y_ref = obj.scalarToVector(y_ref, length(t));
            vx_ref = obj.scalarToVector(vx_ref, length(t));
            vy_ref = obj.scalarToVector(vy_ref, length(t));
            
            sampling = t(2)-t(1);
            
            q = zeros(length(obj.initState), length(t));
            q(:, 1) = obj.initState;
%             ay_ref = zeros(length(t), 1);
            u = zeros(obj.nInputs, length(t));
            
            for k = 2:length(t)
                [q(:, k), u(:, k-1)] = obj.bicycleSingleStepUpdate(controlType, ...
                    [x_ref(k-1), vx_ref(k-1), y_ref(k-1), vy_ref(k-1)],...
                    [Kacc, Kp, Kd], sampling);
            end
            % Last input (just for plotting, doesn't go in the system)
            [~, u(:, k)] = obj.bicycleSingleStepUpdate(controlType, ...
                [x_ref(k), vx_ref(k), y_ref(k), vy_ref(k)],...
                [Kacc, Kp, Kd], sampling);
            
            q = q';
            u = u';
        end
        
    end
end

