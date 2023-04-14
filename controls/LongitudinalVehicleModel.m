classdef LongitudinalVehicleModel < Vehicle
    %LongitudinalVehicleModel Double or triple integrator vehicle model 
    % with longitudinal controller
    properties
        y = 0
    end
    
    properties (SetAccess = protected)
        iterCounter = 1;
        currentInput
    end
       
    properties (Dependent)
        x0
        vx0
        
        x
        vx
        ax
        u
        
        minVehFollGap
        currentState
        position
        velocity
    end

    methods
        function obj = LongitudinalVehicleModel(name, type, simTime, q0, tau)
            % Set variables indices for this class
            obj.statesIdx.x = 1;
            obj.statesIdx.vx = 2;
            
            % Set input indices
            obj.inputsIdx.u = 1;
            
            % For now [03/24/21], all vehicles are passenger vehicles
            obj.setVehicleParams('PV');
            
            if nargin > 0
                obj.name = name;
                obj.setVehicleParams(type);
                obj.simTime = simTime;
                obj.safeHeadwayParams();
                if nargin > 2
                    obj.includeInSimulation(q0, tau)
                end
            else % No argument constructor to allow object array creation
%                 obj.currentState = zeros(2, 1);
%                 obj.states = zeros(2, 1);
            end
            
        end
        
        function [] = includeInSimulation(obj, q0, tau)
            %includeInSimulation sets the vehicle's initial state and other
            %related properties
%             obj.initialState = q0;
            
            obj.inputs = zeros(length(obj.simTime), 1);
            obj.states = zeros(length(obj.simTime), length(q0));
            obj.states(obj.iterCounter, :) = q0;
            
            if tau == 0 % acceleration directly controller
                obj.A = [0 1; 0 0];
                obj.B = [0; 1];
            else % including actuator dynamics
                obj.statesIdx.ax = 3;
                obj.tau = tau;
                obj.A = [0 1 0; 0 0 1; 0 0 -1/obj.tau];
                obj.B = [0; 0; 1/obj.tau];
            end
            obj.C = eye(obj.nStates);
            obj.D = zeros(obj.nInputs);
            
            obj.currentInput = 0;
            % We assume the initial speed to be the desired one just for
            % simplicity - it can be changed during simulation
            obj.desiredVelocity = obj.velocity;
            
        end
                
        function [] = resetStates(obj)
            initialState = obj.states(1, :);
            obj.states = obj.states*0;
            obj.states(1, :) = initialState;
        end
        
        function [xNew] = singleStepUpdate(obj, reference)
            %singleStepUpdate computes system states after one step given
            %input accel;

            % Compute future state given current input
            xStart = obj.currentState;
            input = obj.inputs(obj.iterCounter);
            sampling = obj.simTime(obj.iterCounter + 1) ...
                - obj.simTime(obj.iterCounter);
            xNew = xStart + (obj.A*xStart + obj.B*input)*sampling;

            % Compute next input (using current states)
            if nargin>1
                obj.computeInput(reference);
            else
                obj.computeInput();
            end
            
            % Increase counter and update states
            obj.iterCounter = obj.iterCounter + 1;
            obj.states(obj.iterCounter, :) = xNew;
        end
        
        function [u] = computeInput(obj, reference)
            if nargin>1
                u = obj.controller.singleStepInput(reference);
            else
                u = obj.controller.singleStepInput();
            end
            obj.inputs(obj.iterCounter + 1) = u;
            
        end
        
        function [] = setController(obj, controlType, controlParams)
            setController@Vehicle(obj, controlType, controlParams);
            obj.controller.createRefVelFilter();
            obj.controller.createAccelSaturationFilter();
%             obj.controller.createPosRefFilter();
        end
        
        function [affectedVeh, accelCost, timeToSS] = estimateAdjLaneCosts(obj, ...
                gamma, adjLaneVehArray, platoonLength)
            %estimateGapGenerationCosts Simulates how vehicle in current
            %and adjacent lanes will react to gap generation and possible
            %velocity changes of this vehicle
            % platoon parameter informs this vehicle about the platoon it
            % is in. If the vehicle is alone, platoon should be empty
            
            t = obj.simTime; %0:deltaT:maxTime;
            accelThreshold = 0.05;
            
            if nargin<4
                platoonLength = obj.len;
            end
            
            desiredGap = adjLaneVehArray.vehs(1).minVehFollGap + platoonLength + obj.minVehFollGap;
            
            adjLaneVehArray.vehs(1).setController('OpenLoop', []);
            minTime = adjLaneVehArray.vehs(1).controller.minTimeWithComfortConstraints(desiredGap, gamma, obj.velocity, t);
            
            k=2;
            totalAccel = 1;
            affectedVehCounter = 2;
            while (k<length(t) && totalAccel>0.05) || t(k)<minTime
                adjLaneVehArray.singleStepUpdate();
                totalAccel = sum(abs(squeeze(adjLaneVehArray.inputs(k, :))));

                %%% Adhoc solution to avoid vehicles from start
                %%% accelerating to catch up with the far away leader
                %%% without prohibiting them to accelerate later on
                if affectedVehCounter<=adjLaneVehArray.nv && adjLaneVehArray.inputs(k, affectedVehCounter) < 0
                    adjLaneVehArray.vehs(affectedVehCounter).desiredVelocity = ...
                        1.2*adjLaneVehArray.vehs(affectedVehCounter).desiredVelocity;
                    affectedVehCounter = affectedVehCounter + 1;
                end
                
                k = k + 1;
            end
            
            vehArrayInputs = squeeze(adjLaneVehArray.inputs);
            inputsWithinSimTime = vehArrayInputs(1:k, :);
            
            timeToSS = t(k);
            affectedVeh = find(max(abs(inputsWithinSimTime))>accelThreshold, 1, 'last');
            deltaT = obj.simTime(2)-obj.simTime(1);
            accelCost = sum(sum(squeeze(inputsWithinSimTime).^2)*deltaT);
            
            % Useful plots for debugging
%             adjLaneVehArray.plotStates({'errors'}, [1, 2, 3:3:adjLaneVehArray.nv]);
%             adjLaneVehArray.plotStates({'u', 'velocity'}, [1, 2, 3:3:adjLaneVehArray.nv]);
            
        end
        
        %%% GETTERS %%%
        function value = get.x0(obj)
            value = obj.x(1);
        end
        
%         function value = get.y0(obj)
%             value = obj.initialState(obj.statesIdx.y);
%         end
        
        function value = get.vx0(obj)
            value = obj.vx(1);
        end
        
        function value = get.u(obj)
            value = obj.inputs(:, obj.inputsIdx.u);
        end
        
        function value = get.x(obj)
            value = obj.states(:, obj.statesIdx.x);
        end
        
        function value = get.vx(obj)
            value = obj.states(:, obj.statesIdx.vx);
        end
        
        function value = get.ax(obj)
            try
                value = obj.states(:, obj.statesIdx.ax);
            catch
                warning(['Acceleration is not a state of this vehicle. '...
                    'Returning the vehicle''s input instead'])
                value = obj.u;
            end
        end
        
        function value = get.minVehFollGap(obj)
            if isempty(obj.h)
                h = obj.lambda1;
                d0 = obj.lambda2;
            else
                h = obj.h;
                d0 = obj.d0;
            end
            value = d0 + h*obj.states(obj.iterCounter, obj.statesIdx.vx);
        end
        
        function value  = get.currentState(obj)
            value = obj.states(obj.iterCounter, :)';
        end
        
        function value = get.position(obj)
            value = obj.states(obj.iterCounter, obj.statesIdx.x);
        end
        
        function value = get.velocity(obj)
            value = obj.states(obj.iterCounter, obj.statesIdx.vx);
        end
        
        %%% SETTERS %%%
        
        function [] = set.x0(obj, value)
            obj.states(1, obj.statesIdx.x) = value;
        end
        
        function [] = set.x(obj, value)
            obj.states(:, obj.statesIdx.x) = value;
        end
        
        function [] = set.vx(obj, value)
            obj.states(:, obj.statesIdx.vx) = value;
        end
        
        function [] = set.ax(obj, value)
            obj.states(:, obj.statesIdx.ax) = value;
        end
        
        function [] = set.u(obj, value)
            obj.inputs(:, obj.inputsIdx.u) = value;
        end
        
    end
    
    methods (Access = protected)
        function [affectedVehIdx, totalCost] = gapGenerationCosts(obj, vehArrayInputs, lastTimeIndex)
            accelThreshold = 0.05;
            
            inputsWithinSimTime = squeeze(vehArrayInputs(:, 1:lastTimeIndex, :));
            affectedVehIdx = find(max(abs(inputsWithinSimTime))>accelThreshold, 1, 'last');
            totalCost = sum(sum(squeeze(inputsWithinSimTime).^2)*obj.samplingPeriod);
        end
        
    end

end
