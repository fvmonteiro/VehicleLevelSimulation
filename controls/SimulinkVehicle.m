classdef SimulinkVehicle < Vehicle
    %SimulinkVehicle Class that holds ans sets parameters for the 
    %ACC-equipped vehicles in Simulink simulations
    
    properties
        % Simulation identifiers
        simulationName
        simBlockHandle % handle for this vehicle in the simulation
        
        % To speed-up simulation, we simplify some vehicles. The simple
        % vehicles do not have lateral controller or logic to generate gaps
        isSimple = false; 
        initialState
        
        % More ACC params (some in superclass)
        egMax = inf; %3; % max headway error
        egMin = -100; % min headway error
        p = 10; % velocity filter gain
        c0 = 10; % gap filter gain % not being used
        hFilterGain = 0.3;
        hFilterSaturation = inf;
%         l1 = -10; % headway error threshold in ACC-v to determine between 
        % veh following or speed control
               
        % Storing simulation results
        simErrors % for vehicles that perform lane change, it is easier to 
        % obtain the veh following errors directly from the simulink 
        % simulations. The others can be computed afterwards
        
        % Simulation parameters defining platoon lane change
        platoonLCStrategy = 'synchronous' % (default) synchronous change
        positionInPlatoon = 1 % (default) leader of a 1 vehicle platoon
        
        % States during simulation:
        % Varies a bit depending on strategy
        % 0 - Lane keeping
        % 1 - Longitudinal Adjustment (if leaderFirst: waiting for leader 
        % lane change)
        % 2 - Waiting for platoon safe gap (synchronous) or longitudinal
        % adjustment (leaderFirst). Doesn't exist for other strategies
        % 3 - Lane changing
        % 4 - Waiting to start closing gap (synchronous), doesn't exist
        % (leader first), creating gap for next vehicle (lastFirst and
        % leaderFirstInvert)
        % 5 - Closing gap
        lcState
    end
    
    properties (Hidden = true)
        % Lateral dynamic parameters
        lf = 3 % [m] distance from center of gravity to front axis
        lr = 2 % [m] distance from center of gravity to rear axis
        % Default simulink values
        Cf = 12e3*2; % times two to create model uncertainty
        Cr = 11e3*2; % times two to create model uncertainty
        Iz = 4000;
        % From: Path Planning and Cooperative Control for Automated Vehicle
        % Platoon Using Hybrid Automata
        maxDelta = 25*pi/180;
        maxDeltaDot = 9.4*pi/180;
        
        % Counter to read saved data one time step at a time
        iterCounter;
    end
    
    properties (Dependent)
        x0
        y0
        vx0
        vMin % used when generating the gap
        
        x
        y
        vx
        ax
        psi
        u
        delta
        
        currentState
    end
    
    properties (Constant)
        lcStateNameToNum = struct('laneKeeping', 0, 'laneChanging', 3)
        simStateNames = {'X', 'Y', 'Xdot', 'xddot', 'psi', 'u', 'delta'};
        simErrorNames = {'egReal', 'evReal', 'egVirtual', 'evVirtual', 'ey'};
        possibleStrategies = ["synchronous", "leaderFirst", ...
            "lastFirst", "leaderFirstInvert"];
    end
    
        
    methods
        function obj = SimulinkVehicle(name, type, simName, ...
                platoonLCStrategy, q0)
            %SimulinkVehicle create an instance of class SimulinkVehicle.
            %Parameterss strategy and q0 are optional
            
            % Set variables indices for this class
            obj.statesIdx.x = 1;
            obj.statesIdx.y = 2;
            obj.statesIdx.vx = 3;
            obj.statesIdx.ax = 4;
            obj.statesIdx.psi = 5;
            
            % Set input indices
            obj.inputsIdx.u = 1;
            obj.inputsIdx.delta = 2;

            % Initial states should define: x0, vx0 and y0.
            if nargin>0
                obj.name = char(name);
                obj.setVehicleParams(type);
                obj.iterCounter = 1;
                if strcmpi(obj.name(1), 'p')
                    % if platoon veh, we set it's position in the platoon
                    obj.positionInPlatoon = obj.name(2);
                end
                obj.simulationName = simName;
                obj.simBlockHandle = getSimulinkBlockHandle(...
                    [simName '/' name],true);
                
                obj.safeHeadwayParams();
                obj.safeHeadwayParamsDuringLC();
                
                % h variation should be slower for trucks
                if strcmpi(type, 'HDV')
                    obj.hFilterGain = obj.hFilterGain*0.5;
                end
                
                if nargin > 3
                    obj.platoonLCStrategy = platoonLCStrategy;
                end
                
                if nargin > 4
                    obj.initialState = q0;
                end                
            end

        end

        %%% GETTERS %%%
        function value = get.x0(obj)
            value = obj.initialState(obj.statesIdx.x);
        end
        
        function value = get.y0(obj)
            value = obj.initialState(obj.statesIdx.y);
        end
        
        function value = get.vx0(obj)
            value = obj.initialState(obj.statesIdx.vx);
        end
        
        function value = get.vMin(obj)
            value = obj.initialState(obj.statesIdx.vx)*0.65;
        end
        
        function value = get.x(obj)
            value = obj.states(:, obj.statesIdx.x);
        end
        
        function value = get.y(obj)
            value = obj.states(:, obj.statesIdx.y);
        end
        
        function value = get.vx(obj)
            value = obj.states(:, obj.statesIdx.vx);
        end
        
        function value = get.ax(obj)
            value = obj.states(:, obj.statesIdx.ax);
        end
        
        function value = get.psi(obj)
            value = obj.states(:, obj.statesIdx.psi);
        end
        
        function value = get.u(obj)
            value = obj.inputs(:, obj.inputsIdx.u);
        end
        
        function value = get.delta(obj)
            value = obj.inputs(:, obj.inputsIdx.delta);
        end
        
        function value = get.currentState(obj)
            if obj.iterCounter > length(obj.simTime)
                warning(['Iteration counter of simulink vehicle %s ' ...
                    'restarted'], obj.name);
                obj.iterCounter = 1;
            end
            value = obj.states(obj.iterCounter, :);
            obj.iterCounter = obj.iterCounter + 1;
        end
        
        %%% SETTERS %%%
        function [] = set.x0(obj, value)
            obj.initialState(obj.statesIdx.x) = value;
        end
        
        function [] = set.x(obj, value)
            obj.states(:, obj.statesIdx.x) = value;
        end
        
        function [] = set.y(obj, value)
            obj.states(:, obj.statesIdx.y) = value;
        end
        
        function [] = set.vx(obj, value)
            obj.states(:, obj.statesIdx.vx) = value;
        end
        
        function [] = set.ax(obj, value)
            obj.states(:, obj.statesIdx.ax) = value;
        end
        
        function [] = set.psi(obj, value)
            obj.states(:, obj.statesIdx.psi) = value;
        end
        
        function [] = set.u(obj, value)
            obj.inputs(:, obj.inputsIdx.u) = value;
        end
        
        function [] = set.delta(obj, value)
            obj.inputs(:, obj.inputsIdx.delta) = value;
        end

        
        function [] = setLatController(obj, vxLC, latParams)
            % Define linearized lateral movement matrices
            a1 = 2*(obj.Cf+obj.Cr)/obj.m;
            a2 = 2*(obj.Cf*obj.lf-obj.Cr*obj.lr)/obj.m;
            a3 = 2*(obj.Cf*obj.lf-obj.Cr*obj.lr)/obj.Iz;
            a4 = 2*(obj.Cf*obj.lf^2+obj.Cr*obj.lr^2)/obj.Iz;
            b1 = 2*obj.Cf/obj.m;
            b2 = 2*obj.Cf*obj.lf/obj.Iz;
            obj.A = [0 1 vxLC 0;
                0 -a1/vxLC 0 -vxLC-a2/vxLC;
                0 0 0 1;
                0 -a3/vxLC 0 -a4/vxLC];
            obj.B = [0; b1; 0; b2];
            obj.C = eye(4);
            %To observe Y and ang velocity:
            %obj.C = [1, 0, 5, 0; 0, 0, 1, 0; 0, 0, 0, 1];

            obj.controller.setLatControlGains(latParams);
        end
        
        %%% PROGRAMMATICALLY SET PARAMETERS IN SIMULINK MODEL %%%
        function [] = setSimParams(obj)
            % Vehicle params
            set_param(obj.simBlockHandle, ...
                'm', num2str(obj.m), ...
                'lf', num2str(obj.len/2), 'lr', num2str(obj.len/2), ...
                'maxAcceleration', num2str(obj.accelBounds(2)), ...
                'minAcceleration', num2str(obj.accelBounds(1)), ...
                'maxJerk', num2str(obj.jerkBounds(2)), ...
                'minJerk', num2str(obj.jerkBounds(1)), ...
                'freeFlowVel', num2str(obj.desiredVelocity), ...
                'tau', num2str(obj.tau), ...
                'maxComfAccel', num2str(obj.comfAccelBounds(2)), ...
                'minComfAccel', num2str(obj.comfAccelBounds(1)), ...
                'maxComfJerk', num2str(obj.comfJerkBounds(2)), ...
                'minComfJerk', num2str(obj.comfJerkBounds(1)), ...
                'x0', num2str(obj.x0), 'y0', num2str(obj.y0), ...
                'vx0', num2str(obj.vx0), ...
                'h', num2str(obj.h), 'd0', num2str(obj.d0), ...
                'hLC', num2str(obj.hLC), 'd0LC', num2str(obj.d0LC));
            
            % Longitudinal controllers params
            K = obj.controller.K;
            if contains(obj.controller.type, 'PID')
                Ki = K(1);
                Kp = K(2);
                Kd = K(3);
            else
                Ki = 0;
                Kp = K(1);
                Kd = K(2);
            end
            
            if contains(obj.controller.type, 'CACC')
                Ka = obj.controller.Ka(1);
                Kdd = obj.controller.Ka(2);
            else
                Ka = 0;
                Kdd = 0;
            end
            
            
            velKi = obj.controller.Kvel(1);
            velKp = obj.controller.Kvel(2);
            velKd = obj.controller.Kvel(3);
            
            set_param(obj.simBlockHandle, ...
                'minGapError', num2str(obj.egMin), ...
                'maxGapError', num2str(obj.egMax), ...
                'velFilterGain', num2str(obj.p), ...
                'Ki', num2str(Ki), 'Kp', num2str(Kp), 'Kd', num2str(Kd), ...
                'Kdd', num2str(Kdd), 'Ka', num2str(Ka), ... 
                'velKi', num2str(velKi), 'velKp', num2str(velKp), ...
                'velKd', num2str(velKd),...
                'vMin', num2str(obj.vMin), ...
                'hFilterGain', num2str(obj.hFilterGain), ...
                'hFilterSaturation', num2str(obj.hFilterSaturation));
            
            % Extra parameters for lane change simulation
            if ~contains(obj.simulationName, 'full_maneuver') ...
                    && ~strcmpi(obj.simulationName, 'CACC')
                error(['[UPDATE NEEDED] The vehicle block mask in this '...
                    'simulation is outdated.'])
            end
            
            if ~obj.isSimple
                set_param(obj.simBlockHandle,...
                    'posInPlatoon', num2str(obj.positionInPlatoon), ...
                    'strategyChoice', obj.platoonLCStrategy);
                obj.setSimLateralParams();
            end
        end
        
        function [] = setSimLateralParams(obj)
            
            if ~obj.isSimple
                Klat = obj.controller.Klat;
                L = obj.controller.Llat;
                
                % Lateral controller params
                set_param(obj.simBlockHandle, ...
                    'maxDelta', num2str(obj.maxDelta), ...
                    'maxDeltaDot', num2str(obj.maxDeltaDot), ...
                    'A', mat2str(obj.A), 'B', mat2str(obj.B), ...
                    'C', mat2str(obj.C), 'Klat', mat2str(Klat), ...
                    'L', mat2str(L));
            end
            
        end
        
        function [timeIdx] = findLaneChangeEndTimeIdx(obj)
            timeIdx = find(obj.lcState.data ...
                > obj.lcStateNameToNum.laneChanging, 1);
        end
        
        
        %%% PLOT FUNCTIONS %%%
        function fig = plotSimErrors(obj, errorsToPlot, varargin)
            %plotSimErrors plots the errors saved directly from the
            %simulation (only works for lane changing or gap generating
            %vehicles).
            
            if nargin<2
                % Plot all errors if no arguments
                errorsToPlot = SimulinkVehicle.simErrorNames;
            else
                if ischar(errorsToPlot)
                    errorsToPlot = {errorsToPlot};
                end
            end
            
            % We create one subplot for each type of error and plot errors
            % to real and virtual leaders on the same axis
            errorNames = {'e_g[m]', 'e_v[km/h]', 'e_y[m]'};
            % Find each error type
            idx = [contains(errorsToPlot, 'eg'); ...
                contains(errorsToPlot, 'ev'); ...
                contains(errorsToPlot, 'ey')];
            % Keep only indices of errors being plotted
            errorNames = errorNames(any(idx'));
            idx = idx(any(idx'), :);
            
            % Load data
            errors = obj.simErrors;
            errors.evReal = errors.evReal*Vehicle.mpsToKmph;
            errors.evVirtual = errors.evVirtual*Vehicle.mpsToKmph;
            
            totalAxes = size(idx, 1);
            [fig, ~, lineSpecs] = obj.sortPlotArgs(varargin{:});
            for n = 1:totalAxes 
                currentAxis = subplot(totalAxes, 1, n);
                hold on; grid on;
                
                oldLeg = '';
                if ~isempty(currentAxis.Legend)
                    oldLeg = currentAxis.Legend.String;
                end
                
                plottedVarNames = errorsToPlot(idx(n, :));
                legStr = cell(1, length(plottedVarNames));
                for k = 1:length(plottedVarNames)
                    plottedVar = obj.simErrorPostProcessing(plottedVarNames{k}); 
                    plot(obj.simTime, plottedVar, 'LineWidth', 1.5, lineSpecs{:}); 
                    
                    if isempty(plottedVarNames{k}(3:end))
                        legStr{k} = sprintf('%s', obj.plotName);
                    else
                        legStr{k} = sprintf('%s to %s leader', obj.plotName,...
                            lower(plottedVarNames{k}(3:end)));
                    end
                end
                
                if n == totalAxes
                    xlabel('time [s]');
                end
                xlim([0, ceil(obj.simTime(end))]);
                
                ylabel(errorNames{n});
                
                currentAxis.FontSize = 12;
                currentAxis.Tag = plottedVarNames{k}(1:2);
                
                lgd = legend([oldLeg legStr], 'Location', 'NE');
                lgd.FontSize = 12;
            end

        end
        
        function fig = plotManeuverStates(obj, showTime, varargin)
            %plotManeuverStates plots the evolution of maneuver states over
            % time.
            
            % State names to show in the y label
            switch obj.platoonLCStrategy
                case 'synchronous'
                    stateNames = {'lane keeping', ...
                        sprintf('%s\\newline%s', 'longitudinal', 'adjustment')...
                        sprintf('%s\\newline%s', 'waiting for', 'platoon gap'), ...
                        sprintf('%s\\newline%s', 'lateral', 'maneuver'), ...
                        sprintf('%s\\newline%s', 'waiting to', 'close gap')...
                        'gap closing'};
                    shiftIdx = 0;
                    shift = 0;
                case 'leaderFirst'
                    stateNames = {'lane keeping', ...
                        sprintf('%s\\newline%s', 'waiting leader', 'lane change')...
                        sprintf('%s\\newline%s', 'longitudinal', 'adjustment')...
                        sprintf('%s\\newline%s', 'lateral', 'maneuver'), ...
                        'gap closing'};
                    shiftIdx = 4;
                    shift = 1;
                case 'lastFirst'
                    stateNames = {'lane keeping', ...
                        sprintf('%s\\newline%s', 'longitudinal', 'adjustment')...
                        sprintf('%s\\newline%s', 'lateral', 'maneuver')...
                        sprintf('%s\\newline%s', 'creating gap for', 'next vehicle')...
                        'gap closing'};
                    shiftIdx = 3;
                    shift = 1;
                case 'leaderFirstInvert'
                    stateNames = {'lane keeping', ...
                        sprintf('%s\\newline%s', 'longitudinal', 'adjustment')...
                        sprintf('%s\\newline%s', 'lateral', 'maneuver')...
                        sprintf('%s\\newline%s', 'creating gap for', 'next vehicle')...
                        'gap closing'};
                    shiftIdx = 3;
                    shift = 1;
            end
            stateNames = strtrim(stateNames);
            
            if nargin<2
                showTime = true;
            end
            [fig, ~, lineSpecs] = obj.sortPlotArgs(varargin{:});
            
            time = obj.lcState.Time;
            states = obj.lcState.Data;
            states(states>=shiftIdx) = states(states>=shiftIdx)-shift;
            maxState = max(states);
            plotTime = time(find(states==maxState, 1, 'last')) + 5;
            
            hold on;
            grid on;
            currentAxis = findall(fig, 'type', 'axes');
            oldLeg = '';
            if ~isempty(currentAxis.Legend)
                oldLeg = currentAxis.Legend.String;
            end
            
            if ~showTime
                xticks([]);
            end
            xlim([0, max(currentAxis.XLim(2), plotTime)]);
            yticks(0:maxState);
            xlabel('time');
            
            yticklabels(stateNames);
            ylim([0, double(maxState)+0.3]);
            
            plot(time, states, lineSpecs{:});
            
            currentAxis.FontSize = 15;
            lgd = legend([oldLeg {obj.name}], 'Location', 'NW');
            lgd.FontSize = 15;
            
        end
        
        % lazy coding
        function fig = plotSimLongErrorsAndInput(obj, errorsToPlot, varargin)
            
            if nargin<2
                % Plot all errors if no arguments
                errorsToPlot = SimulinkVehicle.simErrorNames;
            else
                if ischar(errorsToPlot)
                    errorsToPlot = {errorsToPlot};
                end
            end
            
            % We create one subplot for each type of error and plot errors
            % to real and virtual leaders on the same axis
            errorNames = {'e_g[m]', 'e_v[km/h]', 'e_y[m]'};
            % Find each error type
            idx = [contains(errorsToPlot, 'eg'); 
                contains(errorsToPlot, 'ev'); 
                contains(errorsToPlot, 'ey')];
            % Keep only indices of errors being plotted
            errorNames = errorNames(any(idx'));
            idx = idx(any(idx'), :);
            
            % Load data
            errors = obj.simErrors;
            errors.evReal = errors.evReal*Vehicle.mpsToKmph;
            errors.evVirtual = errors.evVirtual*Vehicle.mpsToKmph;
            
            totalAxes = size(idx, 1) + 1;
            [fig, ~, lineSpecs] = obj.sortPlotArgs(varargin{:});
            for n = 1:totalAxes-1
                currentAxis = subplot(totalAxes, 1, n);
                hold on; grid on;
                
                oldLeg = '';
                if ~isempty(currentAxis.Legend)
                    oldLeg = currentAxis.Legend.String;
                end
                
                plottedVarNames = errorsToPlot(idx(n, :));
                legStr = cell(1, length(plottedVarNames));
                for k = 1:length(plottedVarNames)
                    plottedVar = obj.simErrorPostProcessing(plottedVarNames{k}); 
                    plot(obj.simTime, plottedVar, lineSpecs{:}); 
                    
                    if isempty(plottedVarNames{k}(3:end))
                        legStr{k} = sprintf('%s', obj.plotName);
                    else
                        legStr{k} = sprintf('%s to %s leader', obj.plotName,...
                            lower(plottedVarNames{k}(3:end)));
                    end
                end
                
                xlim([0, ceil(obj.simTime(end))]);
                
                ylabel(errorNames{n});
                
                currentAxis.FontSize = 12;
                currentAxis.Tag = plottedVarNames{k}(1:2);
                
                lgd = legend([oldLeg legStr], 'Location', 'NE');
                lgd.FontSize = 11;
            end
            
            subplot(totalAxes, 1, totalAxes);
            plottedVar = obj.u;
            legStr = obj.plotName;
            plot(obj.simTime, plottedVar, lineSpecs{:});
            grid on;
            xlim([0, ceil(obj.simTime(end))]);
            xlabel('time (s)');
            ylabel('u (m/s^2)')
            currentAxis.FontSize = 12;
            currentAxis.Tag = 'u';
            
            lgd = legend(legStr, 'Location', 'NE');
            lgd.FontSize = 11;

        end
        
        % Post-processing for plots
        function [ppError] = simErrorPostProcessing(obj, plottedVarName)
            ppError = obj.simErrors.(plottedVarName);
            if contains(plottedVarName, 'ev', 'IgnoreCase', true)
                % Velocity from m/s to km/h
                ppError = ppError*Vehicle.mpsToKmph;
            end
            % [3/18/21] Commented out because current version of simulink
            % model should output zero virtual errors by default
%             if contains(plottedVarName, 'virtual', 'IgnoreCase', true)
%                 % Set virtual errors to zero before start of adjustments
%                 % and after end of lane change (the only reason they are
%                 % non-zero in simulation is to avoid discontinuities, which
%                 % slow everything down)
%                 maneuverStartTimeIdx = find(obj.lcState.data>0, 1);
%                 maneuverEndTimeIdx = find(obj.lcState.data>4, 1);
%                 maneuverStartTime = obj.lcState.time(maneuverStartTimeIdx);
%                 maneuverEndTime = obj.lcState.time(maneuverEndTimeIdx);
%                 
%                 ppError(obj.simTime<maneuverStartTime) = 0;
%                 ppError(obj.simTime>maneuverEndTime) = 0;
%             end
            
        end
        
    end
    
        methods (Static)
        function [vehStatesBus] = createVehBusObjects()
            
                clear elems;
                
                vehStatesBus = SimulinkVehicle.createBus(...
                    [SimulinkVehicle.simStateNames, ...
                    SimulinkVehicle.simErrorNames]);
%                 vehErrorsBus = SimulinkVehicle.createBus(...
%                     SimulinkVehicle.simErrorNames);
                
                save('vehBuses', 'vehStatesBus');

        end
        
        function [bus] = createBus(elementNames)
            N = length(elementNames);
            elems(N) = Simulink.BusElement;
            
            for n = 1:N
                elems(n).Name = elementNames{n};
                elems(n).Dimensions = 1;
                elems(n).DimensionsMode = 'Fixed';
                elems(n).DataType = 'double';
                elems(n).SampleTime = -1;
                elems(n).Complexity = 'real';
            end
            
            bus = Simulink.Bus;
            bus.Elements = elems;
        end
        
    end
    
end