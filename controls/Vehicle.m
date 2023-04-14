classdef (Abstract) Vehicle < handle
    
    properties (Constant, Hidden = true)
        mpsToKmph = 3.6;
        minTau = 0.01 % actuator delay
        namesThatChangeForPlot = {'Fd', 'Fo', 'Ld', 'Lo'}
    end
    
    properties
        name
        plotName % name to appear in plot legends
        type % passenger vehicle (PV) or heavy-duty vehicle (HDV)
        
        % TODO: leader should be moved to SetAccess = protect and there 
        % should be a method setLeader (not sure if on parent or derived
        % classes) to be sure time headway is updated when a leader is set
        leader % preceding vehicle
        %         virtualLeader
        
        % Vehicle following parameters:
        h % time headway for ACC given difference in decelerations and
        % assumed difference in speeds and max speed
        d0 % distance at standstill for ACC
        
        % Parameters taking into account reduced braking during lane change
        hLC
        d0LC
        
        % Other Params
        tau % actuator lag
        reactionTime = 0.2 % time to start emergency braking
        desiredVelocity
%         initialState
        
        % Storing simulation results
        simTime % simulation time
        inputs
        states % matrix to store the vehicle states obtained in simulation
        
        % Input constraints
        accelBounds
        jerkBounds
        comfAccelBounds
        comfJerkBounds
        accelBoundsDuringLC
    end
    
    properties (SetAccess = protected)
        controller
    end
    
    properties (SetAccess = protected, Hidden = true)
        % System matrices
        A
        B
        C
        D
        
        % Vehicle Parameters
        m
        len
        width
        
        % Structs to allow access by name to the states and inputs of the 
        % vehicle
        statesIdx = struct()
        inputsIdx = struct()

        lambda1 % time headway of the exact min gap solution assuming equal
        % decelerations and speeds
        lambda1LC % same as above but during lane change
        lambda2 % constant term of the exact min gap solution assuming equal
        % decelerations and speeds
        lambda2LC % same as above but during lane change
    end
    
    properties (Dependent)
        nStates
        nInputs
        minVFGap0
        
%         worstCaseBrakeLeading
%         worstCaseBrakeFollowing
    end
    
    properties (Abstract, Dependent)
        x0
        vx0
        x
        vx
        u
    end
    
    methods
        
        function [] = setVehicleParams(obj, vehType)
            if ~any(contains(VehicleTypes.possibleTypes, vehType))
                error('Invalid vehicle type')
            end
            
            obj.type = vehType;
            obj.m = VehicleTypes.m.(vehType);
            obj.len = VehicleTypes.len.(vehType);
            obj.width = VehicleTypes.width.(vehType);
            obj.tau = VehicleTypes.tau.(vehType);
            obj.accelBounds = VehicleTypes.accelBounds.(vehType);
            obj.comfAccelBounds = VehicleTypes.comfAccelBounds.(vehType);
            obj.accelBoundsDuringLC = ...
                VehicleTypes.accelBoundsDuringLC.(vehType);
            obj.jerkBounds = VehicleTypes.jerkBounds.(vehType);
            obj.comfJerkBounds = VehicleTypes.comfJerkBounds.(vehType);
        end
        
        function [lambda1, lambda2] = safeHeadwayParams(obj)
            %safeHeadwayParams computes the minimum safe following distance
            %considering the case where the leader brakes with maximum 
            % force and main vehicle is travelling with maximum acceleration
            % Code expects: maxBrake>0 and maxJerk>0
            
            maxBrake = abs(min(obj.accelBounds));
            maxAccel = max(obj.comfAccelBounds);
            maxJerk = abs(min(obj.jerkBounds));
            delay = obj.reactionTime;
            
            [lambda1, lambda2] = obj.computeHeadwayParams(delay, ...
                maxAccel, maxBrake, maxJerk);
            obj.lambda1 = lambda1;
            obj.lambda2 = lambda2;
            
        end
        
        function [lambda1LC, lambda2LC] = safeHeadwayParamsDuringLC(obj)
            %safeHeadwayParamsDuringLC same as safeHeadwayParams but uses
            %lane change maximum deceleration parameters
            % Code expects: maxBrake>0 and maxJerk>0
            
            maxBrake = abs(min(obj.accelBoundsDuringLC));
            maxAccel = max(obj.accelBoundsDuringLC);
            maxJerk = abs(min(obj.jerkBounds));
            delay = obj.reactionTime;
            
            [lambda1LC, lambda2LC] = obj.computeHeadwayParams(delay, ...
                maxAccel, maxBrake, maxJerk);
            obj.lambda1LC = lambda1LC;
            obj.lambda2LC = lambda2LC;
            
        end
        
        function [timeHeadway, constantTerm] = computeTimeHeadway(obj, ...
                freeFlowVel, rho, isLaneChanging)
            %timeHeadwayMethod1 Defines time headway based on difference of
            % braking capabilities between the vehicle and its leader, the
            % free flow speed and the proportional maximum relative speed
            % freeFlowVel: free flow velocity in m/s
            % rho: defined as (vE - vL) < rho vE
                        
            if ~isempty(obj.leader) % a leader was already assigned
                leaderMaxBrake = abs(min(obj.leader.accelBounds));
%                 if nargin<3 % no expected velocity ratio provided
%                         rho = 1-obj.leader.vx0/freeFlowVel;
%                 end
            else % no leader found
                if ~strcmpi(obj.name(1), 'L')
                    warning([obj.name ' is not a leading vehicle and '...
                        'it was not assigned a leader'])
                end
                leaderMaxBrake = abs(min(obj.accelBounds));
                freeFlowVel = obj.vx0;
                rho = 0;
            end
            
            if nargin > 3 && isLaneChanging 
                maxBrake = abs(min(obj.accelBoundsDuringLC));
                param1 = obj.lambda1LC;
                param2 = obj.lambda2LC;
            else
                maxBrake = abs(min(obj.accelBounds));
                param1 = obj.lambda1;
                param2 = obj.lambda2;
            end
            
            gamma = leaderMaxBrake/maxBrake;
            gammaThreshold = (1-rho)*freeFlowVel/(freeFlowVel+param1);
            
            if gamma<=gammaThreshold
                timeHeadway = rho/maxBrake/(1-gamma)...
                    *(rho*freeFlowVel/2+param1);
                constantTerm = param1^2/(2*maxBrake*(1-gamma)) ...
                    + param2;
            elseif gamma>=(1-rho)^2
                timeHeadway = param1/maxBrake + ...
                    (gamma-(1-rho)^2)*freeFlowVel/(2*maxBrake*gamma);
                constantTerm = param1^2/2/maxBrake + param2;
            else
                timeHeadway = param1/maxBrake;
                constantTerm = param1^2/2/maxBrake + param2;
            end
            
            if nargin > 3 && isLaneChanging 
                obj.hLC = timeHeadway;
                obj.d0LC = max(constantTerm, obj.d0);
            else
                obj.h = timeHeadway;
                obj.d0 = constantTerm;
            end
            
        end
        
        function[hr] = computeTimeHeadwayWithRisk(obj, acceptedRisk, rho, ...
                isLaneChanging, maxLaneChangeVel)
            %computeTimeHeadwayWithRisk Computes the time headway which,
            %in the worst-case scenario, leads to a collision severity
            %no higher thant the accepted risk
            
            if isempty(obj.desiredVelocity)
                error([obj.name ': risky headway cannot be computed '...
                    'without first defining the desired speed.'])
            end
            if isempty(obj.h)
                error([obj.name ': risky headway cannot be computed '...
                    'without first defining the safe time headway.'])
            end
            
            if nargin > 3 && isLaneChanging
                maxVel = maxLaneChangeVel;
                maxBrake = abs(min(obj.accelBounds));
                param1 = obj.lambda1LC;
                relevantH = obj.hLC;
            else
                maxVel = obj.desiredVelocity;
                maxBrake = abs(min(obj.accelBounds));
                param1 = obj.lambda1;
                relevantH = obj.h;
            end
            
            leaderMaxBrake = abs(min(obj.leader.accelBounds));
            gamma = leaderMaxBrake/maxBrake;
            threshold = (1-rho)*maxVel/(maxVel+param1);
            if gamma > threshold
                hr = relevantH - (acceptedRisk).^2 ...
                    /(2*maxVel*maxBrake);
            else
                hr = relevantH - (acceptedRisk).^2 ...
                    /(2*maxVel*maxBrake*(1-gamma));
            end
        end
        
        function [hLC, d0LC] = computeTimeHeadwayDuringLC(obj, lcVel)
            %timeHeadwayDuringLCMethod1 Defines time headway based on 
            %expected difference of max braking from ego and leading 
            %vehicles and on ego vehicle's expected maximum velocity
            % gamma: ego.maxBrake = gamma x leader.maxBrake
            
            % See notes for timeHeadwayMethod1()
            warning(['[May 17 2021] Outdated function. Use '...
                'computeTimeHeadway with parameter isLaneChanging instead'])
            
            % Set some default paramters for ease of use:
            if ~isempty(obj.leader) % a leader was already assigned
                leaderMaxBrake = abs(min(obj.leader.accelBounds));    
            else % no leader found
%                 warning('no leader provided to adjust time headway')
                leaderMaxBrake = abs(min(obj.accelBounds));
                lcVel = obj.initialState(obj.statesIdx.vx);
            end
            
            % During lane change, we assume all vehicles have the same
            % speed
            rho = 1;
            
            maxBrake = abs(min(obj.accelBoundsDuringLC));
            
            gamma = maxBrake/leaderMaxBrake;
            hLC = obj.lambda1LC + (1-gamma*rho^2)*lcVel/(2*maxBrake);
            obj.hLC = max(hLC, obj.lambda1LC);
            d0LC = max(obj.lambda2, 0.5);
            obj.d0LC = d0LC;
        end
                
        function [gap] = computeGap(obj, otherVeh)
            % Computes the gap from this vehicle to its leader or to any
            % other optionally provided vehicle
            if nargin<2
                if isempty(obj.leader)
                    gap = zeros(length(obj.simTime), 1);
                    return
                end
                otherVeh = obj.leader;
            end
            gap = otherVeh.x - otherVeh.len - obj.x ;
        end
        
        function [errors] = computeErrors(obj, errorType, otherVeh)
            % Computes the errors from this vehicle to its leader (default) 
            % or to any other provided vehicle using the vehicle following
            % gap as reference, i.e., h.vx+d0
            % @errorType: string 'gap' or 'velocity'
            % @otherVeh: object of Vehicle type
            
            minGap = obj.h*obj.vx + obj.d0;
            if nargin<3
                if isempty(obj.leader)
                    eHeadway = zeros(length(obj.simTime), 1);
                    eVel = obj.desiredVel - obj.vx;
                    errors = [eHeadway, eVel];
                    return
                end
                otherVeh = obj.leader;
            end
            eHeadway = obj.computeGap(otherVeh) - minGap;
            eVel = otherVeh.vx - obj.vx;
            errors = [eHeadway, eVel];
            
            if nargin>1
                if strcmpi(errorType, 'gap')
                    errors = eHeadway;
                elseif strcmpi(errorType, 'velocity')
                    errors = eVel;
                else
                    error('Unkown type of error')
                end
            end
        end
        
        %%% CONTROLLERS SETTERS %%%
        function [] = setController(obj, controlType, controlParams)
            obj.controller = Controller(obj, controlType);
            if strcmp(controlType, 'OpenLoop')
                obj.controller.setInput(controlParams);
            else
                obj.controller.setGains(controlParams);
            end
        end
        
        function [] = setVelController(obj, velCtrlPoles)
            obj.controller.setVelControlGains(velCtrlPoles);
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
        
        %%% SETTERS %%%
        function [] = set.accelBounds(obj, bounds)
            if bounds(1)>bounds(2)
                error(['First element (lower bound) must be smaller than '...
                    'second element (upper bound)']);
            end
            obj.accelBounds = bounds;
        end
        
        function [] = set.jerkBounds(obj, bounds)
            if bounds(1)>bounds(2)
                error(['First element (lower bound) must be smaller than '...
                    'second element (upper bound)']);
            end
            obj.jerkBounds = bounds;
        end
        
        function [] = set.tau(obj, newTau)
            obj.tau = max(newTau, obj.minTau);
        end
        
%         function [] = set.x0(obj, value)
%             obj.initialState(obj.statesIdx.x) = value;
%         end
        
        %%% GETTERS %%%
        function value = get.nStates(obj)
            value = length(fieldnames(obj.statesIdx));
        end
        
        function value = get.nInputs(obj)
            value = length(fieldnames(obj.inputsIdx));
        end
        
%         function value = get.x0(obj)
%             value = obj.initialState(obj.statesIdx.x);
%         end
%         
%         function value = get.y0(obj)
%             value = obj.initialState(obj.statesIdx.y);
%         end
%         
%         function value = get.vx0(obj)
%             value = obj.initialState(obj.statesIdx.vx);
%         end
        
        function value = get.minVFGap0(obj)
            value = obj.d0 + obj.h*obj.vx0;
        end
%         
%         function value = get.vMin(obj)
%             value = obj.initialState(obj.statesIdx.vx)*0.65;
%         end
        
        function value = get.plotName(obj)
            if isempty(obj.plotName)
                if any(strcmpi(obj.namesThatChangeForPlot, obj.name))
                    value = [lower(obj.name(1)) '_' lower(obj.name(2))];
                else
                    value = obj.name;
                end
            else
                value = obj.plotName;
            end
        end
        
        
        %%% PLOT FUNCTIONS %%%
        function fig = plotStates(obj, statesToPlot, varargin)
            if ischar(statesToPlot)
                statesToPlot = {statesToPlot};
            end
            eIdx = find(contains(statesToPlot, 'errors'), 1);
            if ~isempty(eIdx)
                % include both errors in stateCell
                tempCell = cell(length(statesToPlot)+1, 1);
                tempCell(1:eIdx-1) = statesToPlot(1:eIdx-1);
                tempCell{eIdx} = 'eGap';
                tempCell{eIdx+1} = 'eVelocity';
                tempCell(eIdx+2:end) = statesToPlot(eIdx+1:end);
                statesToPlot = tempCell;
            end
            
            nPlottedStates = length(statesToPlot);
            t = obj.simTime;
            
            [fig, otherVehs, lineSpecs] = obj.sortPlotArgs(varargin{:});
            stateLabels = cell(nPlottedStates, 1);
            for n = 1:nPlottedStates
                ax = subplot(nPlottedStates, 1, n);
                grid on; hold on;
                
                oldLeg = '';
                if ~isempty(ax.Legend)
                    oldLeg = ax.Legend.String;
                end
                
                switch lower(statesToPlot{n})
                    case {'long position', 'position', 'x'}
                        stateLabels{n} = 'x';
                        plottedVar = obj.x;
                        legStr = obj.plotName;
                        unit = 'm';
                    case {'long velocity', 'velocity', 'vx'}
                        stateLabels{n} = 'v_x';
                        plottedVar = obj.vx*obj.mpsToKmph;
                        legStr = obj.plotName;
                        unit = 'km/h';
                    case {'long acceleration', 'acceleration', 'ax'}
                        stateLabels{n} = 'a_x';
                        plottedVar = obj.ax;
                        legStr = obj.plotName;
                        unit = 'm/s^2';
                    case {'lat position', 'y'}
                        stateLabels{n} = 'y';
                        plottedVar = obj.y;
                        legStr = obj.plotName;
                        unit = 'm';
                    case {'lat velocity', 'vy'}
                        error('Lateral velocity is not (yet) an output')
                    case {'u', 'input'}
                        stateLabels{n} = 'u';
                        plottedVar = obj.u;
                        legStr = obj.plotName;
                        unit = 'm/s^2';
                    case 'delta'
                        stateLabels{n} = '\varphi';
                        plottedVar = obj.delta;
                        legStr = obj.plotName;
                        unit = 'rad';
                    case 'gap'
                        stateLabels{n} = 'gap';
                        plottedVar = zeros(length(t), length(otherVehs));
                        legStr = cell(length(otherVehs), 1);
                        for k = 1:length(otherVehs)
                            plottedVar(:, k) = obj.computeGap(otherVehs(k));
                            legStr{k} = [obj.plotName ' to ' 
                                otherVehs(k).plotName];
                        end
                        unit = 'm';
                    case {'eg', 'egap'}
                        stateLabels{n} = 'e_g';
                        plottedVar = zeros(length(t), length(otherVehs));
                        legStr = cell(length(otherVehs), 1);
                        for k = 1:length(otherVehs)
                            plottedVar(:, k) = obj.computeErrors('gap', ...
                                otherVehs(k));
                            legStr{k} = [obj.plotName ' to ' ...
                                otherVehs(k).plotName];
                        end
                        unit = 'm';
                    case {'ev', 'evelocity'}
                        stateLabels{n} = 'e_v';
                        plottedVar = zeros(length(t), length(otherVehs));
                        legStr = cell(length(otherVehs), 1);
                        for k = 1:length(otherVehs)
                            plottedVar(:, k) = obj.computeErrors(...
                                'velocity', otherVehs(k));
                            legStr{k} = [obj.plotName ' to ' ...
                                otherVehs(k).plotName];
                        end
                        plottedVar = plottedVar*obj.mpsToKmph;
                        unit = 'km/h';
                    case 'ey'
                        stateLabels{n} = 'e_Y';
                        if isa(obj,'SimulinkVehicle')
                            plottedVar = obj.simErrors.ey;
                        else
                            warning(['This Vehicle class does not contain '...
                                'the lateral error property'])
                            plottedVar = zeros(length(t), 1);
                        end
                        legStr = obj.plotName;
                        unit = 'm';
                    otherwise
                        error('Unknown requested state');
                end
                
                plot(t, plottedVar, 'LineWidth', 1.5, lineSpecs{:}); grid on;
                if n == nPlottedStates
                    xlabel('time [s]');
                end
                xlim([0, ceil(t(end))]);
                ylabel(['$' stateLabels{n} '[' unit ']$'], ...
                    'Interpreter', 'latex');
                ymin = min([plottedVar(:); ax.YLim(1)]);
                ymax = max([plottedVar(:); ax.YLim(2)]);
                ylim([ymin-0.05*(ymax-ymin) ymax+0.05*(ymax-ymin)]);
                lgd = legend([oldLeg(:); legStr(:)], 'Location', 'best');
                ax.FontSize = 12;
                ax.Tag = stateLabels{n};
                lgd.FontSize = 11;
            end
            
        end
        
    end
    
    methods (Access = protected)
        
        function [lambda1, lambda2] = computeHeadwayParams(~, delay, ...
                accel, brake, jerk)
            tauJ = (accel+brake)/jerk;
            
%             lambda1 = delay + tauJ ...
%                 + 1/brake*(accel*delay + accel*tauJ-1/2*jerk*tauJ^2);
%             lambda2 = 1/2*accel*delay^2 + accel*delay*tauJ + 1/2*accel*tauJ^2 ...
%                 - 1/6*jerk*tauJ^3 ...
%                 + 1/(2*brake)*(accel*delay + accel*tauJ - 1/2*jerk*tauJ^2)^2;
            
            lambda1 = (accel + brake)*(delay + tauJ/2);
            lambda2 = -(accel+brake)/2*(delay^2 + delay*tauJ+tauJ^2/3);
        end
        
        function [fig, otherVehs, lineSpecs] = sortPlotArgs(~, varargin)
            lineSpecs = varargin;
            emptyIdx = false(length(lineSpecs), 1);
            otherVehs = [];
            fig = [];
            for n = 1:length(varargin)
                if any(isgraphics(varargin{n})) && ~isnumeric(varargin{n})
                    if ~isempty(fig)
                        warning(['Possible varargin parsing error:' ...
                            'it finds two figures in the arguments'])
                    end
                    emptyIdx(n) = true;
                    fig = varargin{n};
                elseif any(isa(varargin{n}, 'Vehicle'))
                    emptyIdx(n) = true;
                    otherVehs = varargin{n};
                elseif isempty(varargin{n})
                    emptyIdx(n) = true;
                end
            end
            
            if isempty(fig)
                fig = figure;
            end
            lineSpecs(emptyIdx) = [];
            
        end
        
    end
    
end