classdef SimulationScenarios
    %SimulationScenarios Contains methods to obtain results (mostly plots)
    %regarding minimum safe distances
    % [May 8, 2020] For now the method's are simple copies of previous 
    % files. Eventually interesting to create properties and support 
    % methods to facilitate new tests and plots  
    
    properties
        saveResults = false;
        
        % Base parameters for most scenarios 
        initVel = 25; % [m/s] ; 90 km/h
        totalTime = 15; % [s] total simulation time
        laneWidth = 3.6 % [m]
        tLat = 5; % [s] time to perform lane change
        timeToBrake = struct('human', 1, 'autonomous', 0.3, 'connected', 0.3);
        timeToEmergBrake = struct('human', 1, 'autonomous', 0.3, 'connected', 0);
        gammaF = 0.85; % max decel ratio of follower
        gammaL = 1.15; % max decel ratio of leader
    end
    
    properties (SetAccess = private, Hidden)
        deltaT = 0.01; % [s] sampling time
        deltaS = 0.1; % [m] sampling space
        resultsFolder = 'numerical_results/';
        %         saveToFolder = 'C:\Users\fvall\Google Drive\Lane Change\images\';
        saveToFolder = 'G:\My Drive\Lane Change\images\';
    end
    
    properties (Dependent)
        simTime
    end
    
    properties (Constant)
        mpsToKmph = 3.6;
    end
    
    methods
        
        function figHandle = safeGapVsGamma(obj)
%             close all
            
            % Simulation time
            t = obj.simTime;
            
            % Initial states
            vE = 25; % [m/s]
            rho = 0.1;
            vL = (1-rho)*vE;
            
            % Parameters for vehicle following distance
            delay = obj.timeToBrake.connected;
            gamma = 0.8:0.01:8/3;
            maxVel = 30; % ~ 65 mph
            
            leader = VehicleLongitudinal('T', 'L', [0, vL], [delay, 0]);
            adjustedDelay = 0.1; %delay - leader.maxBrake/leader.maxJerk;
            leader.maxJerk = inf;
            ego = VehicleLongitudinal('T', 'F', [0, vE], [adjustedDelay, 0]);
            
%             collisionFreeGap = zeros(length(gamma), 1);
            collisionFreeGap = zeros(length(gamma), 1);
            vehFollGap = zeros(length(gamma), 1);
            severity = cell(1, length(gamma));
            for k = 1:length(gamma)
                leader.setNewMaxBrake(ego.maxBrake*gamma(k));

                leader.brakeFullStop(t);
                ego.brakeFullStop(t);

                ego.computeHeadway(leader, maxVel, rho);
                % Find minimum gaps
                [collisionFreeGap(k)] = ego.computeFollowingMSS(leader);
%                 [collisionFreeGap(k)] = ego.analyticalFollowingMSS(leader);
                vehFollGap(k) = ego.vehFollGap();
                
                % Simulate for different initial gaps
                initGapArray = 0:obj.deltaS:ceil(vehFollGap(k));
                severity{k} = ego.computeSeverity(leader, initGapArray);
                
            end
            
            maxLength = length(severity{end});
            for k = 1:length(gamma)
                severity{k}(end:maxLength) = 0;
            end
            
            gammaThreshold = (1-rho)*maxVel/(maxVel+ego.lambda1);
            
            fontSize = 20;
            figHandle = figure;
            hold on; grid on;
            % The time discretization generates bumpy results, so we smooth
            % the final result
%             plot(gamma, smooth(collisionFreeGap), 'LineWidth', 1.5);
%             plot(gamma, analyticalGap, '-s','MarkerIndices',threshIdx,...
%                 'MarkerFaceColor','blue', 'MarkerSize',10, 'LineWidth', 1.5);
            
            colormap('gray')
            % xRange = (v0array([1 end])-v0L);
            xRange = [gamma(1), gamma(end)]; %*SimulationScenarios.mpsToKmph;
            sevMatrix = cell2mat(severity);
            yRange = [0 maxLength*obj.deltaS];
            imagesc(xRange, yRange, sevMatrix*SimulationScenarios.mpsToKmph);
            set(gca,'YDir','normal')
            xlim(xRange);
            ylim(yRange);
            c = colorbar('eastoutside');
            c.FontSize = fontSize;
            c.Label.String = 'Severity $[km/h]$';
            c.Label.Interpreter = 'latex';
            c.Label.FontSize = fontSize;
            plot(gamma, collisionFreeGap, 'w-.', 'LineWidth', 2);
            plot(gamma, vehFollGap, 'w:', 'LineWidth', 2);
            xline(gammaThreshold, 'w-', 'LineWidth', 1.5)
%             xline((1-rho)^2, 'm--', 'LineWidth', 1)
            
            legend({'Collision Free Gap', 'Veh Foll Gap', ...
                '$\gamma = (1-\rho) \frac{V_f}{V_f+\alpha_1}$'}, ...
                'Fontsize', fontSize, 'Location', 'southeast', ...
                'Interpreter', 'latex');
            xlabel('$\gamma$', 'Interpreter', 'latex');
            ylabel('$g_{\ell}^*(t_0) [m]$', 'Interpreter', 'latex');
            currentAxis = gca;
            currentAxis.XAxis.FontSize = fontSize;
            currentAxis.YAxis.FontSize = fontSize;
        end
        
        function figHandle = safeGapVsRho(obj)
%             close all
            
            % Simulation time
            t = obj.simTime;
            
            % Initial states
            vE = 25; % [m/s]
            rho = 0.2;
            deltaRho = 0.01;
            rhoArray = (0):deltaRho:(1);
            vL = (1-rhoArray)*vE;
            
            % Parameters for vehicle following distance
            delay = obj.timeToBrake.connected;
            maxVel = 30; % ~ 65 mph
            
            leader = VehicleLongitudinal('P', 'L', [0, 0], [delay, 0]);
            adjustedDelay = 0.1; %delay - leader.maxBrake/leader.maxJerk;
            leader.maxJerk = inf;
            ego = VehicleLongitudinal('P', 'F', [0, vE], [adjustedDelay, 0]);
            
%             collisionFreeGap = zeros(length(gamma), 1);
            analyticalGap = zeros(length(rhoArray), 1);
            vehFollGap = zeros(length(rhoArray), 1);
            severity = cell(1, length(rhoArray));
            for k = 1:length(rhoArray)
                leader.v0 = vL(k);
                leader.brakeFullStop(t);
                ego.brakeFullStop(t);

                ego.computeHeadway(leader, maxVel, rho);
                % Find minimum gaps
%                 [collisionFreeGap(k)] = ego.computeFollowingMSS(leader);
                [analyticalGap(k)] = ego.analyticalFollowingMSS(leader);
                vehFollGap(k) = ego.vehFollGap();
                
                % Simulate for different initial gaps
                initGapArray = 0:obj.deltaS:ceil(analyticalGap(k));
                severity{k} = ego.computeSeverity(leader, initGapArray);
                
            end
            
            maxLength = length(severity{end});
            for k = 1:length(rhoArray)
                severity{k}(end:maxLength) = 0;
            end
            
            fontSize = 20;
            figHandle = figure;
            hold on; grid on;
            colormap('gray')
            xRange = [rhoArray(1), rhoArray(end)]; %*SimulationScenarios.mpsToKmph;
            sevMatrix = cell2mat(severity);
            yRange = [0 maxLength*obj.deltaS];
            imagesc(xRange, yRange, sevMatrix*SimulationScenarios.mpsToKmph);
            set(gca,'YDir','normal')
            xlim(xRange);
            ylim(yRange);
            c = colorbar('northoutside');
            c.FontSize = fontSize;
            c.Label.String = 'Severity $[km/h]$';
            c.Label.Interpreter = 'latex';
            c.Label.FontSize = fontSize;
            plot(rhoArray, analyticalGap, 'LineWidth', 1.5);
            plot(rhoArray, vehFollGap, 'LineWidth', 1.5);
%             xline(gammaThreshold, '--', 'LineWidth', 1)
            xline(rho, 'm-', 'LineWidth', 1.5)
            xlim([rhoArray(1) rhoArray(end)]);
            legend({'Collision Free Gap', 'Veh Foll Gap', '$\rho$'}, ...
                'Fontsize', fontSize, 'Location', 'best', ...
                'Interpreter', 'latex')
            xlabel('$(v_E(t_0) - v_{\ell}(t_0))/v_E(t_0)$', 'Interpreter', 'latex');
            ylabel('$g_{\ell}^*(t_0) [m]$', 'Interpreter', 'latex');
            currentAxis = gca;
            currentAxis.XAxis.FontSize = fontSize;
            currentAxis.YAxis.FontSize = fontSize;
        end

        function h = headwayNumericalEstimation(obj)
            %headwayNumericalEstimation finds the safe gaps when follower
            %is going slower and faster than leader. Then does a linear fit
            %to find the headway time
                       
            t = obj.simTime;
            
            baseVel = obj.initVel;
            maxVel = 30; % ~65 mph
            deltaV = maxVel - baseVel;
            minVel = baseVel - deltaV;
            delay = obj.timeToBrake.connected;
            v0Larray = minVel:0.1:maxVel;
            
            types = {'P', 'T'};
            nTypes = length(types);
            h = -ones(nTypes, nTypes, length(v0Larray));
            for iV0 = 1:length(v0Larray)
                for iLeaderType = 1:nTypes
                    leader = VehicleLongitudinal(types{iLeaderType}, 'L', [0, v0Larray(iV0)], 0);
                    leader.maxJerk = inf;
                    leader.brakeFullStop(t);
                    
                    for iFollType = 1:nTypes
                        follower = VehicleLongitudinal(types{iFollType}, 'F', [0, v0Larray(iV0)], delay);
                        if iLeaderType == iFollType
                            follower.setNewMaxBrake(obj.gammaF*follower.maxBrake);
                        end
                        
                        follower.alternativeTimeHeadway(t, leader);
                        h(iLeaderType, iFollType, iV0) = follower.altH;
                    end
                end
            end
            
            % just exploring the results
            figure;
            hold on; grid on;
            for iLeaderType = 1:nTypes
                for iFollType = 1:nTypes
                    plot(v0Larray, squeeze(h(iLeaderType, iFollType, :)));
                end
            end
        end
        
        function figHandle = sampleWorstCaseFig(obj)
            %sampleWorstCaseFig Just sets parameters for a good 
            %illustration
            followerParams = struct('type', 'human', 'maxJerk', 10, ...
                'timeToBrake', 0.5, 'comfJerk', 7, 'comfBrake', 3, ...
                'maxAccel', 1);
            leaderParams = struct('maxJerk', inf, 'maxBrake', 10);
            figHandle = obj.accelWorstCasePlot(followerParams, leaderParams, ...
                'time ticks', 'accel ticks');
        end
        
        function figHandle = simplifiedWorstCasePlot(obj)
            
            % TODO: this function should just call the one below with
            % proper arguments, but in a hurry today [Sept 20, 2020]
            
            % Font size varies depending on the final size of the figure in
            % the paper
            fontSize = 22;         
            
            t = 0:obj.deltaT/10:obj.totalTime;
            v0L = obj.initVel;
            leader = VehicleLongitudinal('P', 'L', [0, v0L], 0);
            follower = VehicleLongitudinal('P', 'F', [0, v0L], [0.2, 0.3]);
            leader.maxJerk = inf;
            follower.maxJerk = inf;
            follower.comfJerk = inf;
            follower.comfBrake = 0;
            follower.setNewMaxBrake(follower.maxBrake*2);
            leader.setNewMaxBrake(follower.maxBrake*1.2);
            
            leader.brakeFullStop(t);
            figHandle = leader.plotState(t, 'a');
            follower.brakeFullStop(t);
            follower.plotState(t, 'a', figHandle);
            legend({'leader', 'ego'}, 'FontSize', fontSize);
            
            currentAxis = gca;
            currentAxis.XAxis.FontSize = fontSize;
            currentAxis.YAxis.FontSize = fontSize;
            xlabel('time');
            set(currentAxis,'xtick',[])
            ylabel('acceleration');
            yticks([-leader.maxBrake, -follower.maxBrake, 0, follower.maxAccel]);
            yticklabels({'-d_{l}', '-d_E', '0', 'a_E'});
            
        end

        function figHandle = connectedWorstCasePlot(obj)
            
            % TODO: this function should just call the one below with
            % proper arguments, but in a hurry today [Sept 20, 2020]
            
            % Font size varies depending on the final size of the figure in
            % the paper
            fontSize = 22;         
            
            t = 0:obj.deltaT/10:obj.totalTime;
            v0L = obj.initVel;
            leader = VehicleLongitudinal('P', 'L', [0, v0L], 0);
            follower = VehicleLongitudinal('P', 'F', [0, v0L], [0.2, 0]);
            leader.maxJerk = inf;
            follower.setNewMaxBrake(follower.maxBrake*2);
            leader.setNewMaxBrake(follower.maxBrake*1.2);
            
            leader.brakeFullStop(t);
            figHandle = leader.plotState(t, 'a');
            follower.brakeFullStop(t);
            follower.plotState(t, 'a', figHandle);
            legend({'leader', 'ego'}, 'FontSize', fontSize);
            
            currentAxis = gca;
            currentAxis.XAxis.FontSize = fontSize;
            currentAxis.YAxis.FontSize = fontSize;
            xlabel('time');
            set(currentAxis,'xtick',[])
            ylabel('acceleration');
            yticks([-leader.maxBrake, -follower.maxBrake, 0, follower.maxAccel]);
            currentAxis.TickLabelInterpreter = 'latex';
            yticklabels({'$-d_{\ell}$', '$-d_E$', '$0$', '$a_E$'});
        end
        
        function figHandle = accelWorstCasePlot(obj, followerParams, ...
                leaderParams, varargin)
            
            % followerType: human, autonomous, connected
            
            % Font size varies depending on the final size of the figure in
            % the paper
            fontSize = 22;
            
            % Non default parameters set when calling the function
            specialXTicks = false;
            specialYTicks = false;
            figHandle = [];
            for k = 1:length(varargin)
                arg = lower(varargin{k});
                if isgraphics(varargin{k})
                    figHandle = varargin{k};
                    continue;
                elseif strcmpi(arg, 'time ticks')
                    specialXTicks = true;
                    continue;
                elseif strcmpi(arg, 'accel ticks')
                    specialYTicks = true;
                    continue;
                end
                
            end
            
            nF = length(followerParams);
            
            t = 0:obj.deltaT/10:obj.totalTime;
            v0L = obj.initVel;
            v0F = obj.initVel;
            leader = VehicleLongitudinal('P', 'L', [0, v0L], 0);
            follower(nF) = VehicleLongitudinal();
            
            leaderParamsNames = fields(leaderParams);
            for n = 1:length(leaderParamsNames)
                leader.(leaderParamsNames{n}) = ...
                    leaderParams.(leaderParamsNames{n});
            end
            
            leader.brakeFullStop(t);
            if isempty(figHandle)
                figHandle = leader.plotState(t, 'a');
            else
                leader.plotState(t, 'a', figHandle);
            end
            
            for k = 1:nF
                % Default parameters follower
                followerType = followerParams(k).type;
                follower(k) = VehicleLongitudinal('P', 'F', [0, v0F], ...
                    [obj.timeToBrake.(followerType), ...
                    obj.timeToEmergBrake.(followerType)]);
                follower(k).maxBrake = obj.gammaF*leader(k).maxBrake;
                
                % User defined
                followerParamsNames = fields(followerParams(k));
                for n = 1:length(followerParamsNames)
                    follower.(followerParamsNames{n}) = ...
                        followerParams(k).(followerParamsNames{n});
                end
                follower(k).brakeFullStop(t);
                follower(k).plotState(t, 'a', figHandle);
            end
            
            if nF == 1
                legend({'leader', 'ego'}, 'FontSize', fontSize);
            else
                strCell = cell(nF+1, 1);
                strCell{1} = 'leader';
                for k = 1:nF
                    strCell{k+1} = [followerType 'follower'];
                end
                legend(strCell, 'FontSize', fontSize);
            end
            
            currentAxis = gca;
            currentAxis.XAxis.FontSize = fontSize;
            currentAxis.YAxis.FontSize = fontSize;
            if specialXTicks
                xlabel('time');
                set(currentAxis,'xtick',[])
%                 currentAxis.XLim = currentAxis.XLim + [-0.1, 0];
%                 t0 = leader.maxBrake/leader.maxJerk;
%                 xticks([0 t0 follower.timeToBrake]);
%                 xticklabels({'0', 't_0', '\tau'})
            end
            if specialYTicks
                ylabel('acceleration');
                yticks([-leader.maxBrake, -follower.maxBrake, -follower.comfBrake, 0, follower.maxAccel]);
                currentAxis.TickLabelInterpreter = 'latex';
                yticklabels({'$-d_{\ell}$', '$-d_E$', '$d_{E, c}$', '$0$', '$\bar{a}_E$'});
            end
        end

        function figHandle = latAccelExample(obj)

            t = 0:0.01:10;
            ego = VehicleRiskMap('P', 'ego', [0, obj.initVel, 0], obj.timeToEmergBrake.connected);
            ego.sinLatAccel(t, 0, max(t), obj.laneWidth);
            states = [ego.aLat, ego.vLat, cumtrapz(t, ego.vLat)];
            stateNames = {'a_y', 'v_y', 'y'};
            figHandle = figure;
            hold on; grid on;
            for k = 1:size(states, 2)
                subplot(size(states, 2), 1, k)
                plot(t, states(:, k), 'LineWidth', 1.5);
                ylabel(stateNames{k}, 'FontSize', 20)
                yticklabels({})
                xticklabels({})
                
            end
            xlabel('time', 'FontSize', 20);
            
            if obj.saveResults
                obj.mySavePlot(figHandle, 'lateral_states_example', 'png');
            end
            
        end
                
        function figHandles = severityColorMapsByVehicleType(obj, headwayType)
            %severityColorMapsByVehicleType creates 4 collision severity
            %plots along with the conservative linear overestimation of the
            %minimum safe gap
            %@headwayType: 0 (zero) for method from Petros' 94 paper with 
            %my adapation for considering the nonlinearities - this is 
            %only analytically valid for leader with equal or greater
            % deceleration; 1 (one) for newer method, which numerically
            % solves the minimum gap and then finds the linear overestimation

            close all
            
            if nargin==1
                headwayType = 0;
            end
            
            % Simulation time
            t = obj.simTime;
            
            % Initial states
            v0L = 20; % [m/s]
            deltaV = 5; %20/obj.mpsToKmph;
            v0array = v0L-deltaV:0.01:v0L+deltaV; % 54km/h to 126km/h
            nV0 = length(v0array);
            
            % Parameters for vehicle following distance
            delay = obj.timeToBrake.connected;
            gammaD = obj.gammaF;
            rho = 0.1;
            maxVel = 20; %30; % ~ 65 mph
            
            % Main loop to compute severity in all combinations
            types = {'P', 'T'};
%             scenarios = struct('follType', {'P', 'P', 'T', 'T'}, ...
%                 'leaderType', {'P', 'T', 'P', 'T'});
            totalTypes = length(types);
            maxMinSafeGap = zeros(totalTypes);
            maxnGaps = zeros(totalTypes);
            safeGapMatrix = zeros(totalTypes, totalTypes, nV0);
            severity = cell(totalTypes, totalTypes, nV0);
            gapVehFoll = zeros(totalTypes, totalTypes, nV0);
            for iLeaderType = 1:totalTypes
                leader = VehicleLongitudinal(types{iLeaderType}, 'L',...
                    [0, v0L], 0);
%                 leader.maxJerk = inf;
                leader.brakeFullStop(t);
                
                for iFollType = 1:totalTypes
                    follower = VehicleLongitudinal(types{iFollType}, 'F',...
                        [0, v0L], [delay, 0]);
                    if iLeaderType == iFollType
                        follower.setNewMaxBrake(gammaD*follower.maxBrake);
                    end
                    follower.computeHeadway(leader, maxVel, rho);
                    disp(follower.d0)
                    
                    for iV0 = 1:nV0
                        % Follower brakes
                        follower.v0 = v0array(iV0);
                        follower.brakeFullStop(t);
                        
                        % Find minimum gaps
%                         collisionFreeGap = follower.computeFollowingMSS(leader);
                        safeGapMatrix(iLeaderType, iFollType, iV0) = ...
                            follower.analyticalFollowingMSS(leader);
                        collisionFreeGap = ...
                            safeGapMatrix(iLeaderType, iFollType, iV0);
                        switch headwayType
                            case 0
                                gapVehFoll(iLeaderType, iFollType, iV0) = ...
                                    follower.vehFollGap();
                            case 1
                                gapVehFoll(iLeaderType, iFollType, iV0) = ...
                                    follower.alternativeVehFollGap(t, leader, velFactor);
                            otherwise
                                error('Unknown headway type selected (use 0 or 1)')
                        end
                        
                        % Simulate for different initial gaps
                        initGapArray = obj.deltaS:obj.deltaS:ceil(collisionFreeGap); 
                        nGaps = length(initGapArray);
                        
                        if collisionFreeGap>maxMinSafeGap(iLeaderType, iFollType)
                            maxMinSafeGap(iLeaderType, iFollType) = collisionFreeGap;
                            maxnGaps(iLeaderType, iFollType) = nGaps;
                        end

                        severity{iLeaderType,  iFollType, iV0} = ...
                            follower.computeSeverity(leader, initGapArray);
                        
                    end
%                     maxMinSafeGap(iLeaderType, iFollType) = ...
%                         max(safeGapMatrix(iLeaderType, iFollType, :));
                end
            end
                    
            % Fill the shorter arrays with zeros
            for iLeaderType = 1:totalTypes
                for iFollType = 1:totalTypes
                    for iV0 = 1:nV0
                        if isempty(severity{iLeaderType, iFollType, iV0})
                            severity{iLeaderType, iFollType, iV0} = ...
                                zeros(max(maxnGaps(:, iFollType)), 1);
                        else
                            severity{iLeaderType, iFollType, iV0}(end:max(maxnGaps(:, iFollType)), 1) = 0;
                        end
                    end
                end
            end
            
            % Plot
            figHandles = cell(totalTypes);
            figNames = cell(totalTypes);
            for iLeaderType = 1:totalTypes
                for iFollType = 1:totalTypes
                    % Plot
                    figNames{iLeaderType, iFollType} = ...
                        sprintf('sev_lead_%s_foll_%s_gray', types{iLeaderType},...
                        types{iFollType});
                    figHandles{iLeaderType, iFollType} = figure('Name', ...
                        figNames{iLeaderType, iFollType});
                    hold on;
                    colormap('gray')
                    % xRange = (v0array([1 end])-v0L);
                    xRange = [-deltaV, deltaV]; %*SimulationScenarios.mpsToKmph;
                    yRange = [0 ceil(max(maxMinSafeGap(:, iFollType)))];
                    imagesc(xRange, yRange, ...
                        squeeze(cell2mat(severity(iLeaderType, iFollType, :)))*obj.mpsToKmph);
                    set(gca,'YDir','normal')
                    xlim(xRange);
                    ylim(yRange);
                    xlabel('$v_E(t_0) - v_\ell(t_0) [km/h]$', 'Interpreter', 'latex');
                    ylabel('$g(t_0) [m]$', 'Interpreter', 'latex');
                    currentAxis = gca;
                    currentAxis.XAxis.FontSize = 20;
                    currentAxis.YAxis.FontSize = 20;
                    c = colorbar('northoutside');
                    c.FontSize = 18;
                    c.Label.String = 'Severity $[km/h]$';
                    c.Label.Interpreter = 'latex';
                    c.Label.FontSize = 18;
                    
                    plot((v0array-v0L), ...%*SimulationScenarios.mpsToKmph, ...
                        squeeze(gapVehFoll(iLeaderType, iFollType, :)),...
                        'r', 'LineWidth', 1.5);
                    plot((v0array-v0L), ...%*SimulationScenarios.mpsToKmph, ...
                        squeeze(safeGapMatrix(iLeaderType, iFollType, :)),...
                        'b', 'LineWidth', 1.5);
                    xline(v0L*rho/(1-rho), 'green')
                    legend('Time Headway Gap', ...
                        'FontSize', 18, 'Location', 'NW')
                end
            end
            
            if obj.saveResults
                % Increase figure size
                for iLeaderType = 1:totalTypes
                    for iFollType = 1:totalTypes
                    pos = figHandles{iLeaderType, iFollType}.Position;
                    figHandles{iLeaderType, iFollType}.Position = pos + ...
                        [0 -150 0 180];
                    end
                end
                obj.mySavePlot([figHandles{:}], figNames(:));
            end
        end

        function figHandle = laneChangeGapPlot(obj)
            
            % Parameters
            reactionTime = obj.timeToBrake.connected;
            vehLoc = {'Lo', 'Fd', 'Ld'};
            nVeh = length(vehLoc); 
            type = 'P';
            x0Ego = 0;
            y0Ego = 0;
            v0 = obj.initVel; % [m/s]
            deltaV = 10;
            
            % Create vehicles
            egoVeh = VehicleRiskMap(type, 'ego', [x0Ego, y0Ego, v0], reactionTime);
            vehs = VehicleRiskMap.empty(nVeh, 0);
            for iVeh = 1:nVeh
                otherVeh = vehLoc{iVeh};
                
                switch upper(otherVeh(1))
                    case 'L'
                        gamma = obj.gammaL;
                    case 'F'
                        gamma = obj.gammaF;
                    otherwise
                        error('unknown vehicle location')
                end
                
                switch lower(otherVeh(2))
                    case 'd'
                        y0 = y0Ego + obj.laneWidth;
                    case 'o'
                        y0 = y0Ego;
                end
                
                vehs(iVeh) = VehicleRiskMap(type, vehLoc{iVeh}, [0, y0, v0], reactionTime);
                vehs(iVeh).maxBrake = gamma*egoVeh.maxBrake;
            end
            
            % Compute min spacings
            v0array = v0-deltaV:0.1:v0+deltaV;
            [deltaGapsDuringLC, vehFollowingGap, ~] = obj.computeLCGaps(egoVeh, vehs, v0array);
                        
            % Plots
            xRange = (v0array([1 end])-v0)*SimulationScenarios.mpsToKmph;
                                
            figHandle = figure;
            hold on; grid on;
            plot((v0array-v0)*obj.mpsToKmph, vehFollowingGap+deltaGapsDuringLC, 'LineWidth', 1.5);
            legend(vehLoc, 'FontSize', 14, 'Location', 'best');
            xlim(xRange);
            xlabel('\Delta v(0) [km/h]');
            ylabel('g(0) [m]')
            
            if obj.saveResults
                obj.mySavePlot(figHandle, 'min_lc_gaps_equal_speeds');
            end
        end
        
        function figHandles = createRiskMaps(obj)
            %createRiskMaps will plot two risk maps for a left lane
            %change case
            % One risk map shows only vehicle following severity and the
            % second shows severity when we include lane change
            
            close all
            
            % Scenario
            nLanes = 2;
            scenarioWidth = nLanes*obj.laneWidth;
            t = obj.simTime;
            
            % Create vehicles
            reactionTime = obj.timeToBrake.connected; % [s]
            
            vehLoc = {'Fd', 'Ld', 'Fo', 'Lo'};
            nVeh = length(vehLoc);
            
            % Ego vehicle
            type = 'P';
            x0Ego = 0;
            y0Ego = 0;
            v0Ego = obj.initVel; %[m/s]
            
            % Assumption for safe vehicle following gap
%             rho = 0.9; % multiplicative factor vL/vF
%             maxVel = 30;
            
            egoVeh = VehicleRiskMap(type, 'ego', [x0Ego, y0Ego, v0Ego], ...
                reactionTime);
            
            % For now, x0 and y0 of ego are zero
            deltaV = 10/obj.mpsToKmph; % [m/s] -> *3.6[km/h]
            v0Left = v0Ego + deltaV;
            vehs = VehicleRiskMap.empty(nVeh, 0);
            x0 =  [x0Ego-egoVeh.len-40, x0Ego+50, ...
                x0Ego-egoVeh.len-35, x0Ego+40]; % [Fl, Ll, Fo, Lo]
            y0 = zeros(nVeh, 1);
            
            for n = 1:nVeh
                otherVeh = vehLoc{n};
                
                switch lower(otherVeh(2))
                    case {'l', 'd'} % left or destination lane
                        % Create vehicle to the left
                        y0(n) = y0Ego + obj.laneWidth;
                        vehs(n) = VehicleRiskMap(type, vehLoc{n}, ...
                            [x0(n), y0(n), v0Left], reactionTime);
                    case 'o'
                        % Create vehicle on the same lane
                        y0(n) = y0Ego;
                        vehs(n) = VehicleRiskMap(type, vehLoc{n}, ...
                            [x0(n), y0(n), v0Ego], reactionTime);
                end
                
                % Even "worse" case: assume followers brake less than leaders
                switch upper(otherVeh(1))
                    case 'L'
                        vehs(n).maxBrake = obj.gammaL*vehs(n).maxBrake;
                    case 'F'
                        vehs(n).maxBrake = obj.gammaF*vehs(n).maxBrake;
                    otherwise
                        error('unknown vehicle location')
                end
            end
            
            % Compute risks and create severity matrix
            
            minX = min([vehs.x0] - [vehs.len])-1;
            maxX = max([vehs.x0])+3;
            minY = min([vehs.y0] - obj.laneWidth/2);
            longSpace = minX:obj.deltaS:maxX;
            latSpace = minY:obj.deltaS:(minY+scenarioWidth);
            
            scenario = struct('laneChange', {0 1 1}, ...
                'egoAdj', {0 0 1}, 'follAdj', {0 0 0});
            nScenarios = length(scenario); %with or without lane change
            sevMap = cell(nScenarios, 1);
            
            for k = 1:nScenarios
                
                deltaGapsDuringLC = zeros(nVeh, 1);
                minVehFollowingGap = zeros(nVeh, 1);
                severity = cell(nVeh, 1);
                singleVehSevMaps = zeros(length(latSpace), length(longSpace), nVeh);
                
                for n = 1:nVeh
                    otherVeh = vehs(n);
                    
                    egoVeh.sinLatAccel(t, 0, obj.tLat, obj.laneWidth); % assumes left lane change
                    
                    if scenario(k).laneChange
                        if scenario(k).egoAdj
                            egoVeh.accelDuringLC(obj.simTime, v0Left);
                        else
                            egoVeh.zeroAccel(obj.simTime);
                        end
                        otherVeh.zeroAccel(obj.simTime);
                        deltaGapsDuringLC(n) = ...
                            egoVeh.computeGapVariationDuringLC(t, otherVeh, ...
                            obj.tLat);
                    end
                    
                    lcSpatialShift = deltaGapsDuringLC(n);
                    egoVeh.brakeFullStop(t, otherVeh);
                    otherVeh.brakeFullStop(t);
                    switch upper(otherVeh.name(1))
                        case 'L'
%                             egoVeh.adjustTimeHeadway(otherVeh, maxVel, rho);
                            if scenario(k).laneChange
                                minVehFollowingGap(n) = ...
                                    egoVeh.computeFollowingMSSDuringLC(t, ...
                                    otherVeh, scenario(k).egoAdj);
                            else
                                minVehFollowingGap(n) = ...
                                    egoVeh.computeFollowingMSS(otherVeh);
                            end
                            severity{n} = egoVeh.computeSeverityByDistance(otherVeh, minVehFollowingGap(n), obj.deltaS);
                            severity{n} = flip(severity{n});
                            vehRear = otherVeh.x0-otherVeh.len;
                            latCollisionZone = longSpace<=(vehRear) & ...
                                longSpace>=(vehRear-floor(lcSpatialShift));
                            riskyZone = longSpace<=(vehRear-floor(lcSpatialShift)) & ...
                                longSpace>=(vehRear-floor(lcSpatialShift)-ceil(minVehFollowingGap(n)));
                        case 'F'
%                             otherVeh.adjustTimeHeadway(egoVeh, maxVel, rho);
                            minVehFollowingGap(n) = otherVeh.computeFollowingMSS(egoVeh);
                            severity{n} = otherVeh.computeSeverityByDistance(egoVeh, minVehFollowingGap(n), obj.deltaS);
                            latCollisionZone = longSpace>=(otherVeh.x0) & ...
                                longSpace<=(otherVeh.x0+floor(lcSpatialShift));
                            riskyZone = longSpace>=(otherVeh.x0+floor(lcSpatialShift)) & ...
                                longSpace<=(otherVeh.x0+floor(lcSpatialShift)+ceil(minVehFollowingGap(n)));
                    end
                    
                    vehLatIdx = latSpace>=otherVeh.y0-otherVeh.width/2 & ...
                        latSpace<=otherVeh.y0+otherVeh.width/2;
                    singleVehSevMaps(vehLatIdx, latCollisionZone, n) = -1; %-1 is just a marker
                    singleVehSevMaps(vehLatIdx, riskyZone, n) = repmat(severity{n}', sum(vehLatIdx), 1);
                end
                
                singleVehSevMaps(singleVehSevMaps==-1) = max(singleVehSevMaps, [], 'all');
                sevMap{k} = max(singleVehSevMaps, [], 3);
                
            end
            
            % Plots

            cmax = ceil(max(max(cell2mat(sevMap)))*(SimulationScenarios.mpsToKmph));

            figHandles = zeros(nScenarios, 1);
            for k = 1:nScenarios
                figHandles(k) = figure;
                hold on;
                % title('Safe Gaps for Lane Change')
                scenarioLength = longSpace(end);%(idxMaxX+10);
                xRange = [minX, scenarioLength];
                yRange = [minY, minY+scenarioWidth];
                
                % Indicate lateral collision areas if lane change starts
                for n = 1:length(vehs) 
                    if scenario(k).laneChange
                        currentLCgap = deltaGapsDuringLC(n);
                        if strncmpi(vehs(n).name, 'F', 1) % Followers
                            collisionAreaStart = vehs(n).x0;
                            if currentLCgap>0
                                rectangle('Position', [collisionAreaStart, right, currentLCgap, vehs(n).width], 'FaceColor', 'y', 'LineWidth', 1.5);
                            end
                            
                        else % Leaders
                            collisionAreaStart = rear-currentLCgap;
                            if currentLCgap>0
                                rectangle('Position', [collisionAreaStart, right, currentLCgap, vehs(n).width], 'FaceColor', 'y', 'LineWidth', 1.5);
                            end
                            
                        end
                    end
                end
                colormap('gray')
                imagesc(xRange, yRange, sevMap{k}*(SimulationScenarios.mpsToKmph));
                if k==1
                    c = colorbar;
                    c.Location = 'northoutside';
                    set(gca,'YDir','normal')
                    caxis([0 cmax]);
                    c.Label.String = 'Severity [km/h]';
                elseif k==nScenarios
                    xlabel('x [m]')
                end
                xlabel('x [m]')
                %     set(gca,'xcolor','none')
                %     end
                % contour(longSpace, latSpace, sevMap);
                axis equal
                ylabel('y [m]')
                xlim(xRange);
                ylim(yRange);
                xticks(-50:10:50)
                yticks([0, 4])
                
                % To avoid having squeezed numbers on axis:
%                 position = get(gcf, 'Position');
%                 aspectRatio = position(3)/position(4);
%                 increment = 200;
%                 newPosition = position + [-increment*aspectRatio -increment increment*aspectRatio increment];
%                 set(gcf, 'Position', newPosition);
                
                % Plot vehicles and min gaps
                egoRear = egoVeh.x0-egoVeh.len;
                egoRight = egoVeh.y0-egoVeh.width/2;
                egoLeft = egoVeh.y0+egoVeh.width/2;
                egoPatch = patch([egoRear egoRear egoVeh.x0 egoVeh.x0 egoRear], [egoRight egoLeft egoLeft egoRight egoRight], [1 0 0]);
                if scenario(k).laneChange
                    rotate(egoPatch, [0 0 1], 15, [egoVeh.x0-egoVeh.len/2 egoVeh.y0 0]);
                end
                
                for n = 1:length(vehs)
                    rear = vehs(n).x0-vehs(n).len;
                    right = vehs(n).y0-vehs(n).width/2;
%                     myGreen = [0,0.5,0];
                    rectangle('Position', [rear, right, vehs(n).len, vehs(n).width],...
                        'FaceColor', 'g', 'Curvature', 1);
                end
                
            end
            
            if obj.saveResults
                figNames = {'riskMap_no_LC', 'riskMap_with_LC', ...
                    'riskMap_with_LC_and_adj'};
%                 figNames = 'both_risk_maps';
                obj.mySavePlot(figHandles, figNames);
            end
            
        end
        
        function figHandles = platoonPlots(obj)
            
            reactionTime = obj.timeToBrake.connected;
            vehLoc = {'Lo', 'Fd', 'Ld'};
            nVeh = length(vehLoc); 
            x0Ego = 0;
            y0Ego = 0;
            deltaVRange = 10;
            deltaV = -deltaVRange:0.1:deltaVRange; 

            % Different scenarios
            egoType = {'P', 'T', 'T',};
            otherType = {'P', 'T', 'P'};
            platoonVehs = {1:2:9, 5, 5};
            v0 = obj.initVel; % [m/s]
%             v0Cell = {25, 25, 25, [15, 20, 25]}; % [m/s]
            % Create vehicles
            nScenarios = length(egoType);
            gapAdjLane = cell(length(nScenarios), 1);
            for iS = 1:nScenarios
                platoonVeh = VehicleRiskMap(egoType{iS}, 'ego', [x0Ego, y0Ego, v0], reactionTime);
                vehs = VehicleRiskMap.empty(nVeh, 0);
                for iVeh = 1:nVeh
                    otherVeh = vehLoc{iVeh};
                    switch lower(otherVeh(2))
                        case 'd'
                            y0 = y0Ego + obj.laneWidth;
                        case 'o'
                            y0 = y0Ego;
                    end
                    
                    vehs(iVeh) = VehicleRiskMap(otherType{iS}, vehLoc{iVeh}, [0, y0, v0], reactionTime);
                end
                
                v0Array = v0+deltaV;
                gapAdjLane{iS} = obj.gapForPlatoon(platoonVeh, vehs, v0Array, platoonVehs{iS});
                
            end
            
            % First plot: N varies
            figHandles(1) = figure;
            hold on; grid on;
            plot(deltaV*obj.mpsToKmph, gapAdjLane{1}, 'LineWidth', 1.5);
            legend('N = ' + string(platoonVehs{1}), 'FontSize', 14, 'Location', 'best');
            xlim([deltaV(1) deltaV(end)]*SimulationScenarios.mpsToKmph);
            xlabel('\Delta v(0) [km/h]');
            ylabel('g_{dest} [m]')
            
            % Second plot: types vary
            figHandles(2) = figure;
            hold on; grid on;
            plot(deltaV*obj.mpsToKmph, [gapAdjLane{2}, gapAdjLane{3}], 'LineWidth', 1.5);
            legend({'Truck platoon among trucks', 'Truck platoon among PVs'}, 'FontSize', 14, 'Location', 'best');
            xlim([deltaV(1) deltaV(end)]*SimulationScenarios.mpsToKmph);
            xlabel('\Delta v(0) [km/h]');
            ylabel('g_{dest} [m]')
            
            if obj.saveResults
                figNames = {'min_lc_platoon_gaps_by_N','min_lc_platoon_gaps_diff_types'};
                obj.mySavePlot(figHandles, figNames);
            end
        end
                
        %%% Support methods %%%
        function [deltaGapsDuringLC, vehFollowingGap, collisionFreeGap] = computeLCGaps(obj, egoVeh, surroundingVehs, v0Array)
            %computeLCGaps Compute min spacings for each ego vehicle speed
            
            t = 0:obj.deltaT:obj.totalTime;
            nV0 = length(v0Array);
            nVeh = length(surroundingVehs);
            
            deltaGapsDuringLC = zeros(nVeh, nV0);
            collisionFreeGap = zeros(nVeh, nV0);
            vehFollowingGap = zeros(nVeh, nV0);
            maxVel = 30;
            rho = 0.9;
            for iV0 = 1:nV0
                for iVeh = 1:nVeh
                    otherVeh = surroundingVehs(iVeh);
                    
                    egoVeh.v0 = v0Array(iV0);
                    egoVeh.sinLatAccel(t, 0, obj.tLat, obj.laneWidth); % assumes left lane change
                    deltaGapsDuringLC(iVeh, iV0) = egoVeh.computeGapVariationDuringLC(t, otherVeh, obj.tLat);
                    
                    egoVeh.brakeFullStop(t, otherVeh);
                    otherVeh.brakeFullStop(t);
                    switch upper(otherVeh.name(1))
                        case 'L'
                            egoVeh.adjustTimeHeadway(otherVeh, maxVel, rho);
                            [collisionFreeGap(iVeh, iV0), vehFollowingGap(iVeh, iV0)] = egoVeh.computeFollowingMSSDuringLC(t, otherVeh, obj.tLat);
                            vehFollowingGap(iVeh, iV0) = egoVeh.vehFollGap();
                        case 'F'
                            otherVeh.adjustTimeHeadway(egoVeh, maxVel, rho);
                            collisionFreeGap(iVeh, iV0) = otherVeh.computeFollowingMSS(egoVeh);
                            vehFollowingGap(iVeh, iV0) = otherVeh.vehFollGap();
                    end
                end
            end
        end
        
        function gapAdjLane = gapForPlatoon(obj, platoonLeader, vehs, v0Array, vehiclesInPlatoon)
            %gapForPlatoon Compute total necessary gap at the adjacent lane
            
            % Compute min spacings
            [deltaGapsDuringLC, vehFollowingGap, ~] = obj.computeLCGaps(platoonLeader, vehs, v0Array);
            
            lcGap = deltaGapsDuringLC + vehFollowingGap;
            p1ToLdGap = lcGap(strcmp('Ld', {vehs.name}), :);
            pNToFdGap = lcGap(strcmp('Fd', {vehs.name}), :);
            
            midPlatoonVeh = VehicleRiskMap.copyVehicle(platoonLeader);
            midPlatoonVeh.setHeadwayDuringLC(platoonLeader); % just to create hLC
            
            gapAdjLane = zeros(length(v0Array), length(vehiclesInPlatoon));
            for nP = 1:length(vehiclesInPlatoon)
                interPlatoonGap = midPlatoonVeh.hLC*v0Array + midPlatoonVeh.d0LC;
                platoonLength = vehiclesInPlatoon(nP)*platoonLeader.len + (vehiclesInPlatoon(nP)-1)*interPlatoonGap;
                gapAdjLane(:, nP) = p1ToLdGap + platoonLength + pNToFdGap;
            end
            
        end
        
        %%% Save methods %%%
        function [] = mySavePlot(obj, figHandle, figName, fileType)
            
            if ischar(figName)
                figName = {figName};
            end
            
            for n = 1:length(figHandle)
                % Save in matlab format
                saveas(figHandle(n), ['.\figures\' figName{n}]);
                
                % Save in for latex use (or for Microsoft Office is needed)
                if nargin<4
                    print(figHandle(n), [obj.saveToFolder figName{n}], '-depsc', '-painters');
                else
                    saveas(figHandle(n), [obj.saveToFolder figName{n} '.' fileType]);
                end
            end
        end
        
        %%% Getters %%%
        function value = get.simTime(obj)
            value = 0:obj.deltaT:obj.totalTime;
        end
        
    end
end

