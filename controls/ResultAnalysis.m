 classdef ResultAnalysis < handle
    %ResultAnalysis Class containing functions to visualize simulation
    %results and generate plots for papers and reports
    
    properties
        saveResults = 0
        fileFormat = 'eps'
        simulationName
    end
    
    properties (Constant, Hidden)
        resultsFolder = '.\aggregate_maneuver_results\';
%         saveToFolder = 'C:\Users\fvall\Google Drive\Lane Change\images\';
        platoonLCfolder = 'G:\My Drive\Lane Change\platoon_lane_change\';
        laneChangeImageFolder = 'G:\My Drive\Lane Change\images\';
        videoFolder = '.\videos\'

        possibleFormats = {'eps', 'png', 'jpg'};
        mpsToKmph = 3.6;
    end
    
    methods
                      
        %%% Methods to generate report results %%%
        
        function [resultTable] = fullManeuverEfficiencyReport(...
                obj, isTruckLaneChange)
            
            reader = VehicleDataReader();
%             scenario = 'platoon';
            allLCStrategies = cellstr(SimulinkVehicle.possibleStrategies);
            %  = synchronous, leaderFirst, lastFirst, leaderFirstInvert      
            relVelPerStrategy = {[-5,0, 5]; [-5, 0,5]; [0, 5]; [-5, 0, 5]};
            nLCS = length(allLCStrategies);

            nSimulation = length([relVelPerStrategy{:}]);
            lcStrategyTracker = cell(nSimulation, 1);
            relVelTracker = zeros(nSimulation, 1);
            timeCost = zeros(nSimulation, 1);
            totalAccelCost = zeros(nSimulation, 1);
            itemizedAccelCost = struct('fo', cell(nSimulation, 1), ...
                'fd', cell(nSimulation, 1), 'platoon', cell(nSimulation, 1));
            origSpaceTimeCost = zeros(nSimulation, 1);
            destSpaceTimeCost = zeros(nSimulation, 1);
            iSimulation = 1;
            for iLCS = 1:nLCS
                lcStrategy = allLCStrategies{iLCS};
                nRV = length(relVelPerStrategy{iLCS});
                for iRV = 1:nRV
                    relVel = relVelPerStrategy{iLCS}(iRV);
                    lcStrategyTracker{iSimulation} = lcStrategy;
                    relVelTracker(iSimulation) = relVel;
                    
                    [origLane, destLane] = reader.loadVehiclesFromFile(...
                        'full_maneuver', 'platoon', relVel, ...
                        isTruckLaneChange, lcStrategy);
                    timeCost(iSimulation) = ...
                        ResultAnalysis.timeToFinishManeuver(...
                        origLane.getLaneChangingVehicleArray());
                    [totalAccelCost(iSimulation), ...
                        itemizedAccelCost(iSimulation)] = ...
                        ResultAnalysis.accelCost(origLane, destLane);
                    [origSpaceTimeCost(iSimulation), ...
                        destSpaceTimeCost(iSimulation)] = ...
                        ResultAnalysis.computeSpaceTimeCost(origLane, ...
                        destLane);
                    
                    iSimulation = iSimulation + 1;
                end
            end
            
            columnNames = {'Rel Vel [m/s]', 'LC Strategy',...
                'LC Time [s]', 'Fo Accel Cost [m^2/s^3]', ...
                'Fd Accel Cost [m^2/s^3]', 'Platoon Accel Cost [m^2/s^3]',...
                'Total Accel Cost [m^2/s^3]', 'Orig Space-Time [m.s]', ...
                'Dest Space-Time [m.s]'};
            resultTable = table(relVelTracker, lcStrategyTracker, timeCost, ...
                [itemizedAccelCost.fd]', [itemizedAccelCost.fo]', ...
                [itemizedAccelCost.platoon]', totalAccelCost, ...
                origSpaceTimeCost, destSpaceTimeCost, ...
                'VariableNames', columnNames);
            resultTable.('Total Space-Time [m.s]') = ...
                resultTable.('Orig Space-Time [m.s]') ...
                + resultTable.('Dest Space-Time [m.s]');            
            resultTable = sortrows(resultTable, {'Rel Vel [m/s]'});
            
            if isTruckLaneChange
                resultTable.Properties.Description = ['result_table_' ...
                    'truck_lane_change'];
            else
                resultTable.Properties.Description = ['result_table_' ...
                    'passenger_vehicle_lane_change'];
            end
            
            if obj.saveResults
                filename = resultTable.Properties.Description;
                save([obj.resultsFolder filename], 'resultTable')
            end
            
%             figName{iRV} = ['full_maneuver_platoon_rel_vel_' ...
%                 num2str(relVel) '_space_time_cost'];
%             obj.exportToLatexTable(scenario, relVel, lcStrategies, ...
%                 timeCost, accelCost, spaceTimeCost);
%             obj.saveToLaneChangeFolder(figArray, figName, false);
        end
        
        function [] = fullManeuverLCStatesPlot(obj, isTruckLaneChange)
            %fullManeuverLCStatesPlot Plots some illustrative examples of
            %the lane change maneuver states over time
            
            if nargin>1 && isTruckLaneChange
                truckString = 'truckLC_';
            else
                isTruckLaneChange = false;
                truckString = '';
            end
            
            reader = VehicleDataReader();
            allLCStrategies = cellstr(SimulinkVehicle.possibleStrategies);
            %  = synchronous, leaderFirst, lastFirst, leaderFirstInvert      
            relVelPerStrategy = {0; 0; 0; 0};
            nLCS = length(allLCStrategies);
            
            for iLCS = 1:nLCS
                lcStrategy = allLCStrategies{iLCS};
                nRV = length(relVelPerStrategy{iLCS});
                for iRV = 1:nRV
                    relVel = relVelPerStrategy{iLCS}(iRV);
                    [origLane, ~] = reader.loadVehiclesFromFile(...
                        'full_maneuver', 'platoon', relVel, ...
                        isTruckLaneChange, lcStrategy);
                    fig = origLane.plotManeuverStates(false);
                    figName = sprintf(['full_maneuver_platoon_%srel_vel_'...
                        '%d_strategy_%s_maneuverStates'], ...
                        truckString, relVel, lcStrategy);
                    obj.saveToLaneChangeFolder(fig, figName);
                end
            end
            
        end
        
        function [] = fullManeuverPlotsForReport(obj, scenario)
            %fullManeuverPlotsForReport generates one figure with gaps,
            %velocities and lateral position and another with headway,
            %velocity and lateral errors for all simulations
            % The states are plotted for the platoon vehicles; gaps include
            % virtual leader for main lane changing vehicle. The error
            % plots only show the relevant vehicles for each case
            
            close all;
            
            withComms = 0;
            reader = VehicleDataReader();
            
            if strcmp(scenario, 'platoon')
                warning(['Probably need some extra coding to adjust y '...
                    ' scale on gap plots']);
                relVel = [0, -4, 4];
                lcStrategies = 1:3;
                relVelStratPairs = [relVel',lcStrategies'];
            else
                relVel = [0, -5, 5];
                relVelStratPairs = [relVel',ones(length(relVel), 1)];
            end
            
            % Note: careful when running this in debug mode - be sure to
            % set visibility back to 'on' after running this
%             set(0,'DefaultFigureVisible','off');
            for n = 1:size(relVelStratPairs, 1)
                rv = relVelStratPairs(n, 1);
                st = relVelStratPairs(n, 2);
                
                reader.loadVehiclesFromFile('full_maneuver', scenario, rv, ...
                    false, st);    
                obj.plotPlatoonStates();
%                 obj.plotFullManeuverErrors();
            end
            
            set(0,'DefaultFigureVisible','on');
        end
        
        function [] = fullManeuverPlotsForPaper(obj, scenario, withComms)
            %fullManeuverPlotsForPaper generates one figure with gaps,
            %velocities and input and another with headway and
            %velocity errors and lateral position for all simulations
            
            if strcmp(scenario, 'platoon')
                error(['Plots for platoon scenario not yet defined.'...
                    'Consider using same as report.'])
            end
            
            close all;
            
            reader = VehicleDataReader();
            relVel = [0, -5, 5];
            lcStrategy = 1;
            
            for nRV = 1:length(relVel)
                %for nS = 1:length(lcStrategy)
                rv = relVel(nRV);
                
                infoString = sprintf(['Plots for %s lane change, '...
                    '%g relative velocity'], scenario, rv);
                if withComms
                    disp([infoString, ' with acceleration feedback']);
                else
                    disp(infoString);
                end
                
                
                [origLane, destLane] = reader.loadVehiclesFromFile(...
                    'full_maneuver', scenario, rv, withComms, lcStrategy,...
                    true);
                destLane.getVehByName('fd').plotName = 'virtual follower';
                origLane.getVehByName('E').plotName = 'ego';
                obj.plotEgoAndFdErrors();
                obj.plotEgoAndFdInputs();
            end
        end
        
        function [] = createAndSaveAllAnimations(obj, scenario, withComms)
            
            if strcmp(scenario, 'platoon')
                relVel = [0, -4, 4];
                lcStrategies = 1:3;
            else
                relVel = [0, -5, 5];
                lcStrategies = 1;
            end
            
            nRV = length(relVel);
            nS = length(lcStrategies);
            
            % Note: careful when running this in debug mode - be sure to
            % set visibility back to 'on' after running this
%             set(0,'DefaultFigureVisible','off');
            obj.saveResults = true;
            
            for n = 1:nRV
                for k = 1:nS
                    
                    fprintf(['Starting animation for: \n\t%s scenario'...
                        '\n\trel vel: %d \n\tcommunications: %s '...
                        '\n\tstrategy: %d\n'], ... 
                    scenario, relVel(n), mat2str(withComms), k);
                    
                    if relVel(n)<0 && k==3
                        continue
                    end
                    obj.loadVehiclesFromFile('full_maneuver', scenario, ...
                        relVel(n), withComms, k);
                    
                    obj.createAnimation();
                    close;
                end
            end
            
            obj.saveResults = false;
%             set(0,'DefaultFigureVisible','on');
        end
        
        function [] = gapGenerationAllPlots(obj)

            reader = VehicleDataReader();
            scenario = {'platoon', 'single_veh'};
            relVel = [0, -4, 4];

            % Note: careful when running this in debug mode - be sure to
            % set visibility back to 'on' after running this
            set(0,'DefaultFigureVisible','off');
            for sc = scenario
                for rv = relVel
                    reader.loadVehiclesFromFile('gap_generation', ...
                        sc, rv, false, 1);
                    obj.plotGapGenerationResults();
                end
            end
            
            set(0,'DefaultFigureVisible','on');
        end

        %%% Simulink simulation plots %%%
        
        function [statePlots] = plotPlatoonStates(obj, origLane)
            %plotFullManeuverStates Plots states gap, y and velocity for
            %platoon vehicles
            
            figName = [obj.simulationName '_states'];
            
            if contains(obj.simulationName, 'platoon')
                if origLane.lcStrategy<3
                    mainVeh = 'p1';
                else
                    mainVeh = 'p3';
                end
            else
                mainVeh = 'E';
            end
            
            % The plot from p1 to Ld has to be plotted separatly cause they
            % belong to different vehicle arrays
            pMainToLdFig = origLane.getVehByName(mainVeh).plotStates(...
                'gap', obj.destLane.getVehByName('Ld'));
            line = findobj(pMainToLdFig, 'type', 'Line');
            line.LineStyle = '--';
            
            % Create single figure
            laneChangingArray = origLane.getLaneChangingVeh();
            statePlots = laneChangingArray.plotStates(...
                {laneChangingArray.vehs.name}, {'gap', 'velocity', 'y'});
            allAxes = findobj(statePlots, 'type', 'Axes');
            gapAxes = allAxes(3);
            ResultAnalysis.mergePlots(pMainToLdFig, gapAxes);
            
            close(pMainToLdFig);

            obj.saveToLaneChangeFolder(statePlots, figName, true);
            
        end
        
        function [errorPlots] = plotFullManeuverErrors(obj, ...
                origLane, destLane)
            %plotFullManeuverStates Plots critical errors for each scenario
            
            % Check after coding:
            % We'll start coding based on strategy but we might be able to
            % simplify afterwards
            
            figName = [obj.simulationName '_errors'];
            
            if origLane.vehs(1).vx0 >= destLane.vehs(1).vx0
                closestLeader = 'Virtual';
            else
                closestLeader = 'Real';
            end
            
            fd = destLane.getVehByName('fd');
            
            if contains(obj.simulationName, 'platoon')
                warning(['Simulations have to be rerun to save timeseries '...
                    'of vehicle states']);
                lcStrategy = origLane.lcStrategy;
                switch lcStrategy
                    case 1 % Synchronous
                        % veh foll errors of p1 and p3
                        % veh foll errors from Fd to p3
                        veh1 = origLane.getVehByName('p1');
                        veh2 = origLane.getVehByName('p3');                 
                    case 2 % Leader First
                        % Show only for vehicle 1 since all are similar
                        veh1 = origLane.getVehByName('p1');
                        veh2 = veh1;
                    case 3 % Last Vehicle First
                        % Show only for vehicle 3 since all are similar
                        veh1 = origLane.getVehByName('p3');
                        veh2 = veh1;
                    otherwise
                        error('Unknown strategy')
                end
            else
                veh1 = origLane.getVehByName('E');
                veh2 = veh1;
            end
            
            warning(['[3/18/21] Outdated code. Property lcStateDict became '...
                'lcStateNameToNum and the values must be updated'])
            errorPlots = veh1.plotSimErrors({['eg' closestLeader], ['ev' closestLeader], 'ey'});
            errorPlots = fd.plotStates({'eg', 'ev', 'ey'}, veh2, errorPlots);
            lcStateTimeSeries = veh1.lcState;
            lcEndTime = lcStateTimeSeries.time(...
                find(lcStateTimeSeries.data >= SimulinkVehicle.lcStateDict.lcDone, 1));
            
            % Delete Fd from ey plot
            delete(findobj(errorPlots, 'DisplayName', 'Fd'));
            
            % Adjust time in plots
            errorAxes = findobj(errorPlots, 'type', 'axes');
            for n = 1:length(errorAxes)
                errorAxes(n).XLim = [0 lcEndTime-0.1];
                errorAxes(n).YLimMode = 'auto';
                vertLims = errorAxes(n).YLim;
                errorAxes(n).YLim = [vertLims(1) max(vertLims(2), -vertLims(1)/10)];
            end
            
            obj.saveToLaneChangeFolder(errorPlots, figName, true);
            
        end
        
        function [statePlots] = plotEgoAndFdLongStatesAndInput(obj, ...
                origLane, destLane)
            %plotEAndFdLongStatesAndInput plots from E and Fd to their real
            %and virtual leader, their velocities and inputs
            
            %%% OPTION 1: still checking if these are a good choice
            ego = origLane.getVehByName('E');
            fd = destLane.getVehByName('Fd');
            ld = destLane.getVehByName('Ld');
            
            laneChangingArray = origLane.getLaneChangingVeh();
            statePlots = laneChangingArray.plotStates(...
                {laneChangingArray.vehs.name}, {'gap', 'velocity', 'u'});
            fd.plotStates({'gap', 'velocity', 'u'}, ld, statePlots, ...
                'Color', 'r');
            % The plot from E to Ld has to be done separatly cause they
            % belong to different vehicle arrays
            egoToLdFig = ego.plotStates('gap', ld, figure, ...
                'LineStyle', '--');
            fd.plotStates('gap', ego, egoToLdFig, 'Color', 'r', ...
                'LineStyle','--');
            
            allAxes = findobj(statePlots, 'type', 'Axes');
            gapAxes = allAxes(3);
            obj.mergePlots(egoToLdFig, gapAxes);
            
            close(egoToLdFig);
        end
        
        function [errorPlots] = plotEgoAndFdErrors(obj, origLane, destLane)
            
            figName = [obj.simulationName '_errors'];
            
            if origLane.vehs(1).vx0 >= destLane.vehs(1).vx0
                closestLeader = 'Virtual';
            else
                closestLeader = 'Real';
            end
            
            veh1 = origLane.getVehByName('E');
            veh2 = veh1; % might be needed if this is adapted for platoons
            fd = destLane.getVehByName('fd');
            
            errorPlots = veh1.plotSimErrors({['eg' closestLeader], ...
                ['ev' closestLeader], 'ey'}, 'Color', 'r');
            errorPlots = fd.plotStates({'eg', 'ev', 'ey'}, veh2, ...
                errorPlots, 'Color', [0, 0.5, 0]);
            
            % Adjust scale in plots: the headway error plot grows a lot in
            % some cases and we don't need to show it all cause we lose
            % detail on the area that matters
            errorAxes = findobj(errorPlots, 'type', 'axes');
            maxHeadwayError = 5;
            for n = 1:length(errorAxes)
                if strcmpi(errorAxes(n).Tag, 'e_h')
                    currentLim = errorAxes(n).YLim;
                    if currentLim(2) > maxHeadwayError
                        errorAxes(n).YLim = [...
                            currentLim(1)+0.05*(currentLim(2)-maxHeadwayError),...
                            maxHeadwayError];
                    end
                end
            end
            
            obj.saveToLaneChangeFolder(errorPlots, figName);
        end
        
        function [errorPlots] = plotEgoAndFdLongErrors(obj, ...
                origLane, destLane)
            
            figName = [obj.simulationName '_long_errors'];
            
            if origLane.vehs(1).vx0 >= destLane.vehs(1).vx0
                closestLeader = 'Virtual';
            else
                closestLeader = 'Real';
            end
            
            veh1 = origLane.getVehByName('E');
            veh2 = veh1; % might be needed if this is adapted for platoons
            fd = destLane.getVehByName('fd');
            
            errorPlots = veh1.plotSimErrors({['eg' closestLeader], ...
                ['ev' closestLeader]}, 'Color', 'r');
            errorPlots = fd.plotStates({'eg', 'ev'}, veh2, errorPlots, ...
                'Color', [0, 0.5, 0]);
            
            % Adjust scale in plots: the headway error plot grows a lot in
            % some cases and we don't need to show it all cause we lose
            % detail on the area that matters
            errorAxes = findobj(errorPlots, 'type', 'axes');
            maxHeadwayError = 5;
            for n = 1:length(errorAxes)
                if strcmpi(errorAxes(n).Tag, '\tilde{g}')
                    currentLim = errorAxes(n).YLim;
                    if currentLim(2) > maxHeadwayError
                        errorAxes(n).YLim = [...
                            currentLim(1)+0.05*(currentLim(2)-maxHeadwayError),...
                            maxHeadwayError];
                    end
                end
            end
            
            obj.saveToLaneChangeFolder(errorPlots, figName);
        end
        
        function [errorPlots] = plotEgoAndFdLongErrorsAndInput(obj, ...
                origLane, destLane)
            
            figName = [obj.simulationName '_long_errors_and_input'];
            
            if origLane.vehs(1).vx0 >= destLane.vehs(1).vx0
                closestLeader = 'Virtual';
            else
                closestLeader = 'Real';
            end
            
            veh1 = origLane.getVehByName('E');
            veh2 = veh1; % might be needed if this is adapted for platoons
            fd = destLane.getVehByName('fd');
            
            % Find lane change start and end time indices
            warning(['[3/18/21] Outdated code. Property lcStateDict became '...
                'lcStateNameToNum and the values must be updated'])
            lcStartTime = veh1.lcState.time(find(veh1.lcState.data>=...
                veh1.lcStateDict.laneChanging, 1));
            lcEndTime = veh1.lcState.time(find(veh1.lcState.data>...
                veh1.lcStateDict.laneChanging, 1));
            lcStartIdx = find(veh1.simTime>=lcStartTime, 1);
            lcEndIdx = find(veh1.simTime>=lcEndTime, 1);
            
            errorPlots = veh1.plotSimLongErrorsAndInput({['eg' closestLeader]...
                , ['ev' closestLeader]}, '-x', 'MarkerIndices', ...
                [lcStartIdx, lcEndIdx], 'MarkerFaceColor', 'r', ...
                'MarkerSize', 10, 'Color', 'r', 'LineWidth', 1.5);
            errorPlots = fd.plotStates({'eg', 'ev', 'u'}, veh2, errorPlots,...
                'Color', [0, 0.5, 0]);
            
            % Adjust scale in plots: the headway error plot grows a lot in
            % some cases and we don't need to show it all cause we lose
            % detail on the area that matters
            errorAxes = findobj(errorPlots, 'type', 'axes');
            maxHeadwayError = 10;
            for n = 1:length(errorAxes)
                if strcmpi(errorAxes(n).Tag, '\tilde{g}')
                    currentLim = errorAxes(n).YLim;
                    if currentLim(2) > maxHeadwayError
                        errorAxes(n).YLim = [...
                            currentLim(1)+0.05*(currentLim(2)-maxHeadwayError),...
                            maxHeadwayError];
                    end
                end
            end
            
            obj.saveToLaneChangeFolder(errorPlots, figName);
        end
                
        function [uAndDeltaPlot] = plotEgoAndFdInputs(obj, origLane, ...
                destLane)
            
            figName = [obj.simulationName '_inputs'];
            
            ego = origLane.getVehByName('E');
            fd = destLane.getVehByName('Fd');
            
            uAndDeltaPlot = ego.plotStates({'u', 'delta'}, figure, ...
                'Color', 'r');
            uAndDeltaPlot = fd.plotStates({'u', 'delta'}, uAndDeltaPlot, ...
                'Color', [0, 0.5, 0]);
            
            obj.saveToLaneChangeFolder(uAndDeltaPlot, figName);
            
        end
        
        function [uAndYPlot] = plotEgoAndFdInputAndY(obj, origLane, ...
                destLane)
            
            figName = [obj.simulationName '_u_and_y'];
            
            ego = origLane.getVehByName('E');
            fd = destLane.getVehByName('Fd');
            
            uAndYPlot = ego.plotStates({'u', 'ey'}, figure, 'Color', 'r');
            uAndYPlot = fd.plotStates({'u', 'ey'}, uAndYPlot, ...
                'Color', [0, 0.5, 0]);
            
            obj.saveToLaneChangeFolder(uAndYPlot, figName);
            
        end
        
        function [] = plotGapGenerationResults(obj, origLane, destLane)
            %plotResults Plots all figures used in gap generation papers
            %(up to 10 figures per call)
            % figures plotted: 'p1Errors', 'p1ErrorsAndInput', 'p1Gaps', 
            % 'p1VelAndInput', 'fdErrors', 'fdErrorsAndInput', 'fdGaps',
            % 'fdVelAndInput', 'interPlatoonStates', 'interPlatoonErrors'
            
            disp(['Plotting simulation ' obj.simulationName]);
            
            figNames = {'p1Errors', 'p1ErrorsAndInput', 'p1Gaps', 'p1VelAndInput',...
                'fdErrors', 'fdErrorsAndInput', 'fdGaps', 'fdVelAndInput', ...
                'interPlatoonStates', 'interPlatoonErrors'};
            
            p1 = origLane.getVehByName('p1');
            lo = origLane.getVehByName('lo');
            ld = destLane.getVehByName('ld');
            fd = destLane.getVehByName('fd');
            
            % Plots
            figHandle(1) = p1.plotSimErrors();
            figHandle(2) = p1.plotSimErrorsAndInput();
            figHandle(3) = p1.plotStates('gap', [lo, ld]);
            figHandle(4) = p1.plotStates({'velocity', 'u'});
            figHandle(5) = fd.plotSimErrors();
            figHandle(6) = fd.plotSimErrorsAndInput();
            figHandle(7) = fd.plotStates('gap', [ld, origLane.vehs(end-1)]);
            figHandle(8) = fd.plotStates({'velocity', 'u'});
            
            if contains(obj.simulationName, 'platoon')
                figHandle(9) = origLane.plotStates(...
                    {'gap', 'velocity', 'u'}, {'p1', 'p2', 'p3'});
                figHandle(10) = origLane.plotSimErrors({'p2', 'p3'});
            end
            
            obj.saveToLaneChangeFolder(figHandle(f), [obj.simulationName '_' figNames{f}]);
            
        end
        
        %%% Animations %%%
        
        function [] = createAnimation(obj, allVehicles)
            %createAnimation Creates animation based on an array of
            %vehicle objects (any concrete class derived from Vehicle)
            
            % Special vehicles
            % In simulation with a single lane changing vehicle, the
            % vehicle is called E. In simulations with platoon lane
            % changes, the vehicles are called p1, p2, ..., pN
            allNames = {allVehicles.name};
            lo = allVehicles(strcmpi(allNames, 'Lo'));
            ego = allVehicles(strcmpi(allNames, 'E'));
            if isempty(ego)
                ego = allVehicles(strcmpi(allNames, 'ego'));
                if isempty(ego)
                    ego = allVehicles(strcmpi(allNames, 'p1'));
                end
            end
            fo = allVehicles(strcmpi(allNames, 'Fo'));
            ld = allVehicles(strcmpi(allNames, 'Ld'));
            fd = allVehicles(strcmpi(allNames, 'Fd'));
            
            % Remove vehicles that won't show in the plot anyway
            extraVehicles = length(allVehicles) - 5;
            if ego.vx0 >= ld.vx0
                deletePerLane = floor(extraVehicles/4);
                idxToDelete = false(1, length(allVehicles));
                for n = 1:deletePerLane
                    idxToDelete = idxToDelete ...
                        | strcmpi(allNames, ['orig' num2str(n)]);
                end
                for n = deletePerLane+1:2*deletePerLane
                    idxToDelete = idxToDelete ...
                        | strcmpi(allNames, ['dest' num2str(n)]);
                end
            end
            if ego.vx0 <= ld.vx0
                deletePerLane = floor(extraVehicles/4);
                idxToDelete = false(1, length(allVehicles));
                for n = 1:deletePerLane
                    idxToDelete = idxToDelete ...
                        | strcmpi(allNames, ['dest' num2str(n)]);
                end
                for n = deletePerLane+1:2*deletePerLane
                    idxToDelete = idxToDelete ...
                        | strcmpi(allNames, ['orig' num2str(n)]);
                end
            end
            allVehicles(idxToDelete) = [];
            
            %%% TO DO: probably don't need a loop to do this
            vehColors = cell(1, length(allVehicles));
            for n = 1:length(allVehicles)
                if strcmpi(allVehicles(n).name, 'p1') ...
                        || strcmp(allVehicles(n).name, 'E')
                    vehColors{n} = [1 0 0]; % red
                elseif contains(allVehicles(n).name, 'p')
                    vehColors{n} = [1 0.5 0];
                elseif strcmpi(allVehicles(n).name, 'fd')
                    vehColors{n} = [0 1 0]; % green
                else
                    vehColors{n} = [0.5 0.5 0.5]; % grey
                end
            end
            
            % Simulation time
            simTime = ego.simTime;
            
            % Precompute X and Y moving areas during video
            maxX = zeros(length(simTime), 1);
            minX = zeros(length(simTime), 1);
            for k = 1:length(simTime)
                maxX(k) = min(ld.x(k), lo.x(k));
                minX(k) = min(fd.x(k), fo.x(k));
            end
            videoWidth = max(maxX-minX);
            videoXLims = [minX, minX+videoWidth];
            videoYLims = [fo.y(1) - fo.width, ...
                fd.y(1) + fd.width*2];
            
            % Lane marks coordinates
            laneLineYCoord = ones(1,2)*(fd.y(1) + fd.width ...
                + ego.y(1))/2;
            laneLineXCoord = [0, maxX(end)+15];
            
            % Scaling factor for velocity plotting
%             speedScaler = 5;
            
            % We don't want to plot every simulated point. Every 0.1 second
            % is enough
            % plotTime definition is sloppy for now. Better to define
            % simulation end time based on when the maneuver is done
%             [~, ~, endIdx] = ResultAnalysis.timeToFinishManeuver();
            plotTime = simTime(1):0.2:simTime(end-1);
%             plotTime = plotTime(plotTime<ceil(maneuverEndTime)+1);
            K = length(plotTime);
            movieVector(K-1) = struct('cdata',[],'colormap',[]);
            
            % Create figure and subplots, set axes limits, labels etc
            figure();
            
            animationAxis = subplot(3, 1, 1);
            axis equal
            xlabel('x [m]');
            ylim(videoYLims);
            hold on; grid on;
            
            accelAxis = subplot(3, 1, 2);
            hold on; grid on;
            xlabel('t [s]');
            ylabel('a_x [m/s^2]')
            xlim([plotTime(1) plotTime(end)]);
            ylim([min([ego.ax; fd.ax])*1.1 max([ego.ax; fd.ax])*1.1]);
            accelLineEgo = animatedline(accelAxis, 'LineWidth', 1.5, 'Color', 'r');
            accelLineFd = animatedline(accelAxis, 'LineWidth', 1.5, 'Color', 'g');
%             legend(ego.plotName, fd.plotName);
            
            velAxis = subplot(3, 1, 3);
            hold on; grid on;
            xlabel('t [s]')
            ylabel('v_x [m/s^2]')
            xlim([plotTime(1) plotTime(end)]);
            ylim([min([ego.vx; fd.vx])*0.9 max([ego.vx; fd.vx])*1.1]);
            velLineEgo = animatedline(velAxis, 'LineWidth', 1.5, 'Color', 'r');
            velLineFd = animatedline(velAxis, 'LineWidth', 1.5, 'Color', 'g');
%             legend(ego.plotName, fd.plotName);
            
            if ego.vx0<fd.vx0
                sgtitle('Origin lane slower than destination')
            elseif ego.vx0>fd.vx0
                sgtitle('Origin lane faster than destination')
            else
                sgtitle('Both lanes at the same speed')
            end

            for plotTimeIdx = 1:K-1
                % Clear the figure
%                 clf;
                
                % Find the time index
                simTimeIdx = find(simTime >= plotTime(plotTimeIdx), 1);
                k = simTimeIdx;
                
                % Start animation plot
                cla(animationAxis)
                title(animationAxis, ['t = ',num2str(simTime(k))])

                % Set area being shown
                xMin = videoXLims(k, 1) - 10;
                xMax = videoXLims(k, 2) + 15;
                xlim(animationAxis, [xMin xMax]);
                xticks(animationAxis, round(xMin, -1):20:round(xMax, 1))
                ylim(animationAxis, videoYLims);
                
                % Lane line
                plot(animationAxis, laneLineXCoord, laneLineYCoord, '--k');
                % Plot the vehicles
                for nV = 1:length(allVehicles)
                    veh = allVehicles(nV);
                    vehX = veh.x(k);
                    vehY = veh.y(k);
                    vehPsi = veh.psi(k)*180/pi;
                    
                    vehImg = patch(animationAxis, ...
                        [vehX vehX+veh.len vehX+veh.len vehX vehX], ...
                        [vehY vehY vehY+veh.width vehY+veh.width vehY], ...
                        vehColors{nV});
                    rotate(vehImg, [0 0 1], vehPsi, [vehX vehY 0])
                    % Include arrows to indicate speed
%                     obj.drawArrow(gca, 
%                         [vehX + veh.len, vehX + veh.len + veh.states(vxIdx)/speedScaler],...
%                         [vehY + veh.width/2, vehY + veh.width/2]);
                end
                
                % Plot acceleration
                addpoints(accelLineEgo, simTime(k), ego.ax(k));
                addpoints(accelLineFd, simTime(k), fd.ax(k));
                
                % Plot velocity
                addpoints(velLineEgo, simTime(k), ego.vx(k));
                addpoints(velLineFd, simTime(k), fd.vx(k));
                
                % Take a snapshot
                movieVector(plotTimeIdx) = getframe(gcf);
            end
            
            if obj.saveResults
                %Create a VideoWriter object and set properties
                myWriter = VideoWriter(['.\videos\' obj.simulationName ...
                    '_video'], 'MPEG-4'); %create an .avi file
                % myWriter = VideoWriter('curve','MPEG-4');   %create an .mp4 file
                myWriter.FrameRate = 15;
                myWriter.Quality = 40;
                
                %Open the VideoWriter object, write the movie, and close the file
                open(myWriter);
                writeVideo(myWriter, movieVector);
                close(myWriter);
            end
            
        end
        
        function [] = create3DAnimation(obj, origLane, destLane)
            simulinkModelsFolder = './simulink_models/';
            simulinkModelName = 'create_3D_animation';
            
            % Load model if needed
            if ~bdIsLoaded(simulinkModelName)
                modelFullPath = [simulinkModelsFolder simulinkModelName];
                load_system(modelFullPath);
            end
            
            [X, Y, Psi] = loadTrajectories(obj);
            allVehs = [origLane.vehs destLane.vehs];
            [~, ~, maneuverEndTime] = obj.timeToFinishManeuver();
            

            for nV = 1:length(allVehs)
                veh = allVehs(nV);
                vehName = veh.name;
                
                % For some reason Simulink issues an error when we use the 
                % name Ld 
                if strcmpi(vehName, 'Ld') 
                    vehName = 'Ld1';
                end
                
                % Lane changing vehicles are red; others are green
                if contains(allVehs(nV).name, 'p') || strcmp(allVehs(nV).name, 'E')
                    vehColor = 'Red';
                else
                    vehColor = 'Green';
                end
                
                simBlockHandle = getSimulinkBlockHandle([simulinkModelName '/' vehName],true);
                set_param(simBlockHandle, 'ActorName', vehName, 'VehColor', vehColor, ...
                    'x', ['X{' num2str(nV) '}'], 'y', ['Y{' num2str(nV) '}'], 'psi', ['Psi{' num2str(nV) '}']);
            end
            
            set_param(simulinkModelName, 'StopTime', num2str(ceil(maneuverEndTime)));
            set_param(getSimulinkBlockHandle([simulinkModelName '/To Multimedia File' ],true), ...
                'outputFilename', [obj.videoFolder obj.simulationName '_3D.avi']);
            sim(simulinkModelName, 'SrcWorkspace','current');
            
        end
        
        %%% Exporting functions %%%
        
        function [] = saveToLaneChangeFolder(obj, figHandle, figName, enlargeFig)
            folderPath = ResultAnalysis.laneChangeImageFolder;
            if nargin < 4
                enlargeFig = false;
            end
            obj.saveToFolder(figHandle, folderPath, figName, enlargeFig);
        end
        
        function [] = saveToSafetyFolder(obj, figHandle, figName, enlargeFig)
            folderPath = obj.safetyImageFolder;
            if nargin < 4
                enlargeFig = false;
            end
            obj.saveToFolder(figHandle, folderPath, figName, enlargeFig);
        end
        
        function [] = saveToFolder(obj, figHandle, folderPath, ...
                figName, enlargeFig)

            if obj.saveResults
                if ~iscell(figName)
                    figName = {figName};
                end
                
                for iFig = 1:length(figHandle)
                    % Save in matlab format
                    saveas(figHandle(iFig), ['.\figures\' figName{iFig}]);

                    if nargin > 4 && enlargeFig
                        ResultAnalysis.enlargeFig(figHandle(iFig))
                    end

                    % Save in eps for latex use (or png for Microsoft 
                    % Office if needed)
                    if strcmp(obj.fileFormat, 'eps')
                        print(figHandle(iFig), ...
                            [folderPath figName{iFig}], ...
                            '-depsc', '-painters');
                    else
                        saveas(figHandle(iFig), ...
                            [folderPath figName{iFig} ...
                            '.' obj.fileFormat]);
                    end
                end
            end
        end
       
        function [latexTable] = saveMatlabTableToLatex(obj, resultTable)
            nRows = size(resultTable, 1);
            nCols = size(resultTable, 2);
            
            columnPositioning = repmat(' c ', 1, nCols);
            columnPositioning = sprintf('{%s }', ...
                columnPositioning(1:end-1));
            columnTitles = join(string(...
                resultTable.Properties.VariableNames), ' & ');
            columnTitles = insertAfter(columnTitles, '[', '$');
            columnTitles = insertBefore(columnTitles, ']', '$');
            
            tableHeader = sprintf('%s\n%s\n%s\n%s%s\n%s\n%s', ...
                '\begin{table*}[t]', ...
                ['\caption{Platoon Strategies Comparison. '...
                '\label{tab:results_table}}'], ...
                '\centering', ...
                '\begin{tabular}', columnPositioning, ...
                columnTitles, ...
                '\\ \hline');
            
            tableContents = ''; %strings(1, nRows);
            precisionPerCol = {'%d', '%d', '%.f', ...
                '%.1f', '%.1f', '%.1f', '%.1f', ...
                '%.f', '%.f', '%.f'};
            for r = 1:nRows
                line = strings(nCols, 1);
                strategy = resultTable.('LC Strategy')(r);
                isSynchronous = strcmpi(strategy{:}, 'synchronous');
                line(2) = strategy{:};
                if isSynchronous
                    line(1) = sprintf('\\multirow{3}{*}{%d}', ...
                        resultTable{r, 1}*3.6);
                end
                for c = 3:nCols
                    content = resultTable{r, c};
                    if isSynchronous
                        line(c) = num2str(content, precisionPerCol{c});
                    else
                        line(c) = [num2str(content, precisionPerCol{c}) ...
                            '\%'];
                    end
                end
                tableContents = sprintf('%s%s \\\\ \n', tableContents, ...
                    join(line, ' & '));
            end
            
            latexTable = sprintf('%s %s \\end{tabular} \n\\end{table*}', ...
                tableHeader, tableContents);
            
            if obj.saveResults
                filePath = sprintf('%s%s_%s.tex', ...
                    obj.platoonLCfolder, 'platoon', ...
                    resultTable.Properties.Description);
                fid = fopen(filePath,'w+');
                fprintf(fid, '%s', latexTable);
                fclose(fid);
            end
        end
            
        % [March 31 2021] Deprecated function
        function [] = exportToLatexTable(obj, relVel, strategies, timeCost,...
                accelCost, spaceTimeCost)
            tableHeader = sprintf('%s\n%s\n%s\n%s\n%s\n%s', '\begin{table}[!h]', ...
                '\caption{Efficiency Results. \label{tab:efficiency_results}}', ...
                '\centering', ...
                '\begin{tabular}{c | c | c | c | c }', ...
                ['\multirow{2}{*}{$\Delta v_L (km/h)$} & '...
                '\multirow{2}{*}{Strategy} & Maneuver  & '...
                'Acceleration & Reserved\\'], ...
                ['& & Duration $(s)$ & Cost $(m^2/s^3)$ & '...
                'Space-Time $(m.s)$\\ \hline \hline']);
            
            stratStr = {'Sync.', 'Ld. F.', 'L.V.F.'};
            tableContents =  '';
            for n = 1:length(relVel)
                for k = 1:length(strategies)
                    rvStr = '';
                    hlineStr = '\cline{2-5}';
                    if k==2
                        rvStr = num2str(relVel(n)*Vehicle.mpsToKmph);
                    end
                    if k==3
                        hlineStr = '\hline \hline';
                        if n==length(relVel)
                            hlineStr = '';
                        end
                    end
                    
                    tableContents = sprintf(['%s %s & %s & %.f & %.f & %.f '...
                        '\\\\ %s \n'], tableContents, rvStr, stratStr{k}, ...
                        timeCost(n, k), accelCost(n, k), ...
                        spaceTimeCost(n, k), hlineStr);
                end
            end
            tableContents = replace(tableContents, 'Inf', 'N/A');
            ltxTable = sprintf('%s %s \\end{tabular} \n\\end{table}', ...
                tableHeader, tableContents);
            
            if obj.saveResults
                fid = fopen(sprintf('./%s_efficiency_results.tex', 'platoon'),'w+');
                fprintf(fid, '%s', ltxTable);
                fclose(fid);
            end
        end
                
        %%% Special Setters %%%
        function [] = set.fileFormat(obj, value)
            if ~ischar(value) && ~isstring(value)
                warning('Property fileFormat must be a char array or string. Previous format (%s) kept.\n', ...
                    obj.fileFormat)
                return
            end
            
            if any(strcmp(ResultAnalysis.possibleFormats, value))
                obj.fileFormat = value;
            else
                possibleFormatsStr = sprintf('%s, ', ResultAnalysis.possibleFormats{:});
                warning('Property fileFormat must be one of %s. Previous format (%s) kept.\n', ...
                    possibleFormatsStr(1:end-2), obj.fileFormat)
            end
            
        end
        
    end
    
    methods (Static)
        
        %%% Performance measures %%%
        function [maneuverDuration, startTimeIdx, endTimeIdx] = ...
                timeToFinishManeuver(platoonVehs)
            %Computes time taken from the start of longitudinal adjustments
            %(gap generation) until and the moment the whole platoon is 
            %back to simple lane keeping in the destination lane
            
            % In all strategies, either first or last vehicle start and
            % finish the manenuver. Therefore, we only have to check for
            % their start and end times. Maneuver state zero indicates the
            % vehicle is lane keeping, i.e., it is the state both before
            % the lane change starts and after it ends.
            
            leader = platoonVehs.vehs(1);
            last = platoonVehs.vehs(end);
            
            leaderStartIdx = find(leader.lcState.data, 1, 'first');
            leaderEndIdx = find(leader.lcState.data, 1, 'last');
            if isempty(leaderEndIdx)
                    warning('%s did not finish the maneuver', ...
                        leader.name)
                    leaderEndIdx = length(leader.lcState.data);
            end
            lastStartIdx = find(last.lcState.data, 1, 'first');
            lastEndIdx = find(last.lcState.data, 1, 'last');
            if isempty(lastEndIdx)
                    warning('%s did not finish the maneuver', ...
                        last.name)
                    lastEndIdx = length(last.lcState.data);
            end
            
            startTimeIdx = min(leaderStartIdx, lastStartIdx);
            endTimeIdx = max(leaderEndIdx, lastEndIdx);
            simTime = leader.lcState.time;
            maneuverDuration = simTime(endTimeIdx) - simTime(startTimeIdx);

            % [3/16/21] Code below works with older simulink models which
            % defined vehicle lane changing states differently
%             lcStates = [platoonVehs.lcState.data];
%             lcStatesTime = platoonVehs.lcState.time; % different from vehs.simTime
%             
%             startIdxArray = zeros(platoonVehs.nv, 1);
%             endIdxArray = zeros(platoonVehs.nv, 1);
%             for n = 1:platoonVehs.nv
%                 startIdxArray(n) = find(platoonVehs.vehs(n).lcState.data ...
%                     == SimulinkVehicle.lcStateDict.longAdj, 1);
%                 endTimeIdx = find(lcStates(:, n) ...
%                     == SimulinkVehicle.lcStateDict.allDone, 1);
%                 if isempty(endTimeIdx)
%                     warning('Vehicle %s did not finish the maneuver', ...
%                         platoonVehs.vehs(n).name)
%                     endTimeIdx = length(lcStates(:,n));
%                 end
%                 endIdxArray(n) = endTimeIdx;
%             end
%             
%             startTime = lcStatesTime(min(startIdxArray));
%             endTime = lcStatesTime(max(endIdxArray));
%             maneuverDuration = (endTime - startTime);
            
        end
        
        function [lcDuration] = timeToFinishLC(platoonVehs)
            warning(['[3/16/21] Outdated method that works assuming state '...
            'definitions from older simulations. Adaptation to new code '...
            'is simple, but why use this metric instead of maneuver duration?'])
        
            lcStates = platoonVehs.lcState.data;
            lcStatesTime = platoonVehs.lcState.time; % different from vehs.simTime
            
            startIdxArray = zeros(platoonVehs.nv, 1);
            endIdxArray = zeros(platoonVehs.nv, 1);
            for n = 1:platoonVehs.nv
                startIdxArray(n) = find(lcStates(:, n) ...
                    == SimulinkVehicle.lcStateDict.longAdj, 1);
                endIdxArray(n) = find(lcStates(:, n) ... 
                    == SimulinkVehicle.lcStateDict.lcDone, 1);
            end
            
            startTime = lcStatesTime(min(startIdxArray));
            endTime = lcStatesTime(max(endIdxArray));
            lcDuration = (endTime - startTime);
            
        end
        
        function [totalAccelCost, itemizedAccelCosts] = ...
                accelCost(origLane, destLane)
            %accelCost Computes the sum of squared acceleration for all
            %vehicles involved in the lane change, including followers
            
            [~, startIdx, endIdx] = ...
                ResultAnalysis.timeToFinishManeuver(...
                origLane.getLaneChangingVehicleArray);
            
            simTime = origLane.vehs(1).simTime;
%             [~, startIdx] = min(abs(simTime-startTime));
%             [~, endIdx] = min(abs(simTime-endTime));
            
            fo = origLane.getVehByName('fo');
            fd = destLane.getVehByName('fd');
            platoonVehs = origLane.getLaneChangingVehicleArray();
            
            maneuverTime = simTime(startIdx:endIdx);
            foAccelCost = trapz(maneuverTime, fo.ax(startIdx:endIdx).^2);
            fdAccelCost = trapz(maneuverTime, fd.ax(startIdx:endIdx).^2);
            platoonAccelCost = zeros(1, platoonVehs.nv);
            for n = 1:platoonVehs.nv
                platoonAccelCost(n) = trapz(maneuverTime, ...
                    platoonVehs.vehs(n).ax(startIdx:endIdx).^2);
            end
            
            totalAccelCost = foAccelCost + fdAccelCost ...
                + sum(platoonAccelCost);
            
            itemizedAccelCosts = struct('fo', foAccelCost, ...
                'fd', fdAccelCost, 'platoon', sum(platoonAccelCost));
            
        end
        
        % [March 19, 2021] When refactoring and organazing the code, it was
        % easier to rewrite a spaceCost function than to change the
        % existing one below
        function [fdSpaceCost, platoonSpaceCost] = spaceCost(origLane, ...
                destLane)
            
            warning('Deprecated function - use computeSpaceCost instead')
            
            % [April 29, 2020] Note for future code improvment:
            % There may be a way of using simError.egReal to make the code
            % simpler.
            
            simTime = origLane.vehs(1).simTime;
            [~, startIdx, endIdx] = ...
                ResultAnalysis.timeToFinishManeuver();
            
%             [~, startIdx] = min(abs(simTime-startTime));
%             [~, maneuverEndIdx] = min(abs(simTime-maneuverEndTime));
            
            ld = destLane.getVehByName('ld');
            fd = destLane.getVehByName('fd');
            lo = origLane.getVehByName('lo');
            
            platoonVehs = origLane.getLaneChangingVehicleArray();

            % Find for each platoon vehicle: safe veh foll gap, lane change
            % gap and lane change ended index
            pnSafeGap = zeros(length(simTime), platoonVehs.nv);
            pnLCGap = zeros(length(simTime), platoonVehs.nv);
            pnLCEndIdx = zeros(platoonVehs.nv, 1);
            for n = 1:platoonVehs.nv
                pn = platoonVehs.vehs(n);
                pnSafeGap(:, n) = pn.x + pn.len + pn.h*pn.vx + pn.d0;
                pnLCGap(:, n) = pn.x + pn.len + pn.hLC*min(pn.vx0, fd.vx0) + pn.d0LC;
                lcStateTimeSeries = pn.lcState;
                pnLCEndTime = lcStateTimeSeries.time(...
                    find(lcStateTimeSeries.data>=SimulinkVehicle.lcStateDict.lcDone, 1));
                [~, pnLCEndIdx(n)] = min(abs(simTime-pnLCEndTime));
            end
            % Also get the safe veh foll gap for the follower in the
            % destinaiton lane
            fdSafeGap = fd.x + fd.len + fd.h*fd.vx + fd.d0;
            
            % Reserved space in front of platoon:
            % Before lane change: extra spacing to minimum of Lo and its 
            % necessary lc gap
            % After lane change: extra spacing to Ld (being closed) or
            % nothing if Ld is faster
            xReservedP1 = min(pnLCGap(startIdx:pnLCEndIdx(1), 1), ...
                lo.x(startIdx:pnLCEndIdx(1)));
            if ld.vx0 <= lo.vx0 % p1 will close the gap to ld
                xReservedP1 = [xReservedP1; ld.x(pnLCEndIdx(1)+1:endIdx, 1)];
            else % p1 won't close the gap to lc
                xReservedP1 = [xReservedP1; pnSafeGap(pnLCEndIdx(1)+1:endIdx, 1)];
            end
            p1ReservedSpace = xReservedP1 - pnSafeGap(startIdx:endIdx, 1);
            
            strategy = origLane.lcStrategy;
            switch strategy
                case 'synchronous'
                    % Interplatoon reserved space
                    interPlatoonReservedSpace = zeros(length(simTime), ...
                        platoonVehs.nv);
                    for n = 2:platoonVehs.nv
                        interPlatoonReservedSpace(:, n) = max(0, ...
                            platoonVehs.vehs(n-1).x - pnSafeGap(:, n));
                    end

                    % Reserved space ahead of Fd: the space between its
                    % minimum vf gap and the minimum of Ld's position and
                    % the necessary space for the platoon to merge. After
                    % platoon merges, we expected the reserved space to be
                    % close to zero but compute it just to verify.
                    xReservedFd = min(pnLCGap(startIdx:pnLCEndIdx(end), 1),...
                        ld.x(startIdx:pnLCEndIdx(end)));
                    xReservedFd = [xReservedFd; platoonVehs.vehs(end).x(pnLCEndIdx(end)+1:endIdx)];
                    fdReservedSpace = max(0, xReservedFd - fdSafeGap(startIdx:endIdx));
                    
                case 'leaderFirst'
                    % Interplatoon reserved space. Three stages:
                    % 1 - Extra spacing to preceding vehicle while the
                    % preceding vehicle hasn't change lanes
                    % 2 - Reserved extra spacing beyond minimum veh
                    % following space after preceding veh has changed lanes
                    % 3 - Gap closing stage, i.e., extra gap at the
                    % destination lane
                    interPlatoonReservedSpace = zeros(length(simTime), platoonVehs.nv);
                    for n = 2:platoonVehs.nv
                        interval1 = startIdx:pnLCEndIdx(n-1);
                        interPlatoonReservedSpace(interval1, n) = ...
                            max(0, platoonVehs.vehs(n-1).x(interval1) - pnSafeGap(interval1, n));
                        interval2 = pnLCEndIdx(n-1)+1:pnLCEndIdx(n);
                        xReservedPn = min(pnLCGap(interval2, n), lo.x(interval2));
                        interPlatoonReservedSpace(interval2, n) = ...
                            max(0, xReservedPn - pnSafeGap(interval2, n));
                        interval3 = pnLCEndIdx(n)+1:endIdx;
                        interPlatoonReservedSpace(interval3, n) = ...
                            max(0, platoonVehs.vehs(n-1).x(interval3) - pnSafeGap(interval3, n));
                    end

                    % Reserved space ahead of Fd: 
                    % First, the space between Fd's minimum vf gap and the 
                    % minimum of Ld's position and the necessary space for 
                    % p1 to merge.
                    % Next, the reserved space is the distance from Fd's
                    % safe gap to its current leader (p1, p2, till pN)
                    xReservedFd = zeros(length(simTime), 1);
                    xReservedFd(startIdx:pnLCEndIdx(1)) = ...
                        min(pnLCGap(startIdx:pnLCEndIdx(1), 1), ld.x(startIdx:pnLCEndIdx(1)));
                    
                    for n = 2:platoonVehs.nv
                        xReservedFd(pnLCEndIdx(n-1)+1:pnLCEndIdx(n)) = ...
                             platoonVehs.vehs(n-1).x(pnLCEndIdx(n-1)+1:pnLCEndIdx(n));
                    end
                    xReservedFd(pnLCEndIdx(end)+1:endIdx) = ...
                        platoonVehs.vehs(end).x(pnLCEndIdx(end)+1:endIdx);
                    fdReservedSpace = max(0, xReservedFd(startIdx:endIdx) ...
                        - fdSafeGap(startIdx:endIdx));
                    
                case 'lastFirst'                
                    % Interplatoon reserved space. Three stages:
                    % 1 - Extra spacing to preceding vehicle before
                    % changing lanes
                    % 2 - Reserved space in front of vehicle waiting for
                    % the preceding one to merge
                    % 3 - Gap closing stage, i.e., extra gap at the
                    % destination lane
                    interPlatoonReservedSpace = zeros(length(simTime), platoonVehs.nv);
                    for n = 2:platoonVehs.nv
                        interPlatoonReservedSpace(startIdx:pnLCEndIdx(n), n) = ...
                            max(0, platoonVehs.vehs(n-1).x(startIdx:pnLCEndIdx(n)) ...
                            - pnSafeGap(startIdx:pnLCEndIdx(n), n));
                        xReservedPn = ...
                            min(pnLCGap(pnLCEndIdx(n)+1:pnLCEndIdx(n-1), n-1), ...
                            ld.x(pnLCEndIdx(n)+1:pnLCEndIdx(n-1)));
                        interPlatoonReservedSpace(pnLCEndIdx(n)+1:pnLCEndIdx(n-1), n) = ...
                            max(0, xReservedPn ...
                            - pnSafeGap(pnLCEndIdx(n)+1:pnLCEndIdx(n-1), n));
                        interPlatoonReservedSpace(pnLCEndIdx(n-1)+1:endIdx, n) = ...
                            max(0, platoonVehs.vehs(n-1).x(pnLCEndIdx(n-1)+1:endIdx) ...
                            - pnSafeGap(pnLCEndIdx(n-1)+1:endIdx, n));
                    end

                    % Reserved space ahead of Fd: 
                    % Space between Fd's minimum vf gap and the 
                    % minimum of Ld's position and the necessary space for 
                    % p3 to merge.
                    xReservedFd = zeros(length(simTime), 1);
                    xReservedFd(startIdx:pnLCEndIdx(end)) = ...
                        min(pnLCGap(startIdx:pnLCEndIdx(end), end), ...
                        ld.x(startIdx:pnLCEndIdx(end)));
                    xReservedFd(pnLCEndIdx(end)+1:endIdx) = ...
                        platoonVehs.vehs(end).x(pnLCEndIdx(end)+1:endIdx);
                    fdReservedSpace = max(0, xReservedFd(startIdx:endIdx) ...
                        - fdSafeGap(startIdx:endIdx));
                    
                case 'leaderFirstInvert'
                    warning('TO DO')
            end
            
            
            fdSpaceCost = trapz(simTime(startIdx:endIdx), fdReservedSpace);
            p1SpaceCost = trapz(simTime(startIdx:endIdx), p1ReservedSpace);
            interPlatoonSpaceCosts = trapz(simTime(startIdx:endIdx), ...
                interPlatoonReservedSpace(startIdx:endIdx, :));
            platoonSpaceCost = p1SpaceCost +  sum(interPlatoonSpaceCosts);
        end
        
        function [origSpaceTimeCost, destSpaceTimeCost] = ...
                computeSpaceTimeCost(origLane, destLane, checkPlot)
            % Computes the space cost for platoon lane change 
            % @origLane: object of SimulinkVehicleArray class containing
            % the lane changing vehicles
            % @origLane: object of SimulinkVehicleArray class containing
            % the gap generating vehicle
            % @checkPlot: boolean to determine whether to plot the costs
            % over time. Used during tests to check function is working as
            % expected
            
            switch origLane.lcStrategy
                case 'synchronous'
                    [origSpaceCost, destSpaceCost] = ...
                        ResultAnalysis.synchronousSpaceCost(...
                        origLane, destLane);
                case 'leaderFirst'
                    [origSpaceCost, destSpaceCost] = ...
                        ResultAnalysis.leaderFirstSpaceCost(...
                        origLane, destLane);
                case 'lastFirst'
                    [origSpaceCost, destSpaceCost] = ...
                        ResultAnalysis.lastFirstSpaceCost(...
                        origLane, destLane);
                case 'leaderFirstInvert'
                    [origSpaceCost, destSpaceCost] = ...
                        ResultAnalysis.leaderFirstInvertSpaceCost(...
                        origLane, destLane);
                otherwise
                    warning(['Unknown strategy.'...
                        'Skipping space-time cost computation'])
            end
            
            % We want to look only at costs during the maneuver
            platoonVehs = origLane.getLaneChangingVehicleArray();
            [~, startIdx, endIdx] = ResultAnalysis.timeToFinishManeuver(...
                platoonVehs);
            origSpaceCost = origSpaceCost(startIdx:endIdx);
            destSpaceCost = destSpaceCost(startIdx:endIdx, :);
            
            maneuverTime = origLane.simTime(startIdx:endIdx);
            origSpaceTimeCost = trapz(maneuverTime, origSpaceCost);
            %%% TODO: remove the sum() after adjust all functions from
            %%% platoonSpaceCost->origSpaceCost
            destSpaceTimeCost = trapz(maneuverTime, destSpaceCost);
            
            if nargin > 2 && checkPlot
                vehNames = ['fd', {platoonVehs.vehs.name}];
                relVel = destLane.vehs(1).vx0 - origLane.vehs(1).vx0;
                fig = ResultAnalysis.plotCostOverTime(maneuverTime, ...
                    [origSpaceCost, destSpaceCost], vehNames);
                fig.Name = [origLane.lcStrategy ' ' num2str(relVel)];
            end
            
        end
        
        function [origSpaceCost, destSpaceCost] = ...
                synchronousSpaceCost(origLane, destLane)
            % Computes the space cost for platoon lane change using
            % synchronous strategy
            % @origLane: object of SimulinkVehicleArray class containing
            % the lane changing vehicles
            % @origLane: object of SimulinkVehicleArray class containing
            % the gap generating vehicle
            % @checkPlot: boolean to determine whether to plot the costs
            % over time. Used during tests to check function is working as
            % expected
            
            % pn, n > 1: space between pn's safe gap and pn's leader
            % Example (r is the reserved space):
            %   =pn=>|--gVF--|--r--|=pn-1=>
            
            if ~strcmpi(origLane.lcStrategy, 'synchronous')
                warning('%s\n%s', ['Platoon did not use the synchronous '...
                    'strategy'], 'Skipping space cost computation')
                origSpaceCost = 0;
                destSpaceCost = 0;
                return
            end
            
            simTime = origLane.simTime;
            origSpaceCost = zeros(length(simTime), 1);
            destSpaceCost = zeros(length(simTime), 1);
            platoonVehs = origLane.getLaneChangingVehicleArray();
            
            % fd cost
            fd = destLane.getVehByName('fd');
            p1 = platoonVehs.vehs(1);
            fdSpaceCost = ResultAnalysis.computeFdSpaceCost(fd, p1);
            destSpaceCost = destSpaceCost + fdSpaceCost;
            
            % p1 cost
            platoonSpaceCost = zeros(length(simTime), platoonVehs.nv);
            platoonSpaceCost(:, 1) = ResultAnalysis.computeP1SpaceCost(...
                origLane);
            
            % pn, n > 1, cost
            for n = 2:platoonVehs.nv
                pn = platoonVehs.vehs(n);
                platoonSpaceCost(:, n) = pn.computeErrors('gap');
            end
            
            pnLCEndIdx = pn.findLaneChangeEndTimeIdx();
            origSpaceCost(1:pnLCEndIdx) = ...
                sum(platoonSpaceCost(1:pnLCEndIdx, :), 2);
            destSpaceCost(pnLCEndIdx+1:end) = ...
                destSpaceCost(pnLCEndIdx+1:end)...
                + sum(platoonSpaceCost(pnLCEndIdx+1:end, :), 2);
        end
        
        function [origSpaceCost, destSpaceCost] = ...
                leaderFirstSpaceCost(origLane, destLane)
            % Computes the space cost for platoon lane change using
            % leaderFirst strategy
            % @origLane: object of SimulinkVehicleArray class containing
            % the lane changing vehicles
            % @origLane: object of SimulinkVehicleArray class containing
            % the gap generating vehicle
            % @checkPlot: boolean to determine whether to plot the costs
            % over time. Used during tests to check function is working as
            % expected
            
            % pn, n > 1: before lane chage, space between pn's safe gap 
            % and p(n-1), if p(n-1) still in origin lane, or gLC, if
            % p(n-1) has already changed lanes. After lane change, space
            % between its safe gap and p(n-1).
            % Examples (r is the reserved space):
            %   a) Before lane change, leader in origin lane:
            %      =p3=>|--gVF--|--r--|=p2=>
            %   b) Before lane change, leader in destination lane:
            %      =p2=>|--gVF--|---r---|      =lo=>
            %           |------gLC------|
            %   c) After lane change, both in destination lane:
            %      =p3=>|--gVF--|----r----|=p2=>
            
            if ~strcmpi(origLane.lcStrategy, 'leaderFirst')
                warning('%s\n%s', ['Platoon did not use the leaderFirst '...
                    'strategy.'], 'Skipping space cost computation')
                origSpaceCost = 0;
                destSpaceCost = 0;
                return
            end
            
            simTime = origLane.simTime;
            origSpaceCost = zeros(length(simTime), 1);
            destSpaceCost = zeros(length(simTime), 1);
            platoonVehs = origLane.getLaneChangingVehicleArray();
            
            % fd cost
            fd = destLane.getVehByName('fd');
            p1 = platoonVehs.vehs(1);
            fdSpaceCost = ResultAnalysis.computeFdSpaceCost(fd, p1);
            destSpaceCost = destSpaceCost + fdSpaceCost;
            
            % p1 cost
            platoonSpaceCost = zeros(length(simTime), platoonVehs.nv);
            platoonSpaceCost(:, 1) = ResultAnalysis.computeP1SpaceCost(...
                origLane);
            origSpaceCost = origSpaceCost + platoonSpaceCost(:, 1);
            
            % pn, n > 1,  cost
            for n = 2:platoonVehs.nv
                pn = platoonVehs.vehs(n);
                leader = pn.leader;
                % Before pn's leader has changed lanes and after pn 
                % has changes lanes (same formula for both cases)
                platoonSpaceCost(:, n) = pn.computeErrors('gap');
                
                % Time between pn's leader lane change and pn lane change
                pnSafeGap = pn.h*pn.vx + pn.d0;
                pnLaneChangeGap = pn.hLC*pn.vx + pn.d0LC;
                leaderLCEndIdx = leader.findLaneChangeEndTimeIdx();
                pnLCEndIdx = pn.findLaneChangeEndTimeIdx();
                platoonSpaceCost(leaderLCEndIdx+1:pnLCEndIdx, n) = ...
                    pnLaneChangeGap(leaderLCEndIdx+1:pnLCEndIdx)...
                    - pnSafeGap(leaderLCEndIdx+1:pnLCEndIdx);
                
                origSpaceCost(1:pnLCEndIdx) = ...
                    origSpaceCost(1:pnLCEndIdx)...
                    + platoonSpaceCost(1:pnLCEndIdx, n);
                destSpaceCost(pnLCEndIdx+1:end) = ...
                    destSpaceCost(pnLCEndIdx+1:end)...
                    + platoonSpaceCost(pnLCEndIdx+1:end, n);
            end
            
        end
        
        function [origSpaceCost, destSpaceCost] = ...
                lastFirstSpaceCost(origLane, destLane)
            % Computes the space cost for platoon lane change using
            % lastFirst strategy
            % @origLane: object of SimulinkVehicleArray class containing
            % the lane changing vehicles
            % @origLane: object of SimulinkVehicleArray class containing
            % the gap generating vehicle
            % @checkPlot: boolean to determine whether to plot the costs
            % over time. Used during tests to check function is working as
            % expected

            % pn, n > 1: before lane chage, space between pn's safe gap 
            % and p(n-1). After lane change, space between its safe gap and 
            % p(n-1) + gLC until p(n-1) changes lanes. Then, back to space
            % between its safe gap and p(n-1)
            % Examples (r is the reserved space):
            %   a) Before lane change:
            %      =p3=>|--gVF--|--r--|=p2=>
            %   b) After lane change, leader in origin lane, case 1
            %      =p3=>|--gVF--|------------r-----------|        =ld=>
            %                       =p2=>|------gLC------|
            %   c) After lane change, leader in origin lane, case 2
            %      =p3=>|--gVF--|---------r---------|        =ld=>
            %                       =p2=>|----g-----|=p1=>
            %   d) After lane change, both in destination lane:
            %      =p3=>|--gVF--|----r----|=p2=>
            
            if ~strcmpi(origLane.lcStrategy, 'lastFirst')
                warning('%s\n%s', ['Platoon did not use the lastFirst '...
                    'strategy.'], 'Skipping space cost computation')
                origSpaceCost = 0;
                destSpaceCost = 0;
                return
            end
            if origLane.vehs(1).vx0 > destLane.vehs(1).vx0
                warning('%s\n%s', ['LastFirst strategy being evaluated on '...
                    'a scenario where v_orig > v_dest.'], ...
                    'This might yield unexpected results')
            end
            
            simTime = origLane.simTime;
            origSpaceCost = zeros(length(simTime), 1);
            destSpaceCost = zeros(length(simTime), 1);
            platoonVehs = origLane.getLaneChangingVehicleArray();
            
            % fd cost
            fd = destLane.getVehByName('fd');
            pN = platoonVehs.vehs(end);
            fdSpaceCost = ResultAnalysis.computeFdSpaceCost(fd, pN);
            destSpaceCost = destSpaceCost + fdSpaceCost;
            
            % p1 cost
            platoonSpaceCost = zeros(length(simTime), platoonVehs.nv);
            platoonSpaceCost(:, 1) = ResultAnalysis.computeP1SpaceCost(...
                origLane);
            origSpaceCost = origSpaceCost + platoonSpaceCost(:, 1);
            
            % pn, n > 1, cost
            for n = 2:platoonVehs.nv
                pn = platoonVehs.vehs(n);
                leader = pn.leader;
                % Before pn has changed lanes and after pn's leader has 
                % changes lanes (same formula for both cases)
                platoonSpaceCost(:, n) = pn.computeErrors('gap');
                
                % Time between pn lane change and pn's leader lane change
                leaderLaneChangeGap = leader.hLC*leader.vx + leader.d0LC;
                pnLCEndIdx = pn.findLaneChangeEndTimeIdx();
                leaderLCEndIdx = leader.findLaneChangeEndTimeIdx();
                
                %%% MUST DISCUSS AND CHOOSE AMONG THESE TWO POSSIBILITIES
                leaderGap = leader.computeGap();
                platoonSpaceCost(pnLCEndIdx:leaderLCEndIdx, n) = ...
                    platoonSpaceCost(pnLCEndIdx:leaderLCEndIdx, n)...
                    + leader.len ...
                    + min(leaderGap(pnLCEndIdx:leaderLCEndIdx), ...
                    + leaderLaneChangeGap(pnLCEndIdx:leaderLCEndIdx));
%                 platoonSpaceCost(pnLCEndIdx:leaderLCEndIdx, n) = ...
%                     platoonSpaceCost(pnLCEndIdx:leaderLCEndIdx, n)...
%                     + leader.len ...
%                     + leaderLaneChangeGap(pnLCEndIdx:leaderLCEndIdx);

                origSpaceCost(1:pnLCEndIdx) = ...
                    origSpaceCost(1:pnLCEndIdx)...
                    + platoonSpaceCost(1:pnLCEndIdx, n);
                destSpaceCost(pnLCEndIdx+1:end) = ...
                    destSpaceCost(pnLCEndIdx+1:end)...
                    + platoonSpaceCost(pnLCEndIdx+1:end, n);
            end
            
        end
        
        function [origSpaceCost, destSpaceCost] = ...
                leaderFirstInvertSpaceCost(origLane, destLane)
            % Computes the space cost for platoon lane change using
            % leaderFirstInvert strategy

            % pn: before leader lane change, space between its safe gap 
            % and min(leader, pn + gLC). After leader lane change, space 
            % between its safe gap and min(lo, pn + gLC). After pn changes 
            % lances, space between its safe gap and p(n+1) + gLC  (might 
            % be negative right after lane change, we have to set zero as 
            % lower bound) until p(n+1) changes lanes. Then, back to space 
            % between its safe gap and p(n+1)
            %   a) Before leader lane change, lo far:
            %      =p2=>|--gVF--|---r---|=p1=>
            %   b) After leader lane change, pn still in origin lane:
            %      =p2=>|--gVF--|---r---|      =lo=>
            %           |------gLC------|
            %   c) After pn lane change, p_n+1 still in origin lane:
            %      =p1=>|--gVF--|----r----|      =ld=>
            %        =p2=>|------gLC------|
            %   c) After lane change, both in destination lane:
            %      =p1=>|--gVF--|----r----|=p2=>
            
            if ~strcmpi(origLane.lcStrategy, 'leaderFirstInvert')
                warning('%s\n%s', ['Platoon did not use the '...
                    'leaderFirstInvert strategy.'],...
                    'Skipping space cost computation')
                origSpaceCost = 0;
                destSpaceCost = 0;
                return
            end
            
            simTime = origLane.simTime;
            origSpaceCost = zeros(length(simTime), 1);
            destSpaceCost = zeros(length(simTime), 1);
            platoonVehs = origLane.getLaneChangingVehicleArray();
            
            % fd cost
            fd = destLane.getVehByName('fd');
            p1 = platoonVehs.vehs(1);
            fdSpaceCost = ResultAnalysis.computeFdSpaceCost(fd, p1);
            destSpaceCost = destSpaceCost + fdSpaceCost;
            
            % pn cost
            lo = origLane.getVehByName('lo');
            platoonSpaceCost = zeros(length(simTime), platoonVehs.nv);
            for n = 1:platoonVehs.nv
                pn = platoonVehs.vehs(n);
                % Before pn and its leader change lanes, the reserved space
                % is the error (zero for p1)
                if n == 1
                    leaderLCEndtIdx = 0;
                else
                    leader = pn.leader;
                    leaderLCEndtIdx = leader.findLaneChangeEndTimeIdx();
                    fromSafeGapToLeader = pn.computeErrors('gap');
                    platoonSpaceCost(1:leaderLCEndtIdx, n) = ...
                        max(0, fromSafeGapToLeader(1:leaderLCEndtIdx));
                end
                % Between leader lane change and pn lane change
                pnLCEndIdx = pn.findLaneChangeEndTimeIdx();
                pnSafeGap = pn.h*pn.vx + pn.d0;
                pnLaneChangeGap = pn.hLC*pn.vx + pn.d0LC;
                fromSafeToLaneChangeGap = pnLaneChangeGap - pnSafeGap;
                fromSafeGapToLeader = pn.computeErrors('gap', lo);
                platoonSpaceCost(leaderLCEndtIdx+1:pnLCEndIdx, n) = ...
                    min(...
                    fromSafeToLaneChangeGap(leaderLCEndtIdx+1:pnLCEndIdx), ...
                    fromSafeGapToLeader(leaderLCEndtIdx+1:pnLCEndIdx));
                
                if n < platoonVehs.nv
                    % Between pn lane change and follower lane change
                    follower = platoonVehs.vehs(n+1);
                    followerLCEndIdx = follower.findLaneChangeEndTimeIdx();
                    followerLaneChangeGap = follower.hLC*follower.vx ...
                        + follower.d0LC;
                    reservedSpaceForNextVehicle = ...
                        pn.computeErrors('gap', follower) + follower.len ...
                        + followerLaneChangeGap;
                    platoonSpaceCost(pnLCEndIdx+1:followerLCEndIdx, n) = ...
                        max(0, reservedSpaceForNextVehicle(...
                        pnLCEndIdx+1:followerLCEndIdx));
                    % After follower changes lanes
                    fromSafeGapToFollower = pn.computeErrors('gap', follower);
                    platoonSpaceCost(followerLCEndIdx+1:end, n) = ...
                        fromSafeGapToFollower(followerLCEndIdx+1:end);
                end
                
                origSpaceCost(1:pnLCEndIdx) = ...
                    origSpaceCost(1:pnLCEndIdx)...
                    + platoonSpaceCost(1:pnLCEndIdx, n);
                destSpaceCost(pnLCEndIdx+1:end) = ...
                    destSpaceCost(pnLCEndIdx+1:end)...
                    + platoonSpaceCost(pnLCEndIdx+1:end, n);
            end
            
        end
        
        function [fdSpaceCost] = computeFdSpaceCost(fd, laneChangingVeh)
            % Compute space cost for the follower in the destination lane
            
            % For the gap generating vehicle, we must consider
            % - Before the next vehicle changes lanes, space between 
            % the the safe gap and min(realLeader, laneChangingVeh + gLC). 
            % - After the vehicle changes lanes change, space between the 
            % safe gap and the new real leader
            % Examples (r is the reserved space):
            %   a) Before lane change, ld close:
            %      =fd=>|--gVF--|--r--|=ld=>
            %   b) Before lane change, ld far:
            %      =fd=>|--gVF--|--r---|    =ld=>
            %         =p1=>|----gLC----| =lo=>
            %   c) After lane change: like (a) but leader is pn
            
            % we can use fd's simulation error to the its real leader
            % (automatically switches from ld to pn) to help
            fdRealError = fd.simErrors.egReal;
            laneChangingVehLCGap = ...
                laneChangingVeh.hLC*laneChangingVeh.vx ...
                + laneChangingVeh.d0LC;
            laneChangingVehReservedSpace = ...
                fd.computeErrors('gap', laneChangingVeh) ...
                + laneChangingVeh.len + min(laneChangingVehLCGap, ...
                laneChangingVeh.computeGap());
            fdSpaceCost = max(0, min(fdRealError, ...
                laneChangingVehReservedSpace));
        end
        
        function [p1SpaceCost] = computeP1SpaceCost(origLane)
            % Compute space cost for the platoon leader when the platoon
            % leader is not responsible for generating gap for other
            % vehicles
            
            % For the platoon leader, we must consider: 
            % - Before lane change, the space between p1's safe gap 
            % and min(lo, p1 + gLC).
            % - After lane change, the space after p1's safe gap is
            % not reserved
            % Examples (r is the reserved space):
            %   a) Before lane change, lo close:
            %      =p1=>|--gVF--|--r--|=lo=>
            %   b) Before lane change, lo far:
            %      =p1=>|--gVF--|---r---|    =lo=>
            %           |------gLC------|
            
            if strcmpi(origLane.lcStrategy, 'leaderFirstInvert')
                warning('%s\n%s', ['computeP1SpaceCost called for a '...
                    'platoon using leaderFirstInvert strategy'], ...
                    'This yields incorrect results.')
            end
            
            p1 = origLane.getVehByName('p1');
            
            p1SafeGap = p1.h*p1.vx + p1.d0;
            p1LaneChangeGap = p1.hLC*p1.vx + p1.d0LC;
            fromSafeToLaneChangeGap = p1LaneChangeGap - p1SafeGap;
            fromSafeGapToLeader = p1.computeErrors('gap');
            p1SpaceCost = ...
                min(fromSafeToLaneChangeGap, fromSafeGapToLeader);
            laneChangeEndIdx = p1.findLaneChangeEndTimeIdx();
            p1SpaceCost(laneChangeEndIdx+1:end) = 0;
        end
        
        %%% Helper functions %%%
        function [newTable] = computeProportionalCosts(resultTable)
            %Produces a table in which costs are shown relative to the
            %synchronous maneuver cost for each relative velocity
            
            newTable = resultTable;
            
            % Column indices
            relVelIdx = contains(resultTable.Properties.VariableNames, ...
                'vel', 'IgnoreCase', true);
            strategyIdx = contains(resultTable.Properties.VariableNames, ...
                'strategy', 'IgnoreCase', true);           
            costsIdx = ~relVelIdx & ~strategyIdx;
            % Row indices
            synchronousIdx = strcmpi(...
                resultTable{:, strategyIdx}, 'synchronous');
            
            relVel = unique(resultTable{:, relVelIdx});
            
            for iRV = 1:length(relVel)
                currentRelVelIdx = resultTable{:, relVelIdx} == relVel(iRV);
                synchronousCostIdx =  currentRelVelIdx & synchronousIdx;
                otherStrategiesIdx = currentRelVelIdx & ~synchronousIdx;
                synchronousCosts = resultTable{synchronousCostIdx, costsIdx};
                otherStrategiesCosts = resultTable{otherStrategiesIdx, costsIdx};
                newTable{otherStrategiesIdx, costsIdx} = ...
                    (otherStrategiesCosts-synchronousCosts)./synchronousCosts...
                    *100;
            end
            
        end
        
        
        function [figArray] = plotCostBarGraph(resultTable, costName)
            % Generates a bar plot with different colors for each 'item' of
            % the cost.
            
            costIdx = contains(...
                resultTable.Properties.VariableNames, costName, ...
                'IgnoreCase', true);
            % We are already going to plot the individualized costs, no
            % need to show the total cost on top of that
            totalCostIdx = contains(...
                resultTable.Properties.VariableNames, 'total', ...
                'IgnoreCase', true);
            columnIdx = costIdx & ~totalCostIdx;
            
            columns = resultTable.Properties.VariableNames(columnIdx);
            possibleRelVel = unique(resultTable.('Rel Vel [m/s]'));
            nRV = length(possibleRelVel);
            figArray = gobjects(nRV, 1);
            for iRV = 1:nRV
                relVel = possibleRelVel(iRV);
                relVelIdx = resultTable.('Rel Vel [m/s]')==relVel;
                figArray(iRV) = figure('name', ['rel_vel_' ...
                    num2str(relVel)]);
                lcStrategies = resultTable.('LC Strategy')(relVelIdx);
                stratCategories = reordercats(categorical(lcStrategies), ...
                    lcStrategies);
                cost = zeros(sum(relVelIdx), length(columns));
                legendString = cell(length(columns), 1);
                for iC = 1:length(columns)
                    cost(:, iC) = resultTable.(columns{iC})(relVelIdx);
                    cellString = split(columns{iC});
                    legendString{iC} = cellString{1};
                end
                ylabelString = strjoin(cellString(2:end));
                bar(stratCategories, cost, 'stacked');
                ylabel(ylabelString, 'FontSize', 18);
                legend(legendString, 'FontSize', 18, ...
                    'Location', 'northeast', 'Orientation', 'horizontal');
            end 
        end
        
        function fig = plotCostOverTime(time, costs, vehNames)
            
            % It's harder to "play" with the plot when there are too many
            % samples, so we only get 1/10 of them.
            reductionFactor = 10;
            time = ResultAnalysis.subsample(time, ...
                reductionFactor);
            costs = ResultAnalysis.subsample(costs, ...
                reductionFactor);
            fig = figure; 
            grid on; hold on;
            plot(time, costs);
            legend(vehNames);
        end
        
        function xReduced = subsample(x, factor)
            idxToKeep = 1:factor:size(x, 1);
            xReduced = x(idxToKeep, :);
        end
        
        function drawArrow(axisHandle, x, y)
            warning('Not working yet')
            axisPos = axisHandle.Position;
            xLimits = axisHandle.XLim;
            yLimits = axisHandle.YLim;
            xNormalized = axisPos(1) + (x-xLimits(1))*axisPos(3)/(xLimits(2)-xLimits(1));
            yNormalized = axisPos(2) + (y-yLimits(1))*axisPos(4)/(yLimits(2)-yLimits(1));
            annotation('arrow', xNormalized, yNormalized);
        end
        
        function f2 = mergePlots(f1, f2)
            %mergePlots Adds the contents of f1 to f2
            lines = findobj(f1, 'type', 'line');
            copyobj(lines, findobj(f2, 'type', 'axes'));
        end
        
        function [] = enlargeFig(figHandle)
            %%% To make the figure big in my second home screen (check
            %%% what's the best for office computer) 
            mp = get(0, 'MonitorPositions');
            if size(mp, 1)>1 % there are two monitors
                bigScreenPos = mp(2, :);
                figHandle.Position = [bigScreenPos(1:2) bigScreenPos(3)/2 bigScreenPos(4)];
            end
            
%             figHandle.WindowState = 'maximized';
            % Making axes as big as possible
            figAxes = findobj(figHandle, 'type', 'axes');
            figLegends = findobj(figHandle, 'type', 'legend');
            for n = 1:length(figAxes)
                figAxes(n).FontSize = 18;
                figLegends(n).FontSize = 18;
                % Best legend location depends on each plot
                switch figAxes(n).Tag
                    case {'gap', 'e_y', 'e_v'}
                        figLegends(n).Location = 'northeast';
                    otherwise
                        figLegends(n).Location = 'southeast';
                end
            end
            
        end
        
        function [figArray] = figuresFromSubplots(figHandle)
            % figuresFromSubplots Returns an array with figure handles for 
            % each subplot of the original figure
            
            allAxes = findobj(figHandle, 'type', 'Axes');
            allLegends = findobj(figHandle, 'type', 'Legend');
            figArray = zeros(length(allAxes), 1);
            for n = 1:length(allAxes)
                figArray(n) = figure;
                copyobj([allAxes(n) allLegends(n)], figArray(n));
                % Make the axes occupy the whole figure
                newFigAxes = findobj(figArray(n), 'type', 'axes');
                set(newFigAxes, 'OuterPosition', [0, 0, 1, 1]);
            end
        end

        %%% Legacy plotting functions - not sure which simulations call
        %%%these so I'll keep them around just in case [April 2, 2020]
        function [statesFig, inputsFig] = plotBicycleModelKinematics(t, q, ...
                u, title_string, legends)
            %plot_kinematics Plot kinematic variables contained in state vector x(t):
            % jerk(t), a(t), v(t) and s(t).
            % x(t) is t by k by v where t is the number of time samples, k is the
            % number of kinematic variables and v is the number of vehicles.
            
            if exist('title_string', 'var')==1
                statesFig = figure('Name', title_string{1});
                inputsFig = figure('Name', title_string{2});
            else
                statesFig = figure();
                inputsFig = figure();
            end
            
            nv = size(q, 2); % number of variables: from 1 (only position) to 4 (including jerk)
            kinematic_variables = {'x[m]', 'y[m]', 'v[m/s]', '\theta [rad]'};
            inputs = {'a[m/s^2]', '\delta [rad]'};
            
            figure(statesFig)
            for k = 1:nv
                subplot(nv, 1, k);
                y = squeeze(q(:, k, :));
                plot(t, y, 'LineWidth', 1.5); grid on;
                xlabel('t');
                ylabel(kinematic_variables{k});
                ymin = min(y(:));
                ymax = max(y(:));
                if ymin~=ymax
                    ylim([ymin-0.1*(ymax-ymin) ymax+0.1*(ymax-ymin)]);
                end
                
                set(gca,'fontsize',14)
                if ~isempty(legends)
                    legend(legends, 'Location', 'southeast');
                    
                end
                lgd.FontSize = 14;
            end
            
            figure(inputsFig)
            nu = size(u, 2);
            for k = 1:nu
                subplot(nu, 1, k);
                y = squeeze(u(:, k, :));
                plot(t, y, 'LineWidth', 1.5); grid on;
                xlabel('t');
                ylabel(inputs{k});
                ymin = min(y(:));
                ymax = max(y(:));
                if ymin~=ymax
                    ylim([ymin-0.1*(ymax-ymin) ymax+0.1*(ymax-ymin)]);
                end
                set(gca,'fontsize',14)
                if ~isempty(legends)
                    legend(legends, 'Location', 'southeast');
                end
                lgd.FontSize = 14;
            end
            
            
        end
        
        function [h] = plotKinematics(t, x, v_length, title_string, legends)
            %plot_kinematics Plot kinematic variables contained in state vector x(t):
            % jerk(t), a(t), v(t) and s(t).
            % x(t) is t by k by v where t is the number of time samples, k is the
            % number of kinematic variables and v is the number of vehicles.
            
            warning('Function moved to the ResultPlotter class')
            
            if exist('title_string', 'var')==1
                h = figure('Name', title_string);
            else
                h = figure();
            end
            
            nv = size(x, 2); % number of variables: from 1 (only position) to 4 (including jerk)
            kinematic_variables = {'gaps[m]', 'v[m/s]', 'a[m/s^2]', 'jerk[m/s^3]'};
            
            gaps = -diff(x(:,1,:), 1, 3)-v_length;
            
            for k = 1:nv
                subplot(nv, 1, k);
                if k ~= nv
                    y = squeeze(x(:, nv-k+1, :));
                else
                    y = squeeze(gaps);
                    legends = legends(2:end); % not proper coding
                end
                plot(t, y, 'LineWidth', 1.5); grid on;
                xlabel('t');
                ylabel(kinematic_variables{nv-k+1});
                ymin = min(y(:));
                ymax = max(y(:));
                if ymax-ymin>0.5
                    ylim([ymin-0.1*(ymax-ymin) ymax+0.1*(ymax-ymin)]);
                else
                    ylim([ymin-1 ymax+1]);
                end
                lgd = legend(legends, 'Location', 'southeast');
                set(gca,'fontsize',14)
                lgd.FontSize = 14;
            end
            
            % Plot gaps if at least two trajectories are given
            % if size(x, 3)>1
            %     yyaxis right
            %     gap = x(:, 1, 1)-x(:, 1, 2)-v_length;
            %     plot(t, gap, '--k', 'LineWidth', 1.5); grid on;
            %     ylim([min([0, 1.1*min(gap)]) 1.1*max(gap)]);
            %     ylabel('gap(t)')
            %     legend([legends(:)', {'gap'}], 'Location', 'southeast')
            % end
            
        end
        
    end
    
        
end

