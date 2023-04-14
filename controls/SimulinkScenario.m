classdef SimulinkScenario < handle
    %SimulinkScenario Methods to run different simulink scenarios
    
    properties
        simModelName
        nVehicles
        vehsPerLane % array with nLanes elements:
        % leftmost lane at element 1, rightmost at element nLanes
        plotter
        laneWidth = 3.6;
        sampling = 0.1; %used to create reference signals to simulink
        saveResults = false;
        resultsFolder = './numerical_results/';
        simulinkModelsFolder = './simulink_models/';
    end
    
    properties (Constant)
        % States have slightly different names in matlab code and simulink
        % models. The dictionary below is from Matlab to Simulink
        stateNameDict = struct('x', 'X', 'y', 'Y', 'vx', 'Xdot', ...
            'ax', 'xddot', 'psi', 'psi', 'u', 'u', 'delta', 'delta');
    end
    
%     properties (Access = private)
%         imgFolder = 'G:/My Drive/Lane Change/images/';
%     end
    
    methods
%         function obj = SimulinkScenario(saveResults)
%             %SimulinkScenario Set the simulink model and inform how many
%             %vehicles are there
% 
%         end
        

        %%% ACC SIMULATIONS %%%
        % NOTE: none of the pure longitudinal simulations have been
        % adapted to new configurations
        function [platoon, desVel] = accMaxBrake(obj, accPoles, velCtrlPoles) 
            %obj, accPoles, velCtrlPoles, relVel
            obj.simModelName = 'accTests';
            scenario = 'maxBrake';
            ka = -1;
            vehNames = {'p1', 'p2', 'p3', 'p4', 'p5'};
            obj.vehsPerLane = length(vehNames);
            finalT = 25;
            % Run
            [platoon, desVel] = obj.runACCSimulation(scenario, accPoles, velCtrlPoles, ka, [], vehNames, finalT);
            
            % Saving results
            if obj.saveResults
                save([obj.resultsFolder obj.simModelName  '_' scenario '_results'], ...
                    'platoon');
            end
            
        end
        
        function [dataStruct] = accZeroToMaxToZero(obj, poles)
            obj.simModelName = 'accTests';
            scenario = 'zeroToMaxToZero';
            ka = -1;
            % Run
            dataStruct = obj.runACCSimulation(poles, scenario, ka, []);
            % Plot
            figs = obj.plotter.plotDataStruct(dataStruct, 'ACC');
            % Save
            plotNames = {'states', 'errors'};
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('ACC_%s_%s', ...
                    scenario, plotNames{k}))
                end
            end
        end
        
        function [dataStruct] = caccMaxBrake(obj, poles, ka)
            obj.simModelName = 'accWithCommsTests';
            scenario = 'maxBrake';
            % Run
            dataStruct = obj.runACCSimulation(poles, scenario, ka, 0);
            % Plot
            figs = obj.plotter.plotDataStruct(dataStruct, 'CACC');
            % Save
            plotNames = {'states', 'errors'};
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('CACC_%s_%s', ...
                    scenario, plotNames{k}))
                end
            end
        end
        
        function [dataStruct] = caccZeroToMaxToZero(obj, poles, ka)
            obj.simModelName = 'accWithCommsTests';
            scenario = 'zeroToMaxToZero';
            % Run
            dataStruct = obj.runACCSimulation(poles, scenario, ka, 0);
            % Plot
            figs = obj.plotter.plotDataStruct(dataStruct, 'CACC');
            % Save
            plotNames = {'states', 'errors'};
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('CACC_%s_%s', ...
                    scenario, plotNames{k}))
                end
            end
        end
        
        function [dataStruct] = caccFilterMaxBrake(obj, poles, ka)
            obj.simModelName = 'accWithCommsTests';
            scenario = 'maxBrake';
            % Run
            dataStruct = obj.runACCSimulation(poles, scenario, ka, 1);
            % Plot
            figs = obj.plotter.plotDataStruct(dataStruct, 'CACCwFilter');
            % Save
            plotNames = {'states', 'errors'};
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('CACCfilter_%s_%s', ...
                    scenario, plotNames{k}))
                end
            end
        end
        
        function [dataStruct] = caccFilterZeroToMaxToZero(obj, poles, ka)
            obj.simModelName = 'accWithCommsTests';
            scenario = 'zeroToMaxToZero';
            % Run
            dataStruct = obj.runACCSimulation(poles, scenario, ka, 1);
            % Plot
            figs = obj.plotter.plotDataStruct(dataStruct, 'CACCwFilter');
            % Save
            plotNames = {'states', 'errors'};
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('CACCfilter_%s_%s', ...
                    scenario, plotNames{k}))
                end
            end
        end
        
        function [vehArray] = caccBasics(obj, ctrlParams, deltaX0)
            [vehArray] = prepareCACCSimulation(obj, ctrlParams, deltaX0);
            obj.runAndStoreOutput(vehArray);
        end
        
        %%% LANE CHANGE SIMULATIONS %%%
        function [vehArrays] = gapGeneration(obj, accPoles, velCtrlPoles, ...
                relVel, scenario)
                        
            fprintf('Running model %s_%s; rel velocity: %d\n', 'gap_generation', scenario, relVel);
           
            [vehArrays] = obj.prepareGapGenerationSimulation(accPoles, velCtrlPoles, relVel, scenario);
            obj.runAndStoreOutput(vehArrays);
            
            % Saving results
            destLane = vehArrays(1);
            origLane = vehArrays(2);
            if obj.saveResults
                save([obj.resultsFolder obj.simModelName '_rel_vel_' num2str(relVel) '_results'], ...
                    'destLane', 'origLane');
            end
            
            % Some plots for analysis
%             origLane.plotSimErrors(lcVehs);
%             origLane.plotStates({'gap', 'velocity', 'u'},[lcVehs, 'Fo']);
%             destLane.plotStates({'gap', 'velocity', 'u'});

        end
           
        function [vehArrays] = fullManeuver(obj, ctrlParams, simConfig)
            
            % Scenario parameters
            relVel = simConfig.relVel;
            hasTruck = simConfig.isTruckLaneChange;
            
            if simConfig.isPlatoon
                scenario = 'platoon';
                lcStrategy = simConfig.lcStrategy;
                if ischar(lcStrategy)
                    lcStrategy = {lcStrategy};
                end
            else
                scenario = 'single_veh';
                lcStrategy = {'synchronous'};
            end
            
            obj.simModelName = ['full_maneuver_' scenario];
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                modelFullPath = [obj.simulinkModelsFolder obj.simModelName];
                load_system(modelFullPath);
            end
            set_param(obj.simModelName,'FastRestart','off')
            
            nRV = length(relVel);
            nLCS = length(lcStrategy);

            vehArrays(nRV*nLCS, 2) = SimulinkVehicleArray();
            writer = VehicleDataWriter('full_maneuver', scenario);
            isVerbose = true;
            for iLCS = 1:nLCS
                if nRV > 1
                    set_param(obj.simModelName, 'FastRestart', 'on');
                end
                for iRV = 1:nRV
                    if strcmpi(lcStrategy{iLCS}, 'lastFirst') ...
                            && relVel(iRV) < 0
                        continue
                    end
                    try
                        fprintf(['Running model %s_%s\n\trel velocity: '...
                            '%d\n\tstrategy: %s\n'], 'full_maneuver', ...
                            scenario, relVel(iRV), lcStrategy{iLCS});
                        vehArrays((iLCS - 1)*nRV + iRV, :) = ...
                            obj.prepareLCSimulation(hasTruck, ...
                            ctrlParams, relVel(iRV), scenario, ...
                            lcStrategy{iLCS});

                        obj.runAndStoreOutput(...
                            vehArrays((iLCS - 1)*nRV + iRV, :));
                    catch ME
                        warning(['Error during simulation. '...
                            'Moving on to the next one'])
                        warning(ME.message)
                        continue
                    end
                    % Saving results
                    try
                    if obj.saveResults
                        destLane = vehArrays((iLCS - 1)*nRV + iRV, 1);
                        origLane = vehArrays((iLCS - 1)*nRV + iRV, 2);
                        writer.setSimulationName(hasTruck, relVel(iRV), ...
                            lcStrategy{iLCS});
                        writer.writeVehiclesPerLane(...
                            origLane, destLane, isVerbose);
%                         if contains(scenario, 'single_veh')
%                             % To match paper descriptions
%                             origLane.getVehByName('p1').name ='E'; 
%                         end
%                         if hasTruck
%                             truckString = '_truckLC';
%                         else
%                             truckString = '';
%                         end
%                         fileName = sprintf(...
%                             '%s%s%s_rel_vel_%d_strategy_%s_results',...
%                             obj.resultsFolder, obj.simModelName, ...
%                             truckString, relVel(iRV), lcStrategy{iLCS});
%                         save(fileName, 'destLane', 'origLane');
%                         fprintf(...
%                             '%s_rel_vel_%d_strategy_%s_results saved\n',...
%                             obj.simModelName, relVel(iRV), lcStrategy{iLCS});
                    end
                    catch ME
                        warning(['Following error when trying to save '...
                            'results:'])
                        warning(ME.message)
                        prompt = ['Do you want to proceed to the next '...
                            'simulation anyway? [y/n]'];
                        userAnswer = input(prompt);
                        if ~strcmpi(userAnswer, 'y')
                            set_param(obj.simModelName,'FastRestart','off')
                            return
                        end
                    end
                    % Main plot:
                    try
                        origLane = vehArrays((iLCS - 1)*nRV + iRV, 2);
                        laneChangingArray = ...
                            origLane.getLaneChangingVehicleArray();
                        laneChangingArray.plotStatesAllVehs(...
                            {'y', 'gap', 'velocity', 'u'});
                    catch ME
                        warning('Error when trying to plot results')
                        warning(ME.message)
                    end
                end
                set_param(obj.simModelName,'FastRestart','off')
            end
            
        end
        
        function [allVehArrays] = ggRunAndSaveAll(obj, accPoles, ...
                velCtrlPoles)
            relVel = [0, -4, 4];
            scenario = {'single_veh', 'platoon'};
            
            allVehArrays = cell(length(scenario), length(relVel));
            for sc = 1:length(scenario)
                for k = 1:length(relVel)
                    
                    vehArrays = obj.gapGeneration(accPoles, velCtrlPoles, relVel(k), scenario{sc});
                    
                    allVehArrays{sc, k} = vehArrays;
                end
            end
            
        end
        
        function [allVehArrays] = fullManeuverPlatoonRunAll(obj, ...
                vehFollPoles, velCtrlPoles, latPoles)
            
            input('Change all results to rel vel = +-5?');
            
            relVel = [0, -4, 4];
            
            scenario = 'platoon';
            strategies = [1, 2, 3];
            
            obj.simModelName = ['full_maneuver_' scenario];
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                modelFullPath = [obj.simulinkModelsFolder obj.simModelName];
                load_system(modelFullPath);
            end
            set_param(obj.simModelName,'SimulationMode','accelerator');
            
            allVehArrays = obj.fullManeuver(vehFollPoles, ...
                velCtrlPoles, latPoles, relVel, scenario, strategies);
            
            set_param(obj.simModelName,'SimulationMode','normal');
        end
        
        function [vehArrays] = fullManeuverNewStrategy(obj, vehFollPoles, ...
                velCtrlPoles, latPoles, relVel, scenario, strategy)
            
            % Situation where all platoon vehicles adjust their gaps
            % simultaneously but respecting a minimum speed.
            warning('No longer pursuing this possibility')
            % [March 8, 2020] To make this work again requires either 
            %creating a prepareAlternativeLCSimulation method or including 
            %an extra parameter to the current prepareLCSimulation method
            % The property below redefined inside the following function -
            % but that's the first step to make this method work again (if
            % ever needed)
            obj.simModelName = 'full_maneuver_platoon_new_strategy'; 
            
            vehArrays = obj.prepareLCSimulation(vehFollPoles, velCtrlPoles, ...
                latPoles, relVel, scenario, strategy);
            
        end
               
        function [platoon, desVel] = runACCSimulation(obj, scenario, ...
                accPoles, velCtrlPoles, ka, accelFilter, vehNames, finalT) 
            %(scenario, accPoles, velCtrlPoles, ka, vehNames, finalT);
            
            warning(['The simulation shows our controller is string stable, ' ...
                'but it has errors (namely speeds go negative)'])
            % Scenario parameters
            % Leader desired speed
            initVel = 25; %[m/s], ~ 60 [mph]
            vehType = 'PV';
            
            fprintf('Running model %s, scenario %s \n', ...
                obj.simModelName, scenario);
                       
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                folder = 'old_simulink_simulations\';
                modelFullPath = [folder obj.simModelName];
                load_system(modelFullPath);
            end    
            
            % Leader desired speed
            switch scenario
                case 'maxBrake'
                    desVel = obj.maxBrakeVelProfile(initVel, finalT);
                case 'zeroToMaxToZero'
                    disp('Here''s where it won''t work :)');
                    desVel = obj.zeroToMaxToZeroVelProfile(initVel, ...
                        maxAcc, maxJerk);
                    initVel = desVel(1, 2);
                otherwise
                    error('Unkown scenario')
            end
            
            % Create vehicles
            y0 = 0;
            platoon = SimulinkVehicleArray(obj.vehsPerLane, ...
                obj.simModelName, vehNames, vehType);
            platoon.createHomogeneousSSPlatoon(initVel, y0, accPoles, ...
                velCtrlPoles)
                        
            % Controller gains
%             controller = Controller();
            if ka <= 0 % no leader acceleration feedforward
%                 controller.setGains('ACC PID', accPoles, h);
%                 [Kd, Kp, Ki] = obj.accGain(poles, h);
            else % const gain leader acceleration feedforward
                if accelFilter
%                     tau = h;
                else
%                     tau = 0;
                end
                error(['Code of SimulinkVehicle and SimulinkVehicleArray '...
                    'classes not ready for CACC']);
%                 controller.setGains('CACC PID', [accPoles, ka], h);
            end
            
            % Set parameters in simulink simulation
            platoon.setSimParams();
%             % TODO set speed properly
%             set_param(getSimulinkBlockHandle([obj.simModelName '/v0dest' ],true), ...
%                 'Value', num2str(desVelDest));
            
            % Run simulation
            set_param(obj.simModelName, 'StopTime', num2str(finalT));
            simOut = sim(obj.simModelName, 'SrcWorkspace','current');
            
            % Get results
            simTime = simOut.tout;
            platoon.setSimTime(simTime);
            for nV = 1:obj.vehsPerLane
                currentVeh = platoon.vehs(nV);
%                 currentVeh.simTime = simTime;
                currentVeh.states = simOut.find(['states_' lower(currentVeh.name)]);
            end
        end
        
        function [vehArray] = prepareCACCSimulation(obj, ctrlParams, ...
                deltaX0)
            %preparesLCSimulation Sets all the parameters for  
            %'CACC_3D_view' Simulink simulation
            
            % Simulation name
            obj.simModelName = 'CACC';
            
            % Get controllers parameters
            vehFollCtrlParams = ctrlParams.vehFollCtrlParams;
            velCtrlPoles = ctrlParams.velCtrlPoles;
            
            % Set some scenario parameters
            finalT = 100;
            vehNames = {'L', 'F'};
            vehType = 'PV';
            simplifiedVehs = vehNames;
            obj.vehsPerLane = length(vehNames);
            
            baseSpeed = 20; % [m/s] ~ 60 mph
            initVel = baseSpeed;
            maxVel = 25; % [m/s] used to compute safe distances
            
            % Simulink settings (no longer necessary on 03/24/21)
%             clear SimulinkVehicleArray
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                modelFullPath = [obj.simulinkModelsFolder obj.simModelName];
                load_system(modelFullPath);
            end
            
            % Create vehicles
            y0 = 0;

            vehArray = SimulinkVehicleArray(obj.vehsPerLane, ...
                obj.simModelName, vehNames, vehType);
            vehArray.addComms();
            vehArray.simplifyVehicles(simplifiedVehs);
            vehArray.createHomogeneousSSPlatoon(initVel, y0, maxVel, vehFollCtrlParams, velCtrlPoles);
            
            % Put the leader further ahead of the follower
            L = vehArray.vehs(1);
            L.x0 = L.x0+deltaX0;
            
            % Set vehicle parameters in simulink simulation
            vehArray.setSimParams()
            
            % Set remaining simulation time
            set_param(obj.simModelName, 'StopTime', num2str(finalT));
            
        end
        
        function [vehArrays] = prepareGapGenerationSimulation(obj, ...
                vehFollPoles, velCtrlPoles, relVel, scenario)
            %runGapGenerationSimulation Sets all the parameters for 
            %'gap_generation_...' Simulink
            
            obj.simModelName = ['gap_generation_' scenario];
            
            switch scenario
                case 'single_veh'        
                    lcVehs = {'p1'};
                    finalT = 50;
                case 'platoon'
                    lcVehs = {'p1', 'p2', 'p3'};
                    finalT = 70;
                otherwise
                    error('Unknown gap generation scenario');
            end
            vehNames = {{'Ld', 'Fd'}, ['Lo', lcVehs, 'Fo']};
            vehType = 'PV';
            obj.vehsPerLane = [length(vehNames{1}), length(vehNames{2})];
            
            % Scenario parameters
            % Leader desired speed
            baseSpeed = 25; % ~ 60 mph
            estimatedMaxVel = 30; % used to compute safe distances
            initVel = [baseSpeed+relVel; baseSpeed]; %[m/s]
%             finalT = 25;
            lcStartTime = 5; % [s]
            nLanes = length(obj.vehsPerLane);
            
            % Simulink settings
%             clear SimulinkVehicleArray
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                modelFullPath = [obj.simulinkModelsFolder obj.simModelName];
                load_system(modelFullPath);
            end           
            
            desVelDest = initVel(1); %obj.constVelProfile(initVel(1), finalT);
            desVelOrig = initVel(2); %obj.constVelProfile(initVel(2), finalT);           
            
            % Create vehicles
            vehArrays(nLanes) = SimulinkVehicleArray();
            y0 = obj.laneWidth*(nLanes-1):-obj.laneWidth:0;
            
            for n = 1:nLanes
                vehArrays(n) = SimulinkVehicleArray(obj.vehsPerLane(n), ...
                    obj.simModelName, vehNames{n}, vehType);
                vehArrays(n).createHomogeneousSSPlatoon(initVel(n), y0(n), estimatedMaxVel, vehFollPoles, velCtrlPoles)
            end
                       
            % In this scenario, we want the gap from Ego (or p1) to Ld to 
            % be (almost) the steady state one by the time the lane change 
            % starts - if it's exactly the same, some plots are not as
            % interesting
            xLd = vehArrays(1).getVehByName(vehNames{1}{1}).x0 + initVel(1)*lcStartTime;
            xEgo = vehArrays(2).getVehByName(vehNames{2}{1}).x0 + initVel(2)*lcStartTime;
            vehArrays(1).shiftVehArray(xEgo-xLd-1);
            
            % Set parameters in simulink simulation
            for n = 1:nLanes
                vehArrays(n).setSimParams()
            end
            set_param(getSimulinkBlockHandle([obj.simModelName '/Compare To lcStartTime' ],true), ...
                'const', num2str(lcStartTime));
            set_param(getSimulinkBlockHandle([obj.simModelName '/v0dest' ],true), ...
                'Value', num2str(desVelDest));
            set_param(getSimulinkBlockHandle([obj.simModelName '/v0orig' ],true), ...
                'Value', num2str(desVelOrig));
            set_param(obj.simModelName, 'StopTime', num2str(finalT));
            
        end
        
        function [vehArrays] = prepareLCSimulation(obj, hasTruck, ...
                ctrlParams, relVel, scenario, lcStrategy)
            %preparesLCSimulation Sets all the parameters for  
            %'full_maneuver_...' Simulink simulation, which includes gap 
            %generation and lane change
            
            % Get controllers parameters
            vehFollCtrlParams = ctrlParams.vehFollCtrlParams;
            velCtrlPoles = ctrlParams.velCtrlPoles;
            latPoles = ctrlParams.latCtrlParams;
            hasAccelFeedback = ctrlParams.hasAccelFeedback;
            
            % If this method was called by some function outside this class
            if isempty(obj.simModelName)
                obj.simModelName = ['full_maneuver_' scenario];
            end
            
            % Set some scenario parameters and perform basic checks
            if ~any(strcmpi(SimulinkVehicle.possibleStrategies, lcStrategy))
                warning(['Unknown platoon lane change strategy. '...
                    'Setting strategy to synchronous']);
                lcStrategy = 'synchronous';
            end
            if contains(scenario, 'platoon')
                lcVehs = {'p1', 'p2', 'p3'};
                if hasTruck
                    finalT = 400;
                else
                    finalT = 300;
                end
                if strcmpi(lcStrategy, 'synchronous')
                    finalT = finalT + 50;
                end
                if strcmpi(lcStrategy, 'lastFirst') && relVel < 0
                    warning(['Stragey lastFirst should only be used '...
                        'when v_dest >= v_orig (positive relVel'])
                end
            elseif contains(scenario, 'single_veh')
                lcVehs = {'p1'};
                lcStrategy = 'synchronous';
                finalT = 90;
            else
                error('Unkown scenario');
            end
            
            % Set vehicle names and types
            vehNames{1} = {'Ld', 'Fd'};
            vehNames{2} = ['Lo', lcVehs, 'Fo'];
            vehTypes = cell(2, 1);
            for iLane = 1:(length(vehNames))
                vehTypes{iLane} = cell(1, length(vehNames{iLane}));
                for iVeh = 1:length(vehNames{iLane})
                    vehTypes{iLane}{iVeh} = 'PV';
                end
            end
            if hasTruck
                for iVeh = 2:length(vehNames{2})-1
                    vehTypes{2}{iVeh} = 'HDV';
                end
            end
            
            % Only Fd and lcVehs need the full controller. This speeds up
            % simulation
            simplifiedVehs{1} = setdiff(vehNames{1},{'Fd'},'stable');
            simplifiedVehs{2} = setdiff(vehNames{2}, lcVehs,'stable');
            
            obj.vehsPerLane = [length(vehNames{1}), length(vehNames{2})];

            baseSpeed = 20; % [m/s] ~ 60 mph
            initVel = [baseSpeed+relVel; baseSpeed];
            estimatedMaxVel = 30; % [m/s] used to compute safe distances
            laneChangeVel = min(initVel); % baseSpeed if testing robustness 
            % or min(initVel) for easier case
            lcStartTime = 2; % [s]
            nLanes = length(obj.vehsPerLane);
            
            % Simulink settings
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                modelFullPath = [obj.simulinkModelsFolder obj.simModelName];
                load_system(modelFullPath);
            end
            
            % Center of each lane 
            yDest = obj.laneWidth;
            yOrig = 0;
            
            % Create vehicles
            vehArrays(nLanes) = SimulinkVehicleArray();
            y0 = [yDest, yOrig];
            
            for iLane = 1:nLanes
                vehArrays(iLane) = SimulinkVehicleArray(...
                    obj.vehsPerLane(iLane), obj.simModelName, ...
                    vehNames{iLane}, vehTypes{iLane}, lcStrategy);
%                 vehArrays(iLane).addActuatorLag(actuatorLag);
                if hasAccelFeedback
                    vehArrays(iLane).addComms();
                end
                vehArrays(iLane).simplifyVehicles(simplifiedVehs{iLane});
                vehArrays(iLane).createHomogeneousSSPlatoon(...
                    initVel(iLane), y0(iLane), estimatedMaxVel, ...
                    vehFollCtrlParams, velCtrlPoles)
                vehArrays(iLane).setLatControllers(laneChangeVel, latPoles)
            end
                        
            % In this scenario, p1 (or p3) chooses as leader in the
            % destination lane, whoever is one second away from it at the 
            % moment adjustments start
            bufferTime = 1;
            xLd = vehArrays(1).getVehByName('Ld').x0 + initVel(1)*lcStartTime;
            % Only strategy 3 starts by the last vehicle
            if any(strcmpi({'lastFirst'}, lcStrategy))
                refVehName = 'p2';
            else
                refVehName = 'Lo';
            end     
            xEgoLeader = vehArrays(2).getVehByName(refVehName).x0 ...
                + initVel(2)*lcStartTime;
            vehArrays(1).shiftVehArray(xEgoLeader+relVel*bufferTime-xLd);
            
            % Set vehicle parameters in simulink simulation
            for iLane = 1:nLanes
                vehArrays(iLane).setSimParams()
                vehArrays(iLane).setSimLateralParams();
            end
            
            % Set remaining simulation paramters
            set_param(obj.simModelName, 'StopTime', num2str(finalT));
            set_param(getSimulinkBlockHandle(...
                [obj.simModelName '/yDest' ],true), ...
                'Value', num2str(yDest));
%             set_param(getSimulinkBlockHandle(...
%                 [obj.simModelName '/yOrig' ],true), ...
%                 'Value', num2str(yOrig));
            set_param(getSimulinkBlockHandle(...
                [obj.simModelName '/yRef' ],true), ...
                'time', num2str(lcStartTime), 'Before', num2str(yOrig), ...
                'After', num2str(yDest));
            set_param(getSimulinkBlockHandle(...
                [obj.simModelName '/change in ref' ],true), ...
                'const', num2str(yOrig))
        end
        
        function [] = runAndStoreOutput(obj, vehArrays)

            % Run simulation and store results
            
            simOut = sim(obj.simModelName);
            simTime = simOut.tout;
            
%             stateNames = SimulinkVehicle.simStateNames;
            errorNames = SimulinkVehicle.simErrorNames;
            
            for iLane = 1:length(vehArrays)
                vehArrays(iLane).setSimTime(simTime);
%                 statesNames = fieldnames(vehArrays(nL).statesIdx);
%                 totalInputs = vehArrays(nL).nInputs;
                
                for iVeh = 1:vehArrays(iLane).nv
                    currentVeh = vehArrays(iLane).vehs(iVeh);
                    statesNames = fieldnames(currentVeh.statesIdx);
                    inputNames = fieldnames(currentVeh.inputsIdx);
                    isLaneChanging = strcmpi(currentVeh.name, 'E') ...
                        || contains(currentVeh.name, 'p');
%                     isGeneratingGap = strcmpi(currentVeh.name, 'Fd');
                    
                    vehStatesAndInputs = simOut.find(['states_' ...
                        lower(currentVeh.name)]);
                    for nState = 1:length(statesNames)
                        state = statesNames{nState};
                        currentVeh.(state) = vehStatesAndInputs.(...
                            obj.stateNameDict.(state)).data;
                    end
                    for nInput = 1:length(inputNames)
                        input = inputNames{nInput};
                        currentVeh.(input) = vehStatesAndInputs.(...
                            obj.stateNameDict.(input)).data;
                    end
                    
                    if  ~currentVeh.isSimple %isLaneChanging || isGeneratingGap                        
                        for iError = 1:length(errorNames)
                            currentVeh.simErrors.(errorNames{iError}) = ...
                                vehStatesAndInputs.(errorNames{iError}).data;
                        end
                        
                        if isLaneChanging && contains(obj.simModelName, ...
                                'full_maneuver')
                            currentVeh.lcState = simOut.find(['lc_state_' ...
                                lower(currentVeh.name)]);
                        end
                    end
                    
                end
            end
        end
                
        %%% HELPER FUNCTIONS %%%
        function [] = addDummyVehiclesFromFile(obj, isPlatoon)
            
            reader = VehicleDataReader();
            allRelativeVelocities = {[-5,0, 5]; [-5, 0,5]; ...
                [0, 5]; [-5, 0]};
            if isPlatoon
                scenario = 'platoon';
                allLCStrategies = ...
                    cellstr(SimulinkVehicle.possibleStrategies);
                %  = synchronous, leaderFirst, lastFirst, leaderFirstInvert
            else
                scenario = 'single_veh';
                allLCStrategies = {0}; %irrelevant but at least one
                % element to make loop run
            end
            
            nLCS = length(allLCStrategies);
            for iLCS = 1:nLCS
                nRV = length(allRelativeVelocities{iLCS});
                for k = 1:nRV
                    [origLane, destLane] = reader.loadVehiclesFromFile(...
                        'full_maneuver', scenario, ...
                        allRelativeVelocities{iLCS}(k), allLCStrategies{iLCS});
                    obj.addDummyVehicles([origLane, destLane]);
                end
            end
            
        end
        
        function [arrayAhead, arrayBehind] = addDummyVehicles(obj, ...
                vehArrays)
            %Receives data from a Simulink simulation and adds vehicles to
            %the simulation to make videos more realistic
            extraVehicles = 5;
            arrayAhead(length(vehArrays)) = ...
                LongVehicleArray(extraVehicles);
            arrayBehind(length(vehArrays)) = ...
                LongVehicleArray(extraVehicles);
            
            % Create vehicle arrays
            for n = 1:length(vehArrays)
                vehArray = vehArrays(n);
                leader = vehArray.vehs(1);
                last = vehArray.vehs(end);
                simTime = leader.simTime;
                leaderVel = leader.desiredVelocity;
                tau = leader.tau;
                if contains(leader.name, 'd')
                    vehBaseName = 'dest';
                    velDestination = leader.vx0;
                elseif contains(leader.name, 'o')
                    vehBaseName = 'orig';
                    velOrigin = leader.vx0;
                    isPlatoon = vehArray.getLaneChangingVeh().nv > 1;
                else
                    vehBaseName = 'veh';
                end
                % Reverse engineer the vehicle following controller params
                vehFollCtrlParams = leader.controller.K(1) ...
                    / leader.controller.K(2);
                % Reverse engineer the velocity controller poles
                A = [0 1 0; 0 0 1; 0 0 -1/tau];
                B = [0; 0; -1/tau];
                Kvel = leader.controller.Kvel;
                velCtrlPoles = eig(A+B*Kvel);
                
                % Vehicles ahead
                arrayAhead(n) = LongVehicleArray(extraVehicles);
                arrayAhead(n).addComms();
                arrayAhead(n).vehBaseName = vehBaseName;
                arrayAhead(n).createHomogeneousSSPlatoon(leaderVel,...
                    leaderVel, simTime, vehFollCtrlParams, velCtrlPoles, ...
                    tau)
                arrayAhead(n).setLateralPosition(leader.y0)
                arrayAhead(n).shiftVehArray(leader.x(1) + leader.minVFGap0...
                    + arrayAhead(n).vehs(1).len);
                
                % Vehicles behind
                arrayBehind(n) = LongVehicleArray(extraVehicles);
                arrayBehind(n).addComms();
                arrayBehind(n).vehBaseName = vehBaseName;
                arrayBehind(n).leaderNumber = arrayAhead(n).nv + 1;
                arrayBehind(n).createHomogeneousSSPlatoon(leaderVel,...
                    leaderVel, simTime, vehFollCtrlParams, velCtrlPoles, ...
                    tau)
                arrayBehind(n).setLateralPosition(leader.y0)
                arrayBehind(n).vehs(1).leader = last;
                arrayBehind(n).shiftVehArray(last.x0 -last.len ...
                    - arrayBehind(n).vehs(1).minVFGap0 ...
                    - arrayBehind(n).len + arrayBehind(n).vehs(end).len);
            end
            
            % Simulate vehicle array movement
            disp('Starting simulation of dummy vehicles');
            for n = 1:length(vehArrays)
                arrayAhead(n).simulateWithConstantSpeed();
                arrayBehind(n).simulateOverTime();
            end
            
            allVehicles = obj.createVideoVehicles(vehArrays, arrayAhead, ...
                arrayBehind);
            if isPlatoon
                scenario = 'platoon';
                strategy = vehArrays(1).lcStrategy;
            else
                scenario = 'single_veh';
                strategy = [];
            end
            writer = VehicleDataWriter('video_vehicle', scenario);
            isVerbose = true;
            hasTruck = false;
            if obj.saveResults
                relVel = velDestination - velOrigin;
                writer.setSimulationName(hasTruck, relVel, ...
                    strategy);
                writer.writeVehicleDataToFile(allVehicles, isVerbose);
                
%                 if isPlatoon
%                     scenario = 'platoon';
%                     strategyString = ['_strategy_' vehArrays(1).lcStrategy];
%                 else
%                     scenario = 'single_veh';
%                     strategyString = '';
%                 end
%                 
%                 save(sprintf(['%svideo_vehicles_%s_maneuver_rel_vel_%d'...
%                     '%s_results'],...
%                     obj.resultsFolder, scenario, relVel, strategyString), ...
%                     'allVehicles');
%                 fprintf('%s_rel_vel_%d%s_results saved\n',...
%                     scenario, relVel, strategyString);
            end 
        end
        
        function [] = reduceAndSaveVehicleData(obj, isPlatoon, ...
                relVel, hasTruck, lcStrategy)
            %reduceAndSaveVehicleData Load vehicle data, resample the data
            %to uniform time intervals and save it
            
            % UNFINISHED: CODE TO SAVE THE SUBSAMPLED SIMULATION RESULTS %
            
            if isPlatoon
                scenario = 'platoon';
            else
                scenario = 'single_veh';
            end
            
            newSamplingTime = 0.02;
            dataReader = VehicleDataReader();
            [origLane, destLane] = dataReader.loadVehiclesFromFile(...
                'full_maneuver', scenario, relVel, hasTruck, lcStrategy);
            allVehs = [origLane.vehs, destLane.vehs];
            
            for iVeh = 1:length(allVehs)
                veh = allVehs(iVeh);
                time = veh.simTime;
                newTime = time(1):newSamplingTime:time(end);
                vehStates = timeseries(veh.states, time);
                vehStates = resample(vehStates, newTime);
                veh.states = vehStates.Data;
                vehInputs = timeseries(veh.inputs, time);
                vehInputs = resample(vehInputs, newTime);
                veh.inputs = vehInputs.Data;
                if length(veh.simErrors) > 1
                    fields = fieldnames(veh.simErrors);
                    for iF = 1:length(fields)
                        simError = timeseries(...
                            veh.simErrors.(fields{iF}), time);
                        simError = resample(simError, newTime);
                        veh.simErrors.(fields{iF}) = simError;
                    end
                end
                if length(veh.lcState.Time) > 1
                   veh.lcState = resample(veh.lcState, newTime);
                end
                veh.simTime = newTime;
            end
            
        end
        
        function [] = mySavePlot(~, figHandle, figName, fileType)
            figHandle.WindowState = 'maximized';
            imgFolder = 'C:\Users\fvall\Google Drive\Lane Change\images\';
            if nargin<4
                saveas(figHandle, [imgFolder figName '.' 'eps'], 'epsc');
            else
                saveas(figHandle, [imgFolder figName '.' fileType]);
            end
        end
        
    end
    
    methods (Static)
        function [allVehicles] = createVideoVehicles(simulinkArrays, ...
                arrayAhead, arrayBehind)
            nVehicles = sum([simulinkArrays.nv])...
                + sum([arrayAhead.nv]) + sum([arrayBehind.nv]);
            
            allVehicles(nVehicles) = VideoVehicle();
            
            vehCounter = 1;
            for n = 1:length(simulinkArrays)
                for k = 1:simulinkArrays(n).nv
                    allVehicles(vehCounter).copySimulinkVehicle(...
                        simulinkArrays(n).vehs(k));
                    vehCounter = vehCounter + 1;
                end
                for k = 1:arrayAhead(n).nv
                    allVehicles(vehCounter).copyLongitudinalModelVehicle(...
                        arrayAhead(n).vehs(k));
                    vehCounter = vehCounter + 1;
                end
                for k = 1:arrayBehind(n).nv
                    allVehicles(vehCounter).copyLongitudinalModelVehicle(...
                        arrayBehind(n).vehs(k));
                    vehCounter = vehCounter + 1;
                end
            end
        end

    end
        
    methods (Access = private)
            
        function [t] = timeToReachMaxVel(~, jerkMax, accelMax, ...
                speedMax, accelZero, vZero)
                tToMaxAccel = (accelMax-accelZero)/jerkMax;
                vAtMaxAccel = vZero + (accelMax^2-accelZero^2)/(2*jerkMax);
                tToMaxSpeed = (speedMax-vAtMaxAccel)/accelMax;
                t = tToMaxAccel+tToMaxSpeed;
        end
        
        function [desiredVel] = constVelProfile(obj, v, finalTime)
            t = 0:obj.sampling:finalTime;
            desiredVel = [t', v*ones(length(t), 1)];
        end
        
        function [desiredVel] = maxBrakeVelProfile(obj, initVel, finalT)
            brakeTime = 1;
            t = 0:obj.sampling:finalT;
            desiredVel = initVel*ones(length(t), 1);
            desiredVel(t>=brakeTime) = 0;
            desiredVel = [t', desiredVel];
        end
        
        function [desiredVel] = zeroToMaxToZeroVelProfile(obj, maxVel, ...
                maxAcc, maxJerk)
            % Scenario dependent parameters
            initVel = 0;
            finalT = 20;
            t = 0:obj.sampling:finalT;
            desiredVel = zeros(length(t), 1);
            startAccelTime = 1;
            desiredVel(t>=startAccelTime) = maxVel;
            maxVelTime = startAccelTime + ...
                obj.timeToReachMaxVel(maxJerk, maxAcc, maxVel, 0, initVel);
            desiredVel(t>=floor(maxVelTime)) = 0;
            desiredVel = [t', desiredVel];
        end
        
        function [latPosRef] = setPointLaneChangeRef(obj, lcTimes, finalTime)
            %setPointLaneChangeRef constructs a step-like reference with 
            %lane changes at every element of lcTimes. 
            %Lane changes are assumed to alternate between left and right
            t = 0:obj.sampling:finalTime;
            yRef = zeros(length(t), 1);
            for k = 1:length(lcTimes) % not an efficient loop...
                yRef(t>=lcTimes(k)) = rem(k,2)*obj.laneWidth;
            end
%             plot(yRef);
            latPosRef = [t', yRef];
        end
        
        function [latPosRef] = optimalTimeLaneChangeRef(obj, ...
                lcTimes, finalTime, maxLatAcc, maxLatJerk)
            %optimalTimeLaneChangeRef Creates the time optimal lateral
            %position reference under given jerk and acceleration
            %constraints
            %Lane changes are assumed to alternate between left and right
            
            t = 0:obj.sampling:finalTime;
            delta1 = min(maxLatAcc/maxLatJerk, ...
                (obj.laneWidth/2/maxLatJerk)^(1/3));
            delta2poly = delta1*maxLatJerk*[1 3*delta1 2*delta1^2] ...
                - [0 0 obj.laneWidth];
            delta2candidates = roots(delta2poly);
            delta2 = min(delta2candidates(delta2candidates>0));            
            
            jerk = zeros(length(t), 1);
            for k = 1:length(lcTimes)
                t0 = lcTimes(k);
                t1 = t0+delta1;
                t2 = t1+delta2;
                t3 = t2+2*delta1;
                t4 = t3+delta2;
                t5 = t4+delta1;
                jerk((t>=t0 & t<t1) | (t>=t4 & t<t5)) = ...
                    (-1)^(k+1)*maxLatJerk;
                jerk(t>=t2 & t<t3) = (-1)^(k)*maxLatJerk;
            end
            
            plot(jerk);
            yRef = cumtrapz(obj.sampling, cumtrapz(obj.sampling, ...
                cumtrapz(obj.sampling, jerk)));
%             plot(yRef);
            latPosRef = [t', yRef];
        end

        
    end
    
    
end

