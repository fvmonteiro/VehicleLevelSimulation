classdef SimulinkScenario
    %SimulinkScenario Methods to run different simulink scenarios
    
    properties
        simModelName
        nVehicles
        vehiclesPerLane % array with nLanes elements; leftmost lane at element 1, rightmost at element nLanes
        plotter
        laneWidth = 3.6;
        sampling = 0.1; %used to create reference signals to simulink
        saveFigs = 0;
    end
    
%     properties (Access = private)
%         imgFolder = 'G:/My Drive/Lane Change/images/';
%     end
    
    methods
        function obj = SimulinkScenario(vehiclesPerLane, saveFigs)
            %SimulinkScenario Set the simulink model and inform how many
            %vehicles are there
%             obj.nVehicles = nVehicles;
            obj.vehiclesPerLane = vehiclesPerLane;
            obj.nVehicles = sum(vehiclesPerLane);
            obj.plotter = VehiclePlots();
            if nargin>=2
                obj.saveFigs = saveFigs;
            end
        end
        
        function [dataStruct] = accMaxBrake(obj, poles)
            obj.simModelName = 'accTests';
            scenario = 'maxBrake';
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
        
    function [dataStruct] = gapGeneration(obj, accPoles, KiVelControl)
            obj.simModelName = 'gap_generation';
            % Run
            [dataStruct, gapGenDataStruct] = obj.runGapGenerationSimulation(accPoles, KiVelControl);
            % Plot
%             figs = cell(length(dataStruct), 1);
%             for nL = 1:length(dataStruct)
%                 figs{nL} = obj.plotter.plotDataStruct(dataStruct(nL), 'ACC');
%             end
%             figs = cell(length(gapGenDataStruct), 1);
%             mainVehName = {'ego', 'fd'};
%             vehNames = {{'L_o', 'L_d'},{'L_d', 'Ego'}};
%             plotNames = {'gaps', 'errors'};
%             for n = 1:length(gapGenDataStruct)
%                 figs{n} = obj.plotter.plotGapGenerationData(gapGenDataStruct(n), mainVehName{n}, vehNames{n});
%                 if obj.saveFigs
%                     for k = 1:length(figs)
%                         obj.plotter.mySavePlot(figs{n}{k}, sprintf('gg_%s_%s', ...
%                             mainVehName{n}, plotNames{k}))
%                     end
%                 end
%             end
            
            % Save
            
            
    end
        
        function [] = laneChangeSingleVeh(obj, poles, measuredVars)
            
            obj.simModelName = 'lc_single_vehicle';
            nMV = length(measuredVars);
            C = zeros(nMV, 4);
            for n = 1:nMV
                C(n, measuredVars(n)) = 1;
            end
            % Run
            dataStruct = obj.runLCSimulation(poles, C, 'Full state', 'Optimal time', 0);
            
            % Plots
            figs = obj.plotter.plotDataStruct(dataStruct, 'single veh lc');
            
            % Save
            names = {'states', 'obsErrors', 'errors', 'inputAndAccel'};
            varNames = ["Y", "vy", "theta", "omega"];
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('doubleLC_1v_%s_measured_%s', ...
                    join(varNames(measuredVars), '_'), names{k}))
                end
            end
        end
        
        function [] = laneChangeFullState(obj, poles, comms)

            measuredVars = 1:4;
            laneChangeFullStateWithObserver(obj, poles, ...
                measuredVars, comms);
        end
        
        function [] = laneChangeFullStateWithObserver(obj, poles, ...
                measuredVars, comms)
            
            obj.simModelName = 'lc_state_fb';
            
            nMV = length(measuredVars);
            C = zeros(nMV, 4);
            for n = 1:nMV
                C(n, measuredVars(n)) = 1;
            end
            % Run
            dataStruct = obj.runLCSimulation(poles, C, 'Full state', 'Optimal time', comms);
            
            % Plot
            figs = obj.plotter.plotDataStruct(dataStruct, '');
            
            % Save
            names = {'states', 'obsErrors', 'errors', 'inputAndAccel'};
            varNames = ["Y", "vy", "theta", "omega"];
            if obj.saveFigs
                for k = 1:length(figs)
                obj.plotter.saveBigPlot(figs{k}, sprintf('doubleLC_%s_measured_%s_%s', ...
                    join(varNames(measuredVars), '_'), follRefStr, names{k}))
                end
            end
        end
        
        function [] = laneChangeOutputFeedback(obj, poles)

            obj.simModelName = 'laneChangeOutputFeedback';
            C = [1 0 0 0];
            obj.runLCSimulation(poles, C, 'PID');
            
        end
    
    end
    
    
    methods (Access = private)
        
        function [dataStruct] = runACCSimulation(obj, poles, scenario, ka, accelFilter)
            
            fprintf('Running model %s, scenario %s \n', obj.simModelName, scenario);
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                load_system(obj.simModelName);
            end
            
            % Load some parameters
            sim_parameters;
            
            % Leader desired speed
            maxVel = 30;
            switch scenario
                case 'maxBrake'
                    initVel = maxVel;
                    desVel = obj.maxBrakeVelProfile(initVel);
                case 'zeroToMaxToZero'
                    desVel = obj.zeroToMaxToZeroVelProfile(maxVel, maxAcc, maxJerk);
                    initVel = desVel(1, 2);
                otherwise
                    error('Unkown scenario')
            end
            finalT = desVel(end, 1);
            
            % Controller gains
            controller = Controller();
            if ka <= 0 % no leader acceleration feedforward
                controller.setGains('ACC PID', poles, h);
%                 [Kd, Kp, Ki] = obj.accGain(poles, h);
            else % const gain leader acceleration feedforward
                controller.setGains('CACC PID', [poles, ka], h);
%                 [Kd, Kp, Ki] = obj.caccGain(poles, h, ka);
                if accelFilter
                    tau = h;
                else
                    tau = 0;
                end
            end
            
            Ki = controller.K(1);
            Kp = controller.K(2);
            Kd = controller.K(3);
            
            % Run simulation
            obj.setInitialStatesSinglePlatoon(initVel, vLen, g0, h)
            set_param(obj.simModelName, 'StopTime', num2str(finalT));
            simOut = sim(obj.simModelName, 'SrcWorkspace','current');
            simTime = simOut.simTime;
%             simDesVel = simOut.desVel(:, 2);
            states2D = simOut.states;
            
            % We make states a 3D matrix (easier to index)
            nVeh = obj.nVehicles;
            nVar = size(states2D,2)/nVeh;
            states3D = reshape(states2D, [length(simTime), nVar, nVeh]);
            % The first state is the gap
            gaps = -diff(states3D(:,1,:), 1, 3)-vLen;
            states3D(:, 1, 2:end) = gaps; 
            states3D(:, 1, 1) = zeros(length(simTime), 1);% but the leader has no gap, so we fill with zeros
            
            % Compute errors (only for the followers)
            headwayError = gaps-g0-h*states3D(:, 2, 2:end);
            velError = -diff(states3D(:,2,:), 1, 3);
            errors = zeros(length(simTime), 2, obj.nVehicles-1);
            errors(:, 1, :) = headwayError;
            errors(:, 2, :) = velError;
            
            dataStruct = struct('simTime', simTime, 'longStates', states3D, 'longErrors', errors);
            
        end
        
        function [dataStruct, gapGenDataStruct] = runGapGenerationSimulation(obj, poles, KiVelControl)
            
            fprintf('Running model %s, scenario %s \n', obj.simModelName, 'zero rel velocity');
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                load_system(obj.simModelName);
            end
            
            % Load some parameters
            sim_parameters;
            
            % Leader desired speed
            initVel = [20; 20]; %[m/s]
            finalT = 25;
            lcStartTime = 5;
            desVelAdj = obj.constVelProfile(initVel(1), finalT);
            desVelOrig = obj.constVelProfile(initVel(1), finalT);           
            
            
            % Controller gains (Note: we're assuming same gains for
            % everyone)
            controller = Controller();
            controller.setGains('ACC PID', poles, h);            
            Ki = controller.K(1);
            Kp = controller.K(2);
            Kd = controller.K(3);
            velKi = KiVelControl;
            
            % Run simulation
            obj.setDefaultInitialStates(initVel, vLen, g0, h)
            set_param(obj.simModelName, 'StopTime', num2str(finalT));
            simOut = sim(obj.simModelName, 'SrcWorkspace','current');
            simTime = simOut.simTime;
            states2D = simOut.states;
            errorsFromSim = simOut.errors;
            gapsFromSim = simOut.gaps;
            
            nLanes = length(obj.vehiclesPerLane);
            nVar = size(states2D,2)/obj.nVehicles;
            dataStruct = struct('simTime', cell(nLanes, 1), 'longStates', cell(nLanes, 1), 'longErrors', cell(nLanes, 1));
            vehBaseIdx = 0;
            
            % Choosing specifically ego (Vehicle4) and Fd (Vehicle2)
            states3D = reshape(states2D, [length(simTime), nVar, obj.nVehicles]);
            egoIdx = 4;
            ldIdx = 1;
            fdIdx = 2;
            loIdx = 3;
            gapsEgo = zeros(size(gapsFromSim, 1), 3);
            gapsEgo(:, 1) = gapsFromSim(:, 1);
            gapsEgo(:, 2) = states3D(:, 1, loIdx) - states3D(:, 1, egoIdx) - vLen;
            gapsEgo(:, 3) = states3D(:, 1, ldIdx) - states3D(:, 1, egoIdx) - vLen;
            gapsFd = zeros(size(gapsFromSim, 1), 3);
            gapsFd(:, 1) = gapsFromSim(:, 2);
            gapsFd(:, 2) = states3D(:, 1, ldIdx) - states3D(:, 1, fdIdx) - vLen;
            gapsFd(:, 3) = states3D(:, 1, egoIdx) - states3D(:, 1, fdIdx) - vLen;
            
            errorsEgo = zeros(size(gapsFromSim, 1),2, 1);
            errorsEgo(:, :, 1) = errorsFromSim(:, 1:2);
            errorsFd = zeros(size(gapsFromSim, 1),2, 1);
            errorsFd(:, :, 1) = errorsFromSim(:, 3:4);
            
            gapGenDataStruct(1) = struct('simTime', simTime, 'ggGap', gapsEgo, 'ggErrors', errorsEgo);
            gapGenDataStruct(2) = struct('simTime', simTime, 'ggGap', gapsFd, 'ggErrors', errorsFd);
            
            % DataStruct per lane
            for nL = 1:nLanes
                nVeh = obj.vehiclesPerLane(nL);
                vehIdx = vehBaseIdx+1:vehBaseIdx+nVeh;
                vehBaseIdx = vehIdx(end);
                % We make states a 3D matrix (easier to index)
                laneStates3D = states3D(:, :, vehIdx);
                % The first state is the gap
                gaps = -diff(laneStates3D(:,1,:), 1, 3)-vLen;
                laneStates3D(:, 1, 2:end) = gaps;
                laneStates3D(:, 1, 1) = zeros(length(simTime), 1);% but the leader has no gap, so we fill with zeros
                
                % Compute errors (only for the followers)
                headwayError = gaps-g0-h*laneStates3D(:, 2, 2:end);
                velError = -diff(laneStates3D(:,2,:), 1, 3);
                errors = zeros(length(simTime), 2, obj.vehiclesPerLane(nL)-1);
                errors(:, 1, :) = headwayError;
                errors(:, 2, :) = velError;
                dataStruct(nL) = struct('simTime', simTime, 'longStates', laneStates3D, 'longErrors', errors);
            end
            
        end
        
        function [dataStruct] = runLCSimulation(obj, poles, C, controlType, leaderRef, communications)
            
            % Show proper message in command prompt
            variableNames = {'Y', 'vy', 'theta', 'omega'};
            measuredVars = variableNames(logical(sum(C, 1)));
            
            fprintf('Running model %s, measured variables: %s \n', ...
                obj.simModelName, strjoin(measuredVars, ', '));
            
            % Load model if needed
            if ~bdIsLoaded(obj.simModelName)
                load_system(obj.simModelName);
            end
            
            % Load some parameters
            sim_parameters;
            inputTau = 1;
            
            % Lateral reference
            finalTime = 60;
            lcTimes = [1, 30];
            switch leaderRef
                case 'Setpoint'
                    yRef = obj.setPointLaneChangeRef(lcTimes, finalTime);
                case 'Optimal time'
                    yRef = obj.optimalTimeLaneChangeRef(lcTimes, finalTime, ...
                        maxLatAcc, maxLatJerk);
                otherwise
                    error('Unknown leader reference type');
            end
            
            % Speed reference
            vxRef = 20;
            
            % Define system matrices
            a1 = 2*(Cf+Cr)/vehMass;
            a2 = 2*(Cf*lf-Cr*lr)/vehMass;
            a3 = 2*(Cf*lf-Cr*lr)/Iz;
            a4 = 2*(Cf*lf^2+Cr*lr^2)/Iz;
            b1 = 2*Cf/vehMass;
            b2 = 2*Cf*lf/Iz;
            A = [0 1 vxRef 0; 0 -a1/vxRef 0 -vxRef-a2/vxRef; 0 0 0 1; 0 -a3/vxRef 0 -a4/vxRef];
            B = [0; b1; 0; b2];
            
            % Determine gains
            switch controlType
                case 'Full state'
                    % full state feedback with Luenberg observer
                    K = place(A, B, poles);
                    L = place(A', C', poles*3)';
                case 'PID'
                    [Kd, Kp, Ki] = obj.latPidGain(poles, vxRef, [a1 a2 a3 a4], [b1 b2]);
                otherwise
                    error('Unknown controller type');
            end
            
            % Last things to define before running simulation
            % 1. find out if we need an observer
            obs = sum(C(:))~=length(variableNames); % assuming C contains only zeros and ones
            observer = Simulink.Parameter(obs);
            % 2. communicate states or not
            comms = Simulink.Parameter(communications);
            commDelay = 0.0001; % 0.1 for realistic delay 
            
            % Run simulation
            obj.setInitialStatesSinglePlatoon(vxRef, vLen, g0, h);
            set_param(obj.simModelName, 'StopTime', num2str(finalTime));
            simOut = sim(obj.simModelName, 'SrcWorkspace','current');
            simTime = simOut.tout;
            states2D = simOut.states;
            estStates2D = squeeze(simOut.estStates)';
            
            % We make states a 3D matrix (easier to index)
            nVeh = obj.nVehicles;
            nVar = size(states2D,2)/nVeh;
            states3D = reshape(states2D, [length(simTime), nVar, nVeh]);
            % and get only the lateral states
            latStatesInd = [2 5 3 6];
            latStates = states3D(:, latStatesInd, :);
            
            % Observer error
            nLatVars = length(latStatesInd);
            estStates3D = reshape(estStates2D, [length(simTime), nLatVars, nVeh]);
            obsErrors = latStates - estStates3D;
            
            % Compute lateral errors
            if nVeh>1
                errors = zeros(length(simTime), 1, nVeh-1);
                errors(:, 1, :) = -diff(states3D(:,2,:), 1, 3);
            else
                errors(:, 1, 1) = simOut.simYRef - states3D(:, 2, 1);
            end

            % Lateral accelerations
            inputAndAccel = zeros(length(simTime), 1, nVeh);
            inputAndAccel(:, 1, :) = simOut.delta;
            inputAndAccel(:, 2, :) = states3D(:, 8, :);
            
            dataStruct = struct('simTime', simTime, 'latStates', latStates, ...
                'obsErrors', obsErrors, 'latErrors', errors, 'inputAndAccel', inputAndAccel);

        end
        
        
        %%% TODO: change method to receive Veh and/or Platoon paramters %%%
        function [] = setInitialStatesSinglePlatoon(obj, initVel, vLen, g0, h)
            warning('Method has to change to fit future organization')
            
            % Vehicle's initial states
            vehNames = cell(obj.nVehicles, 1);
            vehX0s = zeros(obj.nVehicles, 1);
%             vehInitVel = initVel*ones(obj.nVehicles, 1);
            gaps = zeros(obj.nVehicles-1, 1);
            for n = obj.nVehicles-1:-1:1
                vehNames{n} = ['Vehicle' num2str(n)];
                gaps(n+1) = g0 + h*initVel;
                vehX0s(n) = vehX0s(n+1) + vLen + gaps(n+1);% + vLen + g0 + h*follInitVel(n+1);
            end
            vehNames{obj.nVehicles} = ['Vehicle' num2str(obj.nVehicles)];
            
            % Set values in simulink
            for n = 1:length(vehNames)
                vehicleBlockHandle = getSimulinkBlockHandle([obj.simModelName '/' vehNames{n}],true);
                set_param(vehicleBlockHandle, 'X_o', num2str(vehX0s(n)), ...
                    'initVel', num2str(initVel));
            end
        end
        
        function [] = setDefaultInitialStates(obj, initVel, vLen, g0, h)
            % All vehicles start at the same speed with steady state space
            % from one another
            nLanes = length(obj.vehiclesPerLane);
            laneY = (nLanes-1)*obj.laneWidth:-obj.laneWidth:0;
            
            % Vehicle's initial states
            vehNames = cell(obj.nVehicles, 1);
            vehX0s = zeros(obj.nVehicles, 1);
            vehV0s = zeros(obj.nVehicles, 1);
            vehY0s = zeros(obj.nVehicles, 1);
%             vehInitVel = initVel*ones(obj.nVehicles, 1);
            
            for nL = 1:nLanes
                gaps = zeros(obj.vehiclesPerLane(nL)-1, 1);
                vehBaseIdx = (nL-1)*nLanes;
                vehPerLane = obj.vehiclesPerLane(nL);
                vehNames{vehBaseIdx+vehPerLane} = ['Vehicle' num2str(vehBaseIdx+vehPerLane)];
                vehV0s(vehBaseIdx+vehPerLane) = initVel(nL);
                vehY0s(vehBaseIdx+vehPerLane) = laneY(nL);
                for nV = vehPerLane-1:-1:1
                    vehIdx = vehBaseIdx + nV;
                    vehNames{vehIdx} = ['Vehicle' num2str(vehIdx)];
                    gaps(nV+1) = g0 + h*initVel(nL);
                    vehX0s(vehIdx) = vehX0s(vehIdx+1) + vLen + gaps(nV+1);% + vLen + g0 + h*follInitVel(n+1);
                    vehV0s(vehIdx) = initVel(nL);
                    vehY0s(vehIdx) = laneY(nL);
                end
            end
            
            obj.setInitialStates(vehV0s, vehNames, vehX0s, vehY0s);
        end
        
        function [] = setInitialStates(obj, initVel, vehNames, vehX0s, vehY0s)
            warning('Method has to change to receive Veh and Platoon objects as parameters')
            % Set values in simulink
            for n = 1:length(vehNames)
                vehicleBlockHandle = getSimulinkBlockHandle([obj.simModelName '/' vehNames{n}],true);
                set_param(vehicleBlockHandle, 'X_o', num2str(vehX0s(n)), ...
                    'initVel', num2str(initVel(n)), 'Y_o', num2str(vehY0s(n)));
            end
        end
          
        function [t] = timeToReachMaxVel(~, jerkMax, accelMax, speedMax, accelZero, vZero)
                tToMaxAccel = (accelMax-accelZero)/jerkMax;
                vAtMaxAccel = vZero + (accelMax^2-accelZero^2)/(2*jerkMax);
                tToMaxSpeed = (speedMax-vAtMaxAccel)/accelMax;
                t = tToMaxAccel+tToMaxSpeed;
        end
        
        function [desiredVel] = constVelProfile(obj, v, finalTime)
            t = 0:obj.sampling:finalTime;
            desiredVel = [t', v*ones(length(t), 1)];
        end
        
        function [desiredVel] = maxBrakeVelProfile(obj, initVel)
            finalT = 15;
            brakeTime = 1;
            t = 0:obj.sampling:finalT;
            desiredVel = initVel*ones(length(t), 1);
            desiredVel(t>=brakeTime) = 0;
            desiredVel = [t', desiredVel];
        end
        
        function [desiredVel] = zeroToMaxToZeroVelProfile(obj, maxVel, maxAcc, maxJerk)
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
        
        function [latPosRef] = optimalTimeLaneChangeRef(obj, lcTimes, finalTime,...
                maxLatAcc, maxLatJerk)
            %optimalTimeLaneChangeRef Creates the time optimal lateral
            %position reference under given jerk and acceleration
            %constraints
            %Lane changes are assumed to alternate between left and right
            
            t = 0:obj.sampling:finalTime;
            delta1 = min(maxLatAcc/maxLatJerk, (obj.laneWidth/2/maxLatJerk)^(1/3));
            delta2poly = delta1*maxLatJerk*[1 3*delta1 2*delta1^2] - [0 0 obj.laneWidth];
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
                jerk((t>=t0 & t<t1) | (t>=t4 & t<t5)) = (-1)^(k+1)*maxLatJerk;
                jerk(t>=t2 & t<t3) = (-1)^(k)*maxLatJerk;
            end
            
            plot(jerk);
            yRef = cumtrapz(obj.sampling, cumtrapz(obj.sampling, ...
                cumtrapz(obj.sampling, jerk)));
%             plot(yRef);
            latPosRef = [t', yRef];
        end

        %%%%% CONTROLLER FUNCTIONS MOVED TO CONTROLLER CLASS       
        function [Kd, Kp, Ki] = latPidGain(~, poles, vx, a, b)
            warning('Current PID lateral control DOESN''T WORK; must be done on reduced order system')
            error('This method must be transferred to Controller class');
            beta0 = a(1)*b(2)-a(3)*b(1);
            beta1 = (a(4)*b(1)-a(2)*b(2))/vx;
            alpha2 = (a(1)*a(4)-a(2)*a(3))/(vx^2) - a(3);
            alpha3 = (a(1)+a(4))/vx;
            
            coeffs = 1;
            for p = poles
                coeffs = conv(coeffs, [1 p]);
            end
            z = coeffs(2:end)';
            z(1) = z(1)-alpha3;
            z(2) = z(2)-alpha2;
            
            A = [1 0 0;beta1 1 0; beta0 beta1 1; 0 beta0 beta1; 0 0 beta0];
            
            K = A\z;
            Kd = K(1);
            Kp = K(2);
            Ki = K(3);
        end
        
    end
    
end

