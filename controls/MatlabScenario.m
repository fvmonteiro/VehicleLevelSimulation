classdef MatlabScenario
    properties
        % Parameters
        %         n_states = 3;
        T = 30;%4.5; % maneuver time
        samplingPeriod = .02;
        amin = -3; % maximum deceleration
        amax = 3; % maximum acceleration
        
        % Initial values
        sf0 = 0; % following vehicle initial position (m)
        sl0 = 8; % leading vehicle initial position (m)
        vf0 = 25; % following vehicle initial speed (m/s)
        vl0 = 25; % leading vehicle initial (and constant) speed (m/s)
        af0 = 0; % following vehicle initial acceleration (m/s^2)
        al0 = 0; % leading vehicle initial acceleration (m/s^2)
        
        maxVel = 30;
                
        % For future plots
        legends = {'Ld', 'Fd', 'M'};
    end
    
    properties (Dependent)
        simTime
    end
    
    properties (Constant)
        mpsToKmph = 3.6
    end
    
    methods
        
%         function obj = MatlabScenario()
%             obj.t = 0:obj.samplingPeriod:obj.T;
%         end

        %%% SAFE GAPS AND COLLISION SEVERITY TESTS %%%
        function [leader, follower] = checkCollisionSeverity(~, ...
                leaderType, followerType, gamma)
            %checkCollisionSeverity Computes severity of collision under
            %the worst case scenario numerically and analitically, and
            %compares results
            % @followerType: options: 'PV', 'HDV'
            % @leaderType: options: 'PV', 'HDV'
            % @gamma (optional): (leader max brake) / (follower max brake)
            baseSpeed = 20;
            followerSpeed = baseSpeed;
            leaderSpeed = baseSpeed;
            
            if nargin > 3
            [leader, follower] = MatlabScenario.create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed, gamma);
            else
                [leader, follower] = MatlabScenario.create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed);
            end
            
            finalT = ceil(max(leader.fullStopTime, ...
                follower.fullStopTime)*1.2);
            leader.brakeFullStop(finalT);
            follower.brakeFullStop(finalT);
            [numericalSeverity, g0ArrayNumerical] = ...
                follower.computeSeverityProfile();
            
            [analyticalSeverity, g0ArrayAnalytical] = ...
                follower.computeAnalyticalSeverityProfile();
            
            hold off;
            plot(g0ArrayNumerical, numericalSeverity);
            hold on; grid on;
            plot(g0ArrayAnalytical, analyticalSeverity);
            for n = 1:length(follower.gapThresholds)
                if follower.gapThresholds(n) > 0
                    xline(follower.gapThresholds(n))
                end
            end
            legend('Numerical', 'Analytical');
            xlabel('Initial gap [m]')
            ylabel('Severity [m/s]')
        end
        
        function [follower, leader] = plotCollisionSeverityVsHeadway(~, ...
                leaderType, followerType, gamma)
            
            baseSpeed = 20;
            followerSpeed = baseSpeed;
            leaderSpeed = baseSpeed;
            
            if nargin > 3
            [leader, follower] = MatlabScenario.create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed, gamma);
            else
                [leader, follower] = MatlabScenario.create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed);
            end
            follower.desiredVelocity = 25;
            
            finalT = ceil(max(leader.fullStopTime, ...
                follower.fullStopTime)*1.2);
            leader.brakeFullStop(finalT);
            follower.brakeFullStop(finalT);
            % TODO: change to analytical functions once they work in all
            % cases
            [numericalSeverity, g0ArrayNumerical] = ...
                follower.computeSeverityProfile();
            
            % Get the equivalent time headway for each initial gap
            rho = 0.1;
            follower.computeTimeHeadway(follower.desiredVelocity, rho);
            hArray = (g0ArrayNumerical - follower.d0)/follower.vx0;
            
            % Plot on the existing figure
            hold off;
            plot(hArray(hArray>0), numericalSeverity(hArray>0));
            hold on; grid on;
            plot(follower.h, 0, 'ro', 'MarkerFaceColor', 'r')
            for n = 1:length(follower.gapThresholds)
                hThreshold = ...
                    (follower.gapThresholds(n) - follower.d0)/follower.vx0;
                if hThreshold < follower.h
                    xline(hThreshold)
                end
            end
            xlabel('Time headway [s]')
            ylabel('Severity [m/s]')
            legend('severity', 'safe time headway')
        end
        
        function [] = plotEstimatedSeverity(~, gamma, rho)
            %plotEstimatedSeverity Compares the estimated risk to the 
            %actual risk
            
            vehicleType = 'PV';
            leaderType = vehicleType;
            followerType = vehicleType;
            baseSpeed = 20;
            followerSpeed = baseSpeed;
            leaderSpeed = baseSpeed;
            freeFlowVelocity = 1.2*followerSpeed;
            [~, follower] = MatlabScenario.create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed, ...
                gamma, freeFlowVelocity);
            
            % Exact severity
            [exactSeverity, exactGap0Array] = ...
                follower.computeAnalyticalSeverityProfile();
            
            % Estimated severity
            [estimatedSeverity, estimatedGap0Array] = ...
                follower.estimateSeverity(rho);
            
            hold off;
            plot(exactGap0Array, exactSeverity);
            hold on; grid on;
            plot(estimatedGap0Array, estimatedSeverity);
            legend('Exact', 'Estimated');
            xlabel('Initial gap [m]')
            ylabel('Severity [m/s]')
        end
        
        function [] = plotSeverityFromHeadwayReduction(~, gamma, rho)
            %plotSeverityFromHeadwayReduction Compares the estimated risk
            %when reducing the time headway of the vehicle following gap
            %to the actual risk
            
            vehicleType = 'PV';
            leaderType = vehicleType;
            followerType = vehicleType;
            baseSpeed = 20;
            leaderSpeed = baseSpeed;
            followerSpeed = baseSpeed;
            freeFlowVelocity = 1.2*followerSpeed;
            
            % Exact collision severity
            [leader, follower] = MatlabScenario.create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed, gamma);
%             finalT = ceil(max(leader.fullStopTime, ...
%                 follower.fullStopTime)*1.2);
%             leader.brakeFullStop(finalT);
%             follower.brakeFullStop(finalT);
            [severity, g0Array] = follower.computeAnalyticalSeverityProfile();
            hCollisionFree = (g0Array - follower.d0)/follower.vx0;
            
            % Decreased headway based on accepted estimated severity
            massRatio = max(follower.m, leader.m)/(follower.m + leader.m);
            follower.computeTimeHeadway(freeFlowVelocity, rho);
            gamma = leader.maxBrake/follower.maxBrake;
            threshold = (1-rho)*freeFlowVelocity...
                /(freeFlowVelocity+follower.lambda1);
            maxSeverity = sqrt(follower.minVFGap0*2*follower.maxBrake)...
                *massRatio;
            if gamma<=threshold
                maxSeverity = maxSeverity*sqrt(1-gamma);
            end
            acceptedSeverity = 0:0.01:maxSeverity;
            
            if gamma>threshold
                hrFreeFlow = follower.h - (acceptedSeverity/massRatio).^2 ...
                    /(2*freeFlowVelocity*follower.maxBrake);
                hrCurrentSpeed = follower.h - (acceptedSeverity/massRatio).^2 ...
                    /(2*follower.vx0*follower.maxBrake);
            else
                hrFreeFlow = follower.h - (acceptedSeverity/massRatio).^2 ...
                    /(2*freeFlowVelocity*follower.maxBrake*(1-gamma));
                hrCurrentSpeed = follower.h - (acceptedSeverity/massRatio).^2 ...
                    /(2*follower.vx0*follower.maxBrake*(1-gamma));
            end
            
%             riskyGapFreeFlow = hrFreeFlow*follower.vx0 + follower.d0;
%             riskyGapCurrentSpeed = hrCurrentSpeed*follower.vx0 + follower.d0;
            
            hold off;
            idx1 = hCollisionFree>0;
            plot(hCollisionFree(idx1), severity(idx1));
            hold on; grid on;
            idx2 = hrCurrentSpeed>0;
            plot(hrFreeFlow(idx2), acceptedSeverity(idx2));
            plot(hrCurrentSpeed(idx2), acceptedSeverity(idx2));
            legend('Exact', 'Free Vel Estimate', 'Current Vel Estimate');
            xlabel('Time headway [s]')
            ylabel('Accepted Severity [m/s]')
        end
        
        function [] = plotSeverityAtDifferentSpeeds(~, gamma)
            %plotSeverityAtDifferentSpeeds Function used to check what
            %happens to the severity curve when the deltaV is constant (at
            %zero) but the speeds vary
            
            vehicleType = 'PV';
            vx0Array = 5:5:30;
            legendStrings = cell(1, length(vx0Array));
            hold off
            for iV = 1:length(vx0Array)
                vx0 = vx0Array(iV);
                [leader, follower] = MatlabScenario.create_kinematic_vehicles(...
                    vehicleType, vehicleType, vx0 - 0.1*vx0, vx0, gamma);
                finalT = ceil(max(leader.fullStopTime, ...
                    follower.fullStopTime)*1.2);
                leader.brakeFullStop(finalT);
                follower.brakeFullStop(finalT);
                [severity, g0Array] = ...
                    follower.computeSeverityProfile();
%                 [severity, g0Array] = ...
%                     follower.computeAnalyticalSeverityProfile();
                plot(g0Array, severity);
                legendStrings{iV} = ['v = ' num2str(vx0)];
                hold on
            end
            legend(legendStrings);
            xlabel('Initial gap [m]')
            ylabel('Severity [m/s]')
        end
        
        %%% DEPRECATED: LANE CHANGE TESTS (too complicated to simulate the
        %%% bicycle model in Matlab, all LC tests in Simulink)
        function[statesFig, inputsFig] = testPlatoonLCwithGapGeneration(obj)
            
            t = obj.simTime;
            % Lane change parameters
            lcStart = 1;
            laneWidth = 3.6;
            
            % Controller Parameters
            Ks = 1; % ACC position gain
            Kv = 1; % ACC velocity gain
            Kp = 2; % ALC proportional gain
            Kd = 2; % ALC derivative gain
            
            % Initial conditions
            xf0 = 0;
            yf0 = laneWidth;
            theta0 = 0;
            
            % Vehicles on adjacent lane
            % (since they're not in a platoon, their ACC headways are more
            % conservative)
            vad = 20; % desired speed at the adjacent lane in m/s
            qf0 = [xf0; yf0; vad; theta0];
            follower = BicycleVehicleModel(qf0); % vehicle that ends simulation behind the platoon
            follower.tg = 2;
            follower.g0 = 3;
            follGap = follower.g0 + follower.tg*vad;
            leader = BicycleVehicleModel(qf0 + [follGap+follower.len; 0; 0; 0]);
            % vehicle that ends simulation in front of the platoon
            leader.tg = 2;
            leader.g0 = 3;
            follower.leader = leader;
            
            % Lane changing platoon
            nv = 3; % number of vehicles in the platoon
            platoon = BicyclePlatoon(nv);
            platoon.createHomogeneousSSPlatoon(vad);
%             platoon.vehs(1).leader = leader;
            
            % Controllers
            follContr = Controller(follower);
%             Q = diag([1.0 0.5]);
%             R = 2;
%             Klqr = follContr.simpleLqrFeedback(Q, R);
            KaccFoll = follContr.constHeadwayACC(Ks, Kv);
%             newGap = follGap + platoon.vehs(nv).len + platoon.vehs(nv).g0 ...
%                 + platoon.vehs(nv).tg*vad;
            
%             platoonLeaderContr = Controller(platoon.vehs(1));
%             KaccPl = platoonLeaderContr.constHeadwayACC(Ks, Kv);
            plLeaderGap = platoon.vehs(nv).g0 + platoon.vehs(nv).tg*vad;
            platoonControlParams = [Ks, Kv, Kp, Kd];
            
            % Trajectories
            zeroStates = zeros(length(qf0), length(t));
            zeroInputs = zeros(2, length(t));
            leaderTrajectory = zeroStates;
            followerTrajectory = zeroStates;
            leaderInputs = zeroInputs;
            followerInputs = zeroInputs;
            platoonTrajectory = repmat(zeroStates, 1, 1, nv);
            platoonInputs = repmat(zeroInputs, 1, 1, nv);
            
            leaderTrajectory(:, 1) = leader.currentState;
            followerTrajectory(:, 1) = follower.currentState;
            platoonTrajectory(:, 1, :) = [platoon.vehs.currentState];

            follLeaderIdx = zeros(length(t), 1);
            platoonLeaderLeader = zeros(length(t), 1);
            for k = 2:length(t)
                % First we compute all inputs
                leaderInputs(:, k) = [leader.singleStepACC(KaccFoll); 0];
                
                if t(k)<=lcStart
%                     platoonInputs(:, k, :) = platoon.computeInputs('ACC', ...
%                         platoonControlParams);
                    platoonInputs(:, k, :) = platoon.computeInputs('Longitudinal and Lateral', ...
                        platoonControlParams, [0; 0]);
                else % lane change procedure starts
                    follower.leader = platoon.vehs(platoon.nv); % the vehicle in adjacent gap will
                    % start to create a gap
                    platoon.vehs(1).leader = leader; % the platoon leader will position 
                    % itself behind the leading vehicle in the adjacent
                    % lane
                    
                    % Check if gap is open
                    backGap = platoonTrajectory(1, k-1, nv)-platoon.vehs(nv).len-followerTrajectory(1, k-1);
                    frontGap = leaderTrajectory(1, k-1)-leader.len-platoonTrajectory(1, k-1, 1);
                    if (backGap>0.9*follGap && frontGap>0.9*plLeaderGap)
                        yRef = laneWidth;
                    else
                        yRef = 0;
                    end
                    vyRef = 0;
                    platoonInputs(:, k, :) = platoon.computeInputs('Longitudinal and Lateral', ...
                        platoonControlParams, [yRef; vyRef]);

                end
                followerInputs(:, k) = [follower.singleStepACC(KaccFoll); 0];
                
                % Then we update all states
                leaderTrajectory(:, k) = leader.singleStepUpdate(...
                    leaderInputs(:, k), obj.samplingPeriod);
                platoonTrajectory(:, k, :) = platoon.singleStepUpdate(...
                        platoonInputs(:, k, :), obj.samplingPeriod);
                followerTrajectory(:, k) = follower.singleStepUpdate(...
                    followerInputs(:, k), obj.samplingPeriod);
%                 [followerTrajectory(:, k), followerInputs(:, k)] = follower.singleStepUpdate(...
%                     'ACC', KaccFoll, obj.samplingPeriod);
                
                %%%% CHECKING IF NEW LEADER FINDING FUNCTION WORKS
                % Create matrix with everybody's position
                positions = zeros(2, 1+platoon.nv);
                positions(:, 1) = leaderTrajectory(1:2, k);
%                 for n = 1:platoon.nv
                    positions(:, 2:end) = squeeze(platoonTrajectory(1:2, k,:));
%                 end
                platoonLeaderLeader(k) = platoon.vehs(1).findLeaderBasic(platoonTrajectory(1:2, k, 1), ...
                    [leaderTrajectory(1:2, k), followerTrajectory(1:2, k), squeeze(platoonTrajectory(1:2, k, 2:end))]);
                follLeaderIdx(k) = follower.findLeaderBasic(followerTrajectory(1:2, k), positions);
            end % end of simulation
            
%             plot(obj.t, follLeaderIdx);
            
            % Plots:
            leaderTrajectory = leaderTrajectory'; % to match ploting function format
            followerTrajectory = followerTrajectory';
            platoonTrajectory = permute(platoonTrajectory, [2 1 3]);
            leaderInputs = leaderInputs';
            followerInputs = followerInputs';
            platoonInputs = permute(platoonInputs, [2 1 3]);
            
            platoonLeg = cell(nv, 1);
            for k = 1:nv
                platoonLeg{k} = sprintf('k = %d%', k);
            end
            
            % Longitudinal only
            X(:, :, 1) = [leaderTrajectory(:, 1), leaderTrajectory(:, 3), leaderInputs(:, 1)];
            X(:, :, 2) = [followerTrajectory(:, 1), followerTrajectory(:, 3), followerInputs(:, 1)];
            X(:, :, 3:3+nv-1) = [platoonTrajectory(:, 1, :), platoonTrajectory(:, 3, :), platoonInputs(:, 1, :)];
            statesFig = plot_kinematics(t, X, leader.len, 'Adjacent lane', [obj.legends(1:2), platoonLeg']);
            
            % All states and inputs
            titles = {'Platoon States', 'Platoon Inputs'};
            [~, inputsFig] = plotBicycleModelKinematics(t, platoonTrajectory, platoonInputs, titles, platoonLeg);
        end
        
        function[states_fig, inputs_fig] = testSeqPlatoonLC(obj)
            
            t = obj.simTime;
            
            % Vehicles
            nv = 5; % number of vehicles
            x0 = 0; 
            y0 = 0;
            v0 = 13; % speed in m/s
            theta0 = 0;
            q0 = [x0; y0; v0; theta0];
%             vehicle = BicycleVehicleModel(q0);
            
            % PD gains for lateral control
            polesACC = [1 2];
            
            % ACC gains for forward motion control
            Ks = 1; % space (platoon leader doesn't track any gap)
            Kv = 1; % velocity
%             controller = Controller(vehicle);
%             Kacc = controller.constHeadwayACC(Ks, Kv);
            controlType = 'Longitudinal and Lateral';
            
            controlParams = [Ks, Kv];
            
            % References
%             x_ref = zeros(length(obj.t), 1);
%             vx_ref = v0*ones(length(obj.t), 1);
            % v_ref(obj.t > 5) = 20;
            yRef = zeros(length(t), 1);
            yRef(t > 3) = 4;
            vyRef = zeros(length(t), 1);
            
            % Create platoon
            platoon = BicyclePlatoon(nv);
            platoon.createHomogeneousSSPlatoon(v0, polesACC);
            
            % Simulation NEW standard
            platoonTrajectory = zeros(length(q0), length(t), nv);
            platoonInputs = zeros(2, length(t), nv);
            for k = 1:length(t)
%                 ref = [x_ref(k); vx_ref(k); y_ref(k); vy_ref(k)];
                ref = [yRef(k); vyRef(k)];
                if (t(k)==3)
                    disp('Starting maneuver')
                end
                platoonInputs(:, k, :) = platoon.computeInputs(controlType, ...
                    controlParams, ref);
                platoonTrajectory(:, k, :)= platoon.singleStepUpdate(...
                    platoonInputs(:, k, :), obj.samplingPeriod);
            end

            % Simulation OLD standard
%             [qCheck, uCheck] = platoon.followTheLeaderTrajectory([x_ref, vx_ref, y_ref, vy_ref], [Kacc, Kp, Kd], obj.t);
            
            % Plots
            platoonTrajectory = permute(platoonTrajectory, [2 1 3]);
            platoonInputs = permute(platoonInputs, [2 1 3]);
            leg = cell(nv, 1);
            for k = 1:nv
                leg{k} = sprintf('k = %d%', k);
            end
            titles = {'Platoon States', 'Platoon Inputs'};
            [states_fig, inputs_fig] = plotBicycleModelKinematics(t, ...
                platoonTrajectory, platoonInputs, titles, leg);
            
        end
           
        function[states_fig, inputs_fig] = testBicycleModel(obj)
            
            t = obj.simTime;
            
            % Vehicles
            x0 = 0;
            y0 = 0;
            v0 = 13; % speed in m/s
            theta0 = 0;
            q0 = [x0; y0; v0; theta0];
            vehicle = BicycleVehicleModel(q0);
            
            % PD gains for lateral control
            poles = [1 2];
            vehicle.setController('ACC PD', obj.samplingPeriod, poles);
            
            % ACC gains for forward motion control
            Ks = 1; % space (platoon leader doesn't track any gap)
            Kv = 1; % velocity
            
            latControlParams = [Ks, Kv];
                       
            % References
            yRef = zeros(length(t), 1);
            yRef(t >= 1) = 4;
            vyRef = zeros(length(t), 1);
            
            % Simulation
            trajectory = zeros(length(q0), length(t));
%             trajectorySim = zeros(length(q0), length(obj.t));
            inputs = zeros(2, length(t));
%             inputsSim = zeros(2, length(obj.t));
            for k = 1:length(t)
                ref = [yRef(k); vyRef(k)];
                if (t(k)==1)
                    disp('Starting maneuver')
                end
                [axRef, ayRef] = vehicle.computeAccelRefs(latControlParams, ref);
                [accel, delta] = vehicle.computeInputsFromAccelRef(axRef, ayRef);
                inputs(:, k) = [accel; delta];
                trajectory(:, k) = vehicle.singleStepUpdate(...
                    inputs(:, k), obj.samplingPeriod);
%                 trajectory(:, k) = vehicle.singleStepSimulinkUpdate(...
%                     inputs(:, k), obj.samplingPeriod);
            end
            
            trajectory = trajectory';
            inputs = inputs';
            leg = {};
            titles = {'Vehicle States', 'Vehicle Inputs'};
            [states_fig, inputs_fig] = plotBicycleModelKinematics(t, ...
                trajectory, inputs, titles, leg);
            
        end
        
        %%% GAP GENERATION SIMPLE COST ESTIMATION %%%
        function[resultFig] = ...
                testGapGenerationImpactVsDeterministicSpacing(obj)
            % Test how a simple gap generation approach impacts traffic as
            % a function of rho*minGap
            
            rhoArray = 2;%1.1:0.2:4.5;
            nRho = length(rhoArray);
            obj.T = 200;
            t = obj.simTime;
            
            % Parameters of vehicles on adjacent lane
            nVeh = 15;
            v0 = 20;
            polesACC = [1 2 3]/2;
            
            % Merging platoon parameters
            nP = 5; % vehicles in the merging platoon
            vp = v0; % platoon speed
            mergingPlatoon = LongVehicleArray(nP);
            mergingPlatoon.createHomogeneousSSPlatoon(vp, t, polesACC);
%             platoonLength = nP*(lp+hp*vp+d0p);
            
            % Parameter used to bound speed when generating gap (open-loop)
            gamma = 3/4; % minVel = gamma*desiredVel
            
            affectedVeh = zeros(nRho, 1);
            totalCost = zeros(nRho, 1);
            timeToSS = zeros(nRho, 1);
            
            for n = 1:nRho
                disp(['rho:' num2str(rhoArray(n))])
                
                % Non homogeneous or connected array of vehicles. The first
                % vehicle is Ld, the future leader
                allVehicles = LongVehicleArray(nVeh+1);
                allVehicles.createEquallySpacedVehArray(v0, t, polesACC, rhoArray(n));
                [~, vehArray] = allVehicles.splitLeader();
                
                % Platoon estimates the effects on the other lane
                [affectedVeh(n), totalCost(n), timeToSS(n)] = mergingPlatoon.estimateAdjLaneCosts(gamma, vehArray);
                
                % To speed computation, we only simulate enough vehicles
                % to get the total gap generation effect
                if affectedVeh(n)<nVeh
                    nVeh = affectedVeh(n)+1;
                end
                
            end
            
            resultFig = figure;
            hold on; grid on;
            stem(rhoArray, affectedVeh);
            xlabel('acc gap multiplier');
            ylabel('# affected vehicles');
            figure; hold on; grid on;
            stem(rhoArray, totalCost);
            xlabel('acc gap multiplier');
            ylabel('sum of u(t)^2 for all vehicles')
            figure; hold on; grid on;
            stem(rhoArray, timeToSS);
            xlabel('acc gap multiplier');
            ylabel('effect time')
            
        end
        
        function[resultFig] = testGapGenerationImpactVsRandomSpacing(obj)
            % Test how a simple gap generation approach impacts traffic as
            % a function of rho*minGap
            
            % Since this function might take a while, we load a table,
            % write results and save them on a separate file
            saveProgress = 0;
            resultsTable = readtable('gap_generation_impact_vs_lambda.csv');
            paramStruct = load('gap_generation_impact_params');
            nVeh = paramStruct.nVeh;
            lastLambda = 4.5;
            % If there are no results yet
            if isempty(resultsTable)
                firstLambda = 0.1;
                nVeh = 40;
            else
                firstLambda = resultsTable.Lambda(end) + 0.1;
            end
            % If the table is already filled, we won't fill it again
            if firstLambda>=lastLambda
                saveProgress = 0;
                firstLambda = 0.1;
                nVeh = 40;
            end
            
            % Some simulation parameters
            obj.T = 200;
            t = obj.simTime;
            lambdaArray = firstLambda:0.5:lastLambda;
            nLambda = length(lambdaArray);
            runsPerLambda = 5;
            
            % Parameters of vehicles on adjacent lane
            v0 = 20;
            
            % Merging platoon parameters
            polesACC = [1 2];
            nP = 5; % vehicles in the merging platoon
%             lp = 5; % avg length of vehicles in platoon
%             hp = 1; % avg time headway of platoon
%             d0p = 2; % avg standstill distance of platoon
            vp = v0; % platoon speed
%             platoonLength = nP*(lp+hp*vp+d0p);
            mergingPlatoon = ACCPlatoon(nP);
            mergingPlatoon.createHomogeneousSSPlatoon(vp, t, polesACC);
            
            % Parameter used to bound speed when generating gap (open-loop)
            gamma = 3/4; % minVel = gamma*desiredVel
            
%             accelThreshold = 0.5;
            
            meanAffectedVehIdx = zeros(nLambda, 1);
            meanTotalCost = zeros(nLambda, 1);
            meanTimeToSS = zeros(nLambda, 1);
            varAffectedVehIdx = zeros(nLambda, 1);
            varTotalCost = zeros(nLambda, 1);
            varEffectTime = zeros(nLambda, 1);
            
            affectedVeh = zeros(runsPerLambda, nLambda);
            totalCost = zeros(runsPerLambda, nLambda);
            timeToSS = zeros(runsPerLambda, nLambda);
            for n = 1:nLambda
                disp(['lambda:' num2str(lambdaArray(n))])
                
                for r = 1:runsPerLambda
                    % Non homogeneous or connected array of vehicles. The first
                    % vehicle is Ld, the future leader
                    allVehicles = LongVehicleArray(nVeh+1);
                    allVehicles.createRandomlySpacedVehArray(v0, t, polesACC, lambdaArray(n));
                    [~, vehArray] = allVehicles.splitLeader();
                    
                    % Platoon estimates the effects on the other lane
                    [affectedVeh(n), totalCost(n), timeToSS(n)] = mergingPlatoon.estimateAdjLaneCosts(gamma, vehArray);
                    
                end
                
                meanAffectedVehIdx(n) = mean(affectedVeh(:, n));
                meanTotalCost(n) = mean(totalCost(:, n));
                meanTimeToSS(n) = mean(timeToSS(:, n));
                varAffectedVehIdx(n) = var(affectedVeh(:, n));
                varTotalCost(n) = var(totalCost(:, n));
                varEffectTime(n) = var(timeToSS(:, n));
                
                fprintf(' Affected vehs: %.2f with %.2f var \n Cost: %.2f with %.2f var \n Ripple time: %.2f with %.2f var \n', ...
                    meanAffectedVehIdx(n), varAffectedVehIdx(n), ...
                    meanTotalCost(n), varTotalCost(n),...
                    meanTimeToSS(n), varEffectTime(n));
                
                resultsMatrix = [ones(runsPerLambda, 1)*lambdaArray(n), affectedVeh(:, n), ... varAffectedVehIdx(n), ...
                    totalCost(:, n), ... varTotalCost(n),...
                    timeToSS(:, n)]; ...varEffectTime(n)];
                newTable = array2table(resultsMatrix, 'VariableNames', resultsTable.Properties.VariableNames);
                resultsTable = [resultsTable; newTable];
                if saveProgress
                    writetable(resultsTable, 'gap_generation_impact_vs_lambda.csv');
                end
                
                % To speed computation, we only simulate enough vehicles
                % and enough time to get the total gap generation effect
                % We give some margin to both values just in case
                if meanAffectedVehIdx(n)<nVeh
                    nVeh = max(ceil(meanAffectedVehIdx(n)+3*sqrt(varAffectedVehIdx(n))), 3);
                end
                
                save('gap_generation_impact_params', 'nVeh');
            end
            
            resultFig(1) = figure;
            hold on; grid on;
            errorbar(lambdaArray, meanAffectedVehIdx, sqrt(varAffectedVehIdx));
            xlabel('mean acc gap multiplier');
            ylabel('# affected vehicles');
            resultFig(2) = figure; hold on; grid on;
            errorbar(lambdaArray, meanTotalCost, sqrt(varTotalCost));
            xlabel('mean acc gap multiplier');
            ylabel('sum of a(t)^2 for all vehicles')
            resultFig(3) = figure; hold on; grid on;
            errorbar(lambdaArray, meanTimeToSS, sqrt(varEffectTime));
            xlabel('mean acc gap multiplier');
            ylabel('effect time')
            
        end
        
        function[resultFig] = testGapGenerationImpactVsN(obj)
            % Test how a simple gap generation approach impact traffic
            
            obj.T = 200;
            t = obj.simTime;
            % Parameters of vehicles on adjacent lane
            nVeh = 25; % not counting Ld
            v0 = 20;
            
            % ACC controller params
%             varPoles = diag([0 0]);
            polesACC = [1 2]/2; %abs(repmat([1 2], nVeh+1, 1) + randn(nVeh+1, 2)*varPoles);
            
            % Merging platoon parameters
            maxNP = 5; % vehicles in the merging platoon
            vp = v0;
            
            % Non homogeneous or connected array of vehicles. The first
            % vehicle is Ld, the future leader
            meanExtraGap = 0;
            allVehicles = LongVehicleArray(nVeh+1);
            allVehicles.createRandomlySpacedVehArray(v0, t, polesACC, meanExtraGap);
            [ld, vehArray] = allVehicles.splitLeader();
            
            % Scenario 1: vehicle brakes to a min speed and keeps that
            % speed until the gap is open. It then starts vehicle following
            % its future leader (last platoon vehicle on adjacent lane)
            
            gamma = 3/4; % minVel = gamma*desiredVel
            testedNs = 1:maxNP; % just to make it easy and quick to run tests
            
            affectedVeh = zeros(maxNP, 1);
            totalCost = zeros(maxNP, 1);
            timeToSS = zeros(maxNP, 1);
            for n = testedNs%1:maxNP
                disp(['#platoon vehs:' num2str(n)])
                
                ld.resetStates();
                vehArray.resetStates();
                
                % Merging platoon parameters
                mergingPlatoon = LongVehicleArray(n);
                mergingPlatoon.createHomogeneousSSPlatoon(vp, t, polesACC);
                
                % Platoon estimates the effects on the other lane
                [affectedVeh(n), totalCost(n), timeToSS(n)] = mergingPlatoon.estimateAdjLaneCosts(gamma, vehArray);
                
            end
            
            resultFig(1) = figure;
            hold on; grid on;
            stem(testedNs, affectedVeh);
            xlabel('# platoon vehs');
            ylabel('# affected vehicles');
            resultFig(2) = figure; hold on; grid on;
            stem(testedNs, totalCost);
            xlabel('# platoon vehs');
            ylabel('sum of a(t)^2 for all vehicles')
            resultFig(3) = figure; hold on; grid on;
            stem(testedNs, timeToSS);
            xlabel('# platoon vehs');
            ylabel('effect time')
            
%             % plot
%             X = [permute(vehsTrajectory, [2 1 3]), permute(vehArrayInputs, [2 1 3])];
%             leg = cell(nVeh, 1);
%             for k = 1:nVeh
%                 leg{k} = sprintf('k = %d%', k);
%             end
%           plot_kinematics(obj.t, X, vehArray.vehs(1).len, 'Veh Array', leg);
            
            
        end
        
        %%% CACC CHECKS %%%
        function [] = checkCACCLawNoIntegrator(~, k1, k5)
                                    
            % Actuator lag
            tau = 0.5; % unkown to controller
            maxTau = 1; % estimate used by controller
            
            % Headway: arbitrarily chosen and known by controller
            h = 1; % [s]
            
            % Analytical sufficiency check for string stability
            if k1*h<maxTau
                k1 = maxTau/h+1;
                warning('k1.h < tau. Setting k1 = %g', k1)
            end
            
            % Following paper definitions 
            k2 = 1/h;
            k3 = k1*k5;
            k4 = k5/h;
            
            
            % Stability
            A = [0, 1, 0, 0;...
                -k4*h/tau, (1-k3*h)/tau, (1-k2*h)/tau, 1-k1*h/tau; ...
                0, 0, 0, 1;...
                -k4/tau, (1/h-k3)/tau, (1/h-k2)/tau, -k1/tau];
            B = [0; 0; 0; 1];
            Bd = [0; -h/tau; 0; -1/tau];
            C = eye(4);
            D = 0;
            
            GLeaderJerk = tf(ss(A, B, C, D));
            GDisturbance = tf(ss(A, Bd, C, D));
            
            figure('Name', 'Response to Leader Jerk')
            step(GLeaderJerk)
            figure('Name', 'Response to Disturbance Jerk')
            step(GDisturbance)
            
            step(GLeaderJerk)
            step(GDisturbance);
            
            % String stability test
            G = tf([k1*h, 1+k1*k5*h, k5],[tau*h, h*(1+k1+k1*k5*h), 1+k1*k5*h+k5*h, k5]);
            if (k1*h>maxTau)
                figure('Name', 'Freq Response to Leader Disturbance')
                bode(G);
            else
                warning('String stability condition not met')
            end
            
        end
        
        function [] = checkCACCLaw(~, k1, k5, k6)
                                    
            % Actuator lag
            tau = 0.5; % unkown to controller
            maxTau = 1; % estimate used by controller
            minTau = 0.3; % estimate used by controller
            
            % Headway: arbitrarily chosen and known by controller
            h = 2.5; % [s]
            
            % Analytical sufficiency check for string stability
            oldK1 = k1;
            oldK6 = k6;
            if k1*h<maxTau
                k1 = maxTau/h+1;
                warning('k1.h < tau. Setting k1 = %g', k1)
            end
            upperBoundK6 = k5^2/2/(h*(k1*k5*h+k1+1)-minTau);
            if k6 > upperBoundK6
                k6 = upperBoundK6*0.9;
                warning('k6 > %g. Setting k6 = %g', upperBoundK6, k6)
            end
            
            % Following paper definitions 
            k2 = 1/h;
            k3 = k1*k5;
            k4 = k5/h;
             
            % Stability
            A = [0, 1, 0, 0, 0;...
                0, 0, 1, 0, 0;...
                -k6*h/tau, -k4*h/tau, (1-k3*h)/tau, (1-k2*h)/tau, 1-k1*h/tau; ...
                0, 0, 0, 0, 1;...
                -k6/tau, -k4/tau, (1/h-k3)/tau, (1/h-k2)/tau, -k1/tau];
            B = [0; 0; 0; 0; 1];
            Bd = [0; 0; -h/tau; 0; -1/tau];
            C = eye(5);
            D = 0;
            
            GLeaderJerk = tf(ss(A, B, C, D));
            GDisturbance = tf(ss(A, Bd, C, D));
            
%             figure('Name', 'Response to Leader Jerk')
%             step(GLeaderJerk)
%             figure('Name', 'Response to Disturbance')
%             step(GDisturbance)
%             
%             % String Stability
%             if k1~=oldK1 || k6~=oldK6
%                 G = tf([oldK1*h, 1+oldK1*k5*h, k5 oldK6*h],[tau*h, h*(1+oldK1+oldK1*k5*h), 1+oldK1*k5*h+k5*h, k5+oldK6*h^2, oldK6*h]);
%                 figure('Name', 'Freq Response to Leader Disturbance (conditions not met)')
%                 bode(G);
%             end
%             G = tf([k1*h, 1+k1*k5*h, k5 k6*h],[tau*h, h*(1+k1+k1*k5*h), 1+k1*k5*h+k5*h, k5+k6*h^2, k6*h]);
%             figure('Name', 'Freq Response to Leader Disturbance')
%             bode(G);
                        
        end
        
        
        %%% ACC TESTS %%%
        function [platoon, result_fig] = testInfTimeQueueLQR(obj)
            
            warning(['This scenario has an issue caused by the imposed '...
                'speed limit'])
            %%% First note: this is not a relevant scenario so we can
            %%% disregard the issue.
            %%% Further description of the issue:
            %%% Since the leader accelerates a lot after initial braking,
            %%% it causes its follower to accelerate to a higher than
            %%% desired velocity. This, in turn, brakes the platoon.
            %%% Deactiviting this limit causes problems for the scenario
            %%% where we want non-homogeneous gaps. 
            
            t = obj.simTime;
            
            % Create platoon
            nv = 6; % including the future leader
            platoon = ACCPlatoon(nv-1);
            
            poles = [1 2];
            platoon.createHomogeneousSSPlatoon(obj.vf0, obj.maxVel, t, ...
                poles);
            ssGap = platoon.leader.len + platoon.leader.minACCGap;
            x0 = [platoon.leader.currentState(1)+ssGap; obj.vl0];
            % veh in front of platoon:
            indepLeader = LongitudinalVehicleModel('leader', t, x0); 
            
            % The cooperative leader of the platoon is also following
            % someone from whom it will create a gap
            platoon.leader.leader = indepLeader;
            
            % Cost weights
            ws1 = 1;
            wv1 = 1;
            wa = 2;
            w = [ws1; zeros(nv-2,1); wv1; wa*ones(nv-2,1)];
            r = 2;
            % Controller gains
            platoon.leader.setController('Queue LQR', {w, r});
            
            % The leader just moves forward with no acceleration
            indepLeader.setController('OpenLoop', zeros(length(t), 1));
            
            newGap = 2*ssGap;
            
            for k = 2:length(t)
                indepLeader.singleStepUpdate();
                platoon.singleStepUpdate(newGap);
%                 indepLeader.computeInput();
%                 platoon.computeInputs(newGap);
            end
            result_fig = platoon.plotStates({'u', 'velocity', 'gap'});

        end
        
        function [vehArrays, resultFigs] = testVehArray(obj)
            
            t = obj.simTime;
            
            % Controller poles
            poles = [1 2];
            
            % Initialize platoon object
            nVeh = 5;
            platoon1 = LongVehicleArray(nVeh);
                        
            %%% Scenario 1: Leading vehicle brakes to a full stop or %%%
            %%% sinusoidal accel %%%
            % Platoon initial state
            v0 = 20; % m/s
            vMax = 30; % m/s
            platoon1.createHomogeneousSSPlatoon(v0, obj.maxVel, t, poles);
            platoon1.setDesiredVel(vMax);
            
            % Leader inputs
%             brakeT0 = 3;
%             brakeTf = brakeT0-v0/platoon.leader.accelBounds(1);
%             leaderAccel = zeros(length(obj.t), 1);
%             leaderAccel(obj.t>brakeT0 & obj.t <brakeTf) = ...
%               platoon.leader.accelBounds(1);
            leaderAccel = min(abs((platoon1.vehs(1).accelBounds)))*sin(t);
            platoon1.vehs(1).setController('OpenLoop', leaderAccel);
            
            % Simulation
            for k = 2:length(t)               
                platoon1.singleStepUpdate();
%                 platoon1.computeInputs();
            end
            
            % Plot results
            [~, platoon1] = platoon1.splitLeader();
            resultFigs(1) = platoon1.plotStatesAllVehs(...
                {'u', 'velocity', 'gap'});
            platoon1.plotStatesAllVehs({'errors'});
            
            %%% Scenario 2:  all start with zero speed, accelerate and then
            % brake %%%
            % Platoon initial state
            platoon2 = LongVehicleArray(nVeh);
            v0 = 0;
            vMax = 25; % m/s (approx 60mph)
            platoon2.createHomogeneousSSPlatoon(v0, obj.maxVel, t, poles);
            platoon2.setDesiredVel(vMax);
            
            % Leader inputs
            accelT0 = 1;
            accelTf = accelT0 + vMax/platoon2.vehs(1).accelBounds(2);
            brakeT0 = accelTf + 1;
            brakeTf = brakeT0 - vMax/platoon2.vehs(1).accelBounds(1);
            leaderAccel = zeros(length(t), 1);
            leaderAccel(t>accelT0 & t <accelTf) = ...
                platoon2.vehs(1).accelBounds(2);
            leaderAccel(t>brakeT0 & t <brakeTf) = ...
                platoon2.vehs(1).accelBounds(1);
            platoon2.vehs(1).setController('OpenLoop', leaderAccel);
            
            % Simulation
            for k = 2:length(t)               
                platoon2.singleStepUpdate();
%                 platoon2.computeInputs();
            end
            
            % Plot results
            [~, platoon2] = platoon2.splitLeader();
            resultFigs(2) = platoon2.plotStatesAllVehs(...
                {'u', 'velocity', 'gap'});
            platoon2.plotStatesAllVehs({'errors'});
            
            vehArrays = [platoon1, platoon2];
        end
        
        function [vehArrays] = testACC(obj)
            
            t = obj.simTime;
            poles = [1 2 3]/2;

            %%% Scenario 1: Leading vehicle brakes to a full stop
            vehArray1 = LongVehicleArray(2);
            vehArray1.createHomogeneousSSPlatoon(obj.vf0, obj.maxVel, ...
                t, poles);
            
            leader = vehArray1.vehs(1); %vehicles(1);
            follower = vehArray1.vehs(2); %vehicles(2);

            % Leader inputs
            brake_t0 = 3;
            brake_tf = brake_t0 -obj.vl0/leader.accelBounds(1);
            ul = zeros(length(t), 1);
            ul(t>brake_t0 & t <brake_tf) = leader.accelBounds(1);
            leader.setController('OpenLoop', ul);
            
            % System trajectory
            for k = 2:length(t) % k=1 (the initial state) is already set
                follower.singleStepUpdate();
                leader.singleStepUpdate();
            end

            vehArray1.plotStates({'u', 'velocity', 'gap'});
            
            %%% Scenario 2: both start with zero speed, accelerate and then
            %%% brake
            desiredVel = 25; % m/s (approx 60mph)
            vehArray2 = LongVehicleArray(2);
            vehArray2.createHomogeneousSSPlatoon (0, obj.maxVel, t, poles);
            leader = vehArray2.vehs(1);
            follower = vehArray2.vehs(2);
            
            follower.desiredVelocity = desiredVel;
            leader.desiredVelocity = desiredVel;

            % Leader inputs            
            accel_tf = desiredVel/leader.accelBounds(2);
            brake_t0 = accel_tf + 1;
            brake_tf = brake_t0 - desiredVel/leader.accelBounds(1);
            ul = zeros(length(t), 1);
            ul(t <accel_tf) = leader.accelBounds(2);
            ul(t>brake_t0 & t <brake_tf) = leader.accelBounds(1);
            leader.setController('OpenLoop', ul);
            
            for k = 2:length(t)
                follower.singleStepUpdate();
                leader.singleStepUpdate();
%                 leader.computeInput();
%                 follower.computeInput();
            end
            vehArray2.plotStates({'u', 'velocity', 'gap'});
            
            vehArrays = [vehArray1, vehArray2];
            
        end
                        
        function [vehArray, resultFig] = testOpenLoopControllers(obj, ...
                control_type, control_params)

            % Possible plot titles
            control_type_name = {'Min time', 'Min time and power', ...
                'Min time and fuel'};
            
            % Create objects
            poles = [1 2];
            vehArray = LongVehicleArray(2);
            vehArray.createHomogeneousSSPlatoon (obj.vf0, obj.t, poles);
            leader = vehArray.vehs(1); 
            follower = vehArray.vehs(2);
            
            leader.setController('OpenLoop', zeros(length(obj.t), 1));
            follower.setController('OpenLoop', []);
            
            new_gap = 2*follower.minACCGap; % assuming homogeneous vehicles
            % Controls
            switch control_type
                case 1 % min time
%                     u = 
                    follower.controller.minimum_time_control(new_gap, ...
                        obj.t);
                case 2 % min time and power
                    if nargin == 3
                        rp = control_params;
                    else
                        rp = 0.2;
                    end
%                     u = 
                    follower.controller.min_time_and_power(new_gap, rp, ...
                        obj.t);
                case 3 % min time and fuel
                    if nargin == 3
                        rf = control_params;
                    else
                        rf = 0.1;
                    end
%                     u = 
                    follower.controller.min_time_and_fuel(new_gap, rf, ...
                        obj.t);
                otherwise
                    error(['There are only 3 possible open loop '...
                        'controllers.\n Please choose 1, 2 or 3.'])
            end

            
%             leaderTrajectory = zeros(leader.nStates, length(obj.t));
%             followerTrajectory = zeros(follower.nStates, length(obj.t));
%             accelLead = zeros(length(obj.t), 1);
%             accelFoll = zeros(length(obj.t), 1);
            for k = 2:length(obj.t)
%                 accelLead(k) = leader.computeInput();
%                 accelFoll(k) = follower.computeInput();
%                 leaderTrajectory(:, k) = 
                leader.singleStepUpdate();
%                 followerTrajectory(:, k) = 
                follower.singleStepUpdate();
                leader.computeInput();
                follower.computeInput();
            end
            resultFig = vehArray.plotStates({'u', 'velocity', 'gap'});
            resultFig.Name = control_type_name{control_type};
%             X(:, :, 1) = [leaderTrajectory', accelLead];
%             X(:, :, 2) = [followerTrajectory', accelFoll];
%             result_fig = plot_kinematics(obj.t, X, leader.len, ...
%               control_type_name{control_type}, obj.legends(1:2));
        end
        
        function [] = testFilters(obj)
            
            t = obj.simTime;
            poles = [2 1];
            q0 = [0; 0];
            veh = LongitudinalVehicleModel(q0);
            veh.desiredVelocity = 30;
            veh.setController('ACC PD',  obj.samplingPeriod, poles); 
            
           % System trajectory
            vehTrajectory = zeros(veh.nStates, length(t));
            vehInput = zeros(1, length(t));
            for k = 1:length(t)
                vehInput(k) = veh.computeInput();
                vehTrajectory(:, k) = veh.singleStepUpdate(vehInput(k), ...
                    obj.samplingPeriod);
            end

            figure; 
            subplot(3, 1, 1)
            hold on; grid on;
            plot(t, vehInput);
            ylabel('accel');
            subplot(3, 1, 2)
            hold on; grid on;
            plot(t, vehTrajectory(2, :));
            ylabel('vel');
            subplot(3, 1, 3)
            hold on; grid on;
            plot(t, vehTrajectory(1, :));
            ylabel('position');
        end
        
        function simTime = get.simTime(obj)
            simTime = 0:obj.samplingPeriod:obj.T;
        end
                
    end
    
    methods (Static)
        function [leader, follower] = create_kinematic_vehicles(...
                leaderType, followerType, leaderSpeed, followerSpeed, ...
                gamma, freeFlowVelocity)
            %create_kinematic_vehicles Creates a follower and a leader of 
            %the desired types and set follower's max brake to 1/gamma of
            %the leader's max brake
            
            vL0 = leaderSpeed;
            vF0 = followerSpeed;
            rho = 0.1;
            
            leader = KinematicVehicle('leader', leaderType, vL0);
            follower = KinematicVehicle('follower', followerType, vF0);
            if nargin>4
                follower.maxBrake = leader.maxBrake/gamma;
            end
            if nargin>5
                follower.desiredVelocity = freeFlowVelocity;
            else
                follower.desiredVelocity = follower.vx0;
            end
            
            follower.setLeaderAndTimeHeadway(leader, rho);
        end
        
%         function [vehicles] = create_vehicles(obj, nVeh)
%             vehicles(nVeh, 1) = LongitudinalVehicleModel();
%             % All vehicle gaps are initialized at the ACC steady state
%             x0 = [obj.sf0; obj.vf0];
%             
%             for n = 1:nVeh
%                 vehicles(n) = LongitudinalVehicleModel(x0, ...
%                   ['veh ' num2str(n)], obj.t);
% %                 vehicles(n).accelBounds = [obj.amin, obj.amax];
% %                 ss_gap = vehicles(n).tg*vehicles(n).initState(2) ...
% %                     + vehicles(n).len + vehicles(n).g0;
%                 ssGap = vehicles(n).minACCGap + vehicles(n).len; 
%                 x0 = x0 + [ssGap; 0];
%             end
%             vehicles = flip(vehicles);
%             % We assign leaders to each vehicle except the first
%             for n = 2:nVeh
%                 vehicles(n).leader = vehicles(n-1);
%             end
%         end
        
    end
    
end