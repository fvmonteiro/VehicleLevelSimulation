classdef WorstCaseAnalyzer < handle
    %WorstCaseAnalyzer Class to generate all plots relative to analysis of
    %the worst case scenario
    
    properties
        saveResults = 0
        fileFormat = 'eps'
    end
    
    properties (Constant, Hidden)
        safetyImageFolder = 'G:\My Drive\Safety in Mixed Traffic\images\';

        possibleFormats = {'eps', 'png', 'jpg'};
        mpsToKmph = 3.6;
    end
    
    methods
        function [] = plotWorstCaseCollisionSeverities(obj, gamma)
            
            type = 'PV';
            leaderType = type;
            followerType = type;
            baseVelocity = 20;
            freeFlowVelocity = baseVelocity*1.2;
            vL0 = baseVelocity;
            vF0 = baseVelocity;
            
            leader = KinematicVehicle('leader', leaderType, vL0);
            
            severity = cell(length(gamma), 1);
            g0Array = cell(length(gamma), 1);
            follower = KinematicVehicle(length(gamma));
            for n = 1:length(gamma)
                follower(n) = KinematicVehicle('follower', followerType, vF0);
                follower(n).desiredVelocity = freeFlowVelocity;
                follower(n).leader = leader;
                follower(n).maxBrake = leader.maxBrake/gamma(n);
                [severity{n}, g0Array{n}] = ...
                    follower(n).computeAnalyticalSeverityProfile();
                % Compute total severity of the maneuver (not just for one
                % vehicle)
                severity{n} = severity{n}*(follower(n).m + leader.m)...
                    /max(follower(n).m, leader.m);
            end
            
            % Plot
            gapThresholdNames = {'G_1', 'G_2', 'G_3', 'g^*'};
            figBaseName = 'worst_case_severity';
            for n = 1:length(gamma)
                figHandle = figure;
%                 set(figHandle, 'WindowStyle', 'Docked');
%                 subplot(1, length(gamma), n);
                hold on; grid on;
                ax = gca;
                set(ax, 'GridAlpha', 1);
                set(ax, 'FontSize', 14);
                plot(g0Array{n}, severity{n}*ResultAnalysis.mpsToKmph, ...
                    'LineWidth', 1.5);
                xlabel('Initial gap [m]')
                ylabel('Severity [km/h]')
                yticks([])
                xticks(follower(n).gapThresholds(...
                    follower(n).gapThresholds>0));
                xticklabels(gapThresholdNames(follower(n).gapThresholds>0))
                if gamma(n) < 1
                    figName = [figBaseName '_ego_brakes_harder'];
                elseif gamma(n) > 1
                    figName = [figBaseName '_leader_brakes_harder'];
                else
                    figName = [figBaseName 'same_braking'];
                end
                obj.saveToSafetyFolder(figHandle, figName);
            end
            
        end
        
        function [] = plotEstimatedCollisionSeverities(obj, gamma)
            
            type = 'PV';
            leaderType = type;
            followerType = type;
            baseVelocity = 20;
            freeFlowVelocity = baseVelocity*1.2;
            vL0 = baseVelocity;
            vF0 = baseVelocity;
            rho = 0.1;
            
            leader = KinematicVehicle('leader', leaderType, vL0);
            
            exactSeverity = cell(length(gamma), 1);
            exactGap0Array = cell(length(gamma), 1);
            estimatedSeverity = cell(length(gamma), 1);
            estimatedGap0Array = cell(length(gamma), 1);
            follower = KinematicVehicle(length(gamma));
            for n = 1:length(gamma)
                follower(n) = KinematicVehicle(...
                    'follower', followerType, vF0);
                follower(n).desiredVelocity = freeFlowVelocity;
                if gamma > 1
                    % Makes plot look nicer
                    follower(n).jerkBounds = [-20, 50];
                end
                follower(n).leader = leader;
                follower(n).maxBrake = leader.maxBrake/gamma(n);
                [exactSeverity{n}, exactGap0Array{n}] = ...
                    follower(n).computeAnalyticalSeverityProfile();
                [estimatedSeverity{n}, estimatedGap0Array{n}] = ...
                    follower(n).estimateSeverity(rho);
                % Compute total severity of the maneuver (not just for one
                % vehicle)
                massRatio = max(follower(n).m, leader.m)...
                    /(follower(n).m + leader.m);
                exactSeverity{n} = exactSeverity{n}/massRatio;
                estimatedSeverity{n} = estimatedSeverity{n}/massRatio;
            end
            
            % Plot
            gapThresholdNames = {'$G_1$', '$G_2$', '$G_3$', '$g^*$'};
            figBaseName = 'estimated_severity';
            for n = 1:length(gamma)
                figHandle = figure;
                %                 set(figHandle, 'WindowStyle', 'Docked');
                hold on; grid on;
                ax = gca;
                set(ax, 'GridAlpha', 1);
                set(ax, 'ticklabelinterpreter', 'latex')
                set(ax, 'FontSize', 12);
                
                plot(exactGap0Array{n}, ...
                    exactSeverity{n}*ResultAnalysis.mpsToKmph, ...
                    'LineWidth', 1.5);
                plot(estimatedGap0Array{n}, ...
                    estimatedSeverity{n}*ResultAnalysis.mpsToKmph, ...
                    'LineWidth', 1.5);
                xlabel('Initial gap [m]')
                ylabel('Severity [km/h]')
                legend({'Exact', 'Estimated'}, 'Fontsize', 12)
                yticks([]);
                xticks([]);
                xticks(follower(n).gapThresholds(...
                    follower(n).gapThresholds>0));
                xticklabels(gapThresholdNames(follower(n).gapThresholds>0))
                if gamma(n) < 1
                    figName = [figBaseName '_ego_brakes_harder'];
                elseif gamma(n) > 1
                    figName = [figBaseName '_leader_brakes_harder'];
                else
                    figName = [figBaseName 'same_braking'];
                end
                obj.saveToSafetyFolder(figHandle, figName);
            end
            
        end
        
        function [] = plotTimeHeadwayVsRisk(obj)
            
            type = 'PV';
            leaderType = type;
            followerType = type;
            baseVelocity = 20;
            freeFlowVelocity = baseVelocity*1.2;
            vL0 = baseVelocity;
            vF0 = baseVelocity;
            gamma = [0.7, 1/0.8];
            rho = 0.1;
            
            leader = KinematicVehicle('leader', leaderType, vL0);
            
            exactSeverity = cell(length(gamma), 1);
            hCollisionFree = cell(length(gamma), 1);
            acceptedRisk = cell(length(gamma), 1);
            hRiskEstimate = cell(length(gamma), 1);
            follower = KinematicVehicle(length(gamma));
            for n = 1:length(gamma)
                follower(n) = KinematicVehicle('follower', ...
                    followerType, vF0);
                follower(n).desiredVelocity = freeFlowVelocity;
                follower(n).maxBrake = leader.maxBrake/gamma(n);
                follower(n).setLeaderAndTimeHeadway(leader, rho);
                
                massRatio = max(follower(n).m, leader.m)...
                    /(follower(n).m + leader.m);
                [exactSeverity{n}, g0Array] = ...
                    follower(n).computeAnalyticalSeverityProfile();
                hCollisionFree{n} = (g0Array - follower(n).d0)...
                    /follower(n).vx0;
                exactSeverity{n} = exactSeverity{n}/massRatio;
                
                maxRisk = sqrt(follower(n).minVFGap0*2 ...
                    *follower(n).maxBrake);
                threshold = (1-rho)*freeFlowVelocity...
                    /(freeFlowVelocity+follower(n).lambda1);
                if gamma(n)<=threshold
                    maxRisk = maxRisk*sqrt(1-gamma(n));
                end
                acceptedRisk{n} = [0:0.01:maxRisk maxRisk];
                hRiskEstimate{n} = ...
                    follower(n).computeTimeHeadwayWithRisk(...
                    acceptedRisk{n}, rho);
                
            end
            
            % Plot
            figBaseName = 'time_headway_vs_risk';
            for n = 1:length(gamma)
                figHandle = figure;
                %                 set(figHandle, 'WindowStyle', 'Docked');
                hold on;
                ax = gca;
                set(ax, 'GridAlpha', 1);
                set(ax, 'FontSize', 12);
                plot(exactSeverity{n}, ...
                    hCollisionFree{n}*ResultAnalysis.mpsToKmph, ...
                    'LineWidth', 1.5);
                plot(acceptedRisk{n}, ...
                    hRiskEstimate{n}*ResultAnalysis.mpsToKmph, ...
                    'LineWidth', 1.5);
                xlabel('Severity [km/h]')
                ylabel('Time headway [km/h]')
                legend({'Collision Free h', 'Estimated h'}, 'Fontsize', 12)
                %                 yticks([])
                %                 xticks([]);
                if gamma(n) < 1
                    figName = [figBaseName '_ego_brakes_harder'];
                elseif gamma(n) > 1
                    figName = [figBaseName '_leader_brakes_harder'];
                else
                    figName = [figBaseName 'same_braking'];
                end
                obj.saveToSafetyFolder(figHandle, figName);
            end
            
        end
        
        function [] = plotFlowVsRisk(obj)
            %plotFlowVsRisk Plots the maximum flow q = v.C, where v is
            %speed and C capacity, versus accepted risk for three cases:
            % 1. Theoretical maximum, assuming exact collision-free gaps
            % 2. Connected vehicles, which have smaller reaction time and
            % rho
            % 3. Autonomous vehicles, which have higher reaction time and
            % rho
            
            % Parameters
            type = 'PV';
            baseVelocity = 25;
            freeFlowVelocity = 30;
            connectedRho = 0.1;
            connectedReactionTime = 0.2;
            autonomousRho = 0.2;
            autonomousReactionTime = 0.4;
            
            % Vehicles
            leader = KinematicVehicle('veh', type, baseVelocity);
            follower = KinematicVehicle('veh', type, baseVelocity);
            follower.desiredVelocity = freeFlowVelocity;
            
            % Upper bound on flow
            follower.leader = leader;
            exactMaxRisk = sqrt(2*follower.maxBrake...
                *follower.computeAnalyticalCollisionFreeGap(leader));
            exactRisk = 0:0.01:exactMaxRisk;
%             gamma = 1/0.8;
%             follower.maxBrake = leader.maxBrake / 0.8;
%             [exactRisk, exactGap0Array] = ...
%                     follower.computeAnalyticalSeverityProfile();
            maxFlow = zeros(length(exactRisk), 1);
            for iR = 1:length(exactRisk)
                capacity = 1000/(follower.len ...
                    + follower.computeSimplifiedRiskyGap(exactRisk(iR)));
                maxFlow(iR) = follower.vx0*obj.mpsToKmph*capacity;
            end
            
            % Flow with connected vehicles
            leader.maxBrake = 1.1*follower.maxBrake;
            follower.reactionTime = connectedReactionTime;
            follower.setLeaderAndTimeHeadway(leader, connectedRho);
            connectedMaxRisk = sqrt(2*follower.maxBrake*follower.minVFGap0);
            connectedRisk = 0:0.01:connectedMaxRisk;
            connectedFlow = zeros(length(connectedRisk), 1);
            for iR = 1:length(connectedRisk)
                hr = follower.computeTimeHeadwayWithRisk(...
                    connectedRisk(iR), connectedRho);
                riskyGap = hr*follower.vx0 + follower.d0;
                capacity = 1000/(follower.len + riskyGap);
                connectedFlow(iR) = follower.vx0*obj.mpsToKmph*capacity;
            end
            
            % Flow with automated vehicles
            leader.maxBrake = 1.2*follower.maxBrake;
            follower.reactionTime = autonomousReactionTime;
            follower.setLeaderAndTimeHeadway(leader, autonomousRho);
            autonomousMaxRisk = sqrt(2*follower.maxBrake*follower.minVFGap0);
            autonomousRisk = 0:0.01:autonomousMaxRisk;
            autonomousFlow = zeros(length(autonomousRisk), 1);
            for iR = 1:length(autonomousRisk)
                hr = follower.computeTimeHeadwayWithRisk(...
                    autonomousRisk(iR), connectedRho);
                riskyGap = hr*follower.vx0 + follower.d0;
                capacity = 1000/(follower.len + riskyGap);
                autonomousFlow(iR) = follower.vx0*obj.mpsToKmph*capacity;
            end
            
            % Plot
            figHandle = figure; hold on; grid on;
            ax = gca;
            set(ax, 'ticklabelinterpreter', 'latex')
            set(ax, 'FontSize', 12);
            plot(exactRisk*obj.mpsToKmph, maxFlow, 'LineWidth', 1.5)
            plot(connectedRisk*obj.mpsToKmph, connectedFlow, ...
                'LineWidth', 1.5)
            plot(autonomousRisk*obj.mpsToKmph, autonomousFlow, ...
                'LineWidth', 1.5)
            legend({'Flow Upper Bound', 'Flow Connected Vehicles', ...
                'Flow Autonomous Vehicles'}, ...
                'Location', 'Best')
            xlabel('risk [km/h]')
            ylabel('flow [veh/km]')
            
            figName = 'flow_vs_risk';
            obj.saveToSafetyFolder(figHandle, figName)
        end
        
        function [] = plotLaneChangeTradeOff(obj, gamma)
            %plotLaneChangeTradeOff Plots the bounds on mobility cost and 
            %maneuver risk as function of accepted risk
            
            type = 'PV';
            leaderType = type;
            followerType = type;
            baseVelocity = 20; % [m/s]
            freeFlowVelocity = baseVelocity*1.2;
            vL0 = baseVelocity;
            vF0 = baseVelocity;
            rho = 0.2;
            maxLaneChangeVel = freeFlowVelocity;
            laneChangeDuration = 5; %[s]
            epsilon = 0.05;
            
            leader = KinematicVehicle('leader', leaderType, vL0);
            follower = KinematicVehicle('follower', followerType, vF0);
            follower.desiredVelocity = freeFlowVelocity;
            follower.maxBrake = leader.maxBrake/gamma;
            follower.setLeaderAndTimeHeadway(leader, rho);
            isLaneChanging = true;
            follower.computeTimeHeadway(maxLaneChangeVel, rho, ...
                isLaneChanging);
            
            maxLaneChangeBrake = abs(min(follower.accelBounds));
            gammaLaneChange = leader.maxBrake/maxLaneChangeBrake;
            threshold = (1-rho)*maxLaneChangeVel...
                /(maxLaneChangeVel+follower.lambda1LC);
            if gammaLaneChange > threshold
                maxAcceptedRisk = sqrt(...
                    2*maxLaneChangeVel*maxLaneChangeBrake...
                    *(follower.hLC - follower.h - epsilon));
            else
                maxAcceptedRisk = sqrt(...
                    2*maxLaneChangeVel*maxLaneChangeBrake...
                    *(1-gammaLaneChange)...
                    *(follower.hLC - follower.h - epsilon));
            end
            
            acceptedRisk = 0:0.01:maxAcceptedRisk;
            
            hr = zeros(length(acceptedRisk), 1);
            for iR = 1:length(acceptedRisk)
                hr(iR) = follower.computeTimeHeadwayWithRisk(...
                    acceptedRisk(iR), rho, ...
                    isLaneChanging, maxLaneChangeVel);
            end
            transitionTime = log((hr - follower.h)/epsilon)...
                /follower.hFilterGain;
            mobilityCost = 2*(hr - follower.h)*maxLaneChangeVel.*...
                (transitionTime ...
                + (epsilon./(hr - follower.h)-1)/follower.hFilterGain);
            maneuverRisk = acceptedRisk*laneChangeDuration;
            
            figHandle = figure;
            hold on; grid on;
            ax = gca;
            set(ax, 'ticklabelinterpreter', 'latex')
            set(ax, 'FontSize', 12);
            xlabel('accepted risk [m/s]')
            plot(acceptedRisk, mobilityCost, 'LineWidth', 1.5);
            ylabel('M [m.s]')
            yyaxis right
            plot(acceptedRisk, maneuverRisk, 'LineWidth', 1.5);
            ylabel('R [m]')
            
            legend({'mobility cost bound', 'maneuver risk bound'}, ...
                'Location', 'North')
            
            figName = 'mobility_risk_trade_off';
            obj.saveToSafetyFolder(figHandle, figName)
        end
        
        %%% Exporting functions %%% 
        
        function [] = saveToSafetyFolder(obj, figHandle, ...
                figName, enlargeFig)
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
    end
end

