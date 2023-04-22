classdef CBFScenarios < LongitudinalScenario 
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        cbfCheckMargin = 0 % TODO: why isn't a hard threshold working?
    end

    methods
%         function obj = CBFScenarios(inputArg1,inputArg2)
%             %UNTITLED2 Construct an instance of this class
%             %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
%         end

        function [] = accelAndBrakeTest(obj)
            %accelAndBrakeTest Follower accelerates to catch up with 
            % leader, and leader performs emergency braking
            finalTime = 150;
            emergencyBrakingTime = 150;

            obj.setStopTime(finalTime);
            obj.vehicleArray = SafeVehicleArray(2);

            names = {'leader', 'follower'};
            % Create vehicles
            q0L = [6, 0]';
            q0F = [0, 0]';
            q = [q0L, q0F];
            desiredVelocity = [30; 30];
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, q, ...
                desiredVelocity, isConnected)
            
            % Run
            leader = obj.vehicleArray.getVehByName(names(1));
            follower = obj.vehicleArray.getVehByName(names(2));
            follower.accelBounds(2) = 2;
            followerMaxAccel = max(follower.accelBounds);

            for k = 1:(length(obj.simTime)-1)
                if (obj.simTime(k) > emergencyBrakingTime ...
                    && leader.velocity > 0)
                    leaderAccel = -leader.maxBrake;
                else
                    leaderAccel = 1;
                end
                obj.vehicleArray.singleStepUpdate( ...
                    [leaderAccel, followerMaxAccel]);
            end
            obj.vehicleArray.plotStatesAllVehs({'gap', 'vx', 'ax'});
            obj.cbfCheck();
        end

        function [] = sinusoidalAccelTest(obj, period)
            %test1 Leader starts far away. After follower catches up,
            %leader brakes as hard as possible
            obj.setStopTime(50);
            
            nV = 2;
            names = cell(nV, 1);
            q0 = zeros(2, nV);
            vx0 = 0;
            for n = 1:nV
                names{n} = ['veh' num2str(n)];
                q0(:, n) = [(nV-n) * (vx0 + 1 + 5); vx0];
            end
            desiredVelocity = 30*ones(nV, 1);
            isConnected = true;
            obj.vehicleArray = SafeVehicleArray(nV);
            obj.vehicleArray.createVehicles(names, obj.simTime, q0, ...
                desiredVelocity, isConnected)
            
            % Run
            amplitude = 3;
            for k = 1:(length(obj.simTime)-1)
                leaderAccel = amplitude*sin(obj.simTime(k)*2*pi/period);
                obj.vehicleArray.singleStepUpdate(leaderAccel);
            end
            obj.vehicleArray.plotStates([1 nV], {'gap', 'vx', 'ax'});
            obj.cbfCheck();
        end

        function [] = gapIncrease(obj)
            finalTime = 100;
            gapGenerationTime = 5;
            obj.setStopTime(finalTime);

            nV = 3;
            names = {'fo', 'ego', 'lo'};
            q0 = zeros(2, nV);
            vx0 = 20;
            for n = 1:nV
                q0(:, n) = [(nV-n) * (vx0 + 1 + 5); vx0];
            end
            desiredVelocity = 30*ones(nV, 1);
            obj.vehicleArray = SafeVehicleArray(nV);
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, q0, ...
                desiredVelocity, isConnected)

            % For nicer plots
            for n = 1:nV
                obj.vehicleArray.vehs(n).accelBounds(2) = 2;
                obj.vehicleArray.vehs(n).comfAccelBounds = [-2; 2];
            end
            
            % Run
            ego = obj.vehicleArray.getVehByName('ego');
            simulationGapCreationTime = -1;
            maneuverStartIdx = -1;
            for k = 1:(length(obj.simTime)-1)
                if abs(obj.simTime(k) - gapGenerationTime) ...
                        < obj.samplingPeriod/2
                    maneuverStartIdx = k;
                    ego.hasLaneChangeIntention = true;
                end
                if abs(obj.simTime(k) - 50) < obj.samplingPeriod/2
                    ego.hasLaneChangeIntention = false;
                end

                obj.vehicleArray.singleStepUpdate();

                if (ego.cbfValues(k, 3) >= 0 ...
                        && simulationGapCreationTime < 0)
                    simulationGapCreationTime = obj.simTime(k) ...
                        - gapGenerationTime;
                end
            end
            obj.vehicleArray.plotStatesAllVehs({'gap', 'vx', 'ax'});
%             obj.cbfCheck();

            rho = ego.laneChangeDistCBFparameter(1);
            gamma = ego.laneChangeDistCBFparameter(2);
            theoryGapCreationTime = ...
                abs(ego.cbfValues(maneuverStartIdx, 3))^(1-rho)...
                / gamma / (1-rho);
            fprintf('Simulation T: %.1f\nTheory T: %.1f\n', ...
                simulationGapCreationTime, theoryGapCreationTime)
        end

        function [] = cbfCheck(obj)
            for n = 1:length(obj.vehicleArray.vehs)
                
                check = obj.vehicleArray.vehs(n).cbfValues ...
                    < obj.cbfCheckMargin;
                for k = 1:size(check, 2)
                    if any(check(:, k))
                        nonZeroIdx = find(check(:, k));
                        fprintf(['Veh %d CBF %d below %.2f from ' ...
                            '%.1fs to %.1fs\n'], n, k, obj.cbfCheckMargin, ...
                            obj.simTime(nonZeroIdx(1)), ...
                            obj.simTime(nonZeroIdx(end)));
                        figure; hold on; grid on;
                        plot(obj.simTime, ...
                            obj.vehicleArray.vehs(n).cbfValues(:, k))
                        title(sprintf('veh %d, cbf %d', n, k))
                    end
                end
            end
        end

    end
end