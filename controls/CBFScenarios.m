classdef CBFScenarios < LongitudinalScenario 
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        cbfCheckMargin = -0.1 % due to discretization errors
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
            emergencyBrakingTime = 50;

            obj.setStopTime(finalTime);
            obj.vehicleArray = BicycleVehicleArray(2);

            names = {'leader', 'follower'};
            % Create vehicles

            q0L = [6, 0, 0, 0]';
            q0F = [0, 0, 0, 0]';
            q = [q0L, q0F];
            desiredVelocity = [30; 30];
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, q, ...
                desiredVelocity, isConnected)
            
            % Run
            leader = obj.vehicleArray.getVehByName(names(1));
            follower = obj.vehicleArray.getVehByName(names(2));
            follower.accelBounds(2) = 2;

            for k = 1:(length(obj.simTime)-1)
                if (obj.simTime(k) > emergencyBrakingTime)
                    if leader.velocity > 0
                        leaderAccel = -leader.maxBrake;
                    else
                        leaderAccel = 0;
                    end
                else
                    leaderAccel = 1;
                end
                obj.vehicleArray.singleStepUpdate( ...
                    [leaderAccel, 0]);
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
            q0 = zeros(4, nV);
            y0 = 0;
            theta0 = 0;
            vx0 = 0;
            for n = 1:nV
                names{n} = ['veh' num2str(n)];
                q0(:, n) = [(nV-n) * (vx0 + 1 + 5); y0; theta0; vx0];
            end
            desiredVelocity = 30*ones(nV, 1);
            isConnected = true;
            obj.vehicleArray = BicycleVehicleArray(nV);
            obj.vehicleArray.createVehicles(names, obj.simTime, q0, ...
                desiredVelocity, isConnected)
            
            % Run
            amplitude = 3;
            for k = 1:(length(obj.simTime)-1)
                leaderAccel = amplitude*sin(obj.simTime(k)*2*pi/period);
                obj.vehicleArray.singleStepUpdate([leaderAccel, 0]);
            end
            obj.vehicleArray.plotStates([1 nV], {'gap', 'vx', 'ax'});
            obj.cbfCheck();
        end

        function [] = gapIncrease(obj)
            finalTime = 100;
            gapGenerationTime = 5;
            obj.setStopTime(finalTime);

            nV = 3;
            names = {'lo', 'ego', 'fo'};
            q0 = zeros(4, nV);
            y0 = 0;
            theta0 = 0;
            vx0 = 30;
            for n = 1:nV
                q0(:, n) = [(nV-n) * (vx0 + 1 + 5); y0; theta0; vx0];
            end
            desiredVelocity = 30*ones(nV, 1);
            obj.vehicleArray = BicycleVehicleArray(nV);
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, q0, ...
                desiredVelocity, isConnected)

            % For nicer plots
            for n = 1:nV
                obj.vehicleArray.vehs(n).accelBounds(2) = 2;
                obj.vehicleArray.vehs(n).comfAccelBounds = [-2; 2];
                obj.vehicleArray.vehs(n).accelBoundsDuringLC(2) = 1;
            end
            
            % Run
            ego = obj.vehicleArray.getVehByName('ego');
            simulationGapCreationTime = -1;
            for k = 1:(length(obj.simTime)-1)
                leaderInputs = [0; 0];
                % if obj.simTime(k) > gapGenerationTime + 1 ...
                %         && obj.simTime(k) < gapGenerationTime + 2
                %     leaderInputs = [-5; 0];
                % end

                if abs(obj.simTime(k) - gapGenerationTime) ...
                        < obj.samplingPeriod/2
                    ego.setLaneChangeDirection(1);
                end

                obj.vehicleArray.singleStepUpdate(leaderInputs);
                
                if (ego.maneuverState == VehicleStates.laneChanging ...
                        && simulationGapCreationTime < 0)
                    simulationGapCreationTime = obj.simTime(k) ...
                        - gapGenerationTime;
                end
            end
            obj.vehicleArray.plotStatesAllVehs({'gap', 'vx', 'ax'});
            obj.vehicleArray.plotStatesAllVehs({'y', 'theta', 'delta'});
            fprintf('Simulation T: %.2f\n', simulationGapCreationTime)
        end

        function [] = lateralTest(obj)
            finalTime = 20;
            obj.setStopTime(finalTime);

            nV = 1;
            names = {'ego'};
            q0 = zeros(4, nV);
            y0 = 0;
            theta0 = 0;
            vx0 = 20;
            for n = 1:nV
                q0(:, n) = [(nV-n) * (vx0 + 1 + 5); y0; theta0; vx0];
            end
            desiredVelocity = 30*ones(nV, 1);
            obj.vehicleArray = BicycleVehicleArray(nV);
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, q0, ...
                desiredVelocity, isConnected)

            % For nicer plots
            for n = 1:nV
                obj.vehicleArray.vehs(n).accelBounds(2) = 2;
                obj.vehicleArray.vehs(n).comfAccelBounds = [-2; 2];
            end

            leaderAccel = 0;
            leaderBeta = 1*2*pi/180;
            for k = 1:(length(obj.simTime)-1)
                obj.vehicleArray.singleStepUpdate([leaderAccel, leaderBeta]);
            end
            ego = obj.vehicleArray.getVehByName('ego');
            figure; plot(ego.x, ego.y)
%             obj.vehicleArray.plotStatesAllVehs({'gap', 'vx', 'ax'});
            obj.vehicleArray.plotStatesAllVehs({'theta'});
        end

        function [] = cbfCheck(obj)
            for n = 1:length(obj.vehicleArray.vehs)
                check = obj.vehicleArray.vehs(n).controller.cbfValuesLog ...
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
                            obj.vehicleArray.vehs(n).controller.cbfValuesLog(:, k))
                        title(sprintf('veh %d, cbf %d', n, k))
                    end
                end
            end
        end

    end
end