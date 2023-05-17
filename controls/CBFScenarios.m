classdef CBFScenarios < LongitudinalScenario 
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        cbfCheckMargin = -0.1 % due to discretization errors
        debugStopTime = -1
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

            x0 = [6, 0];
            v0 = [0, 0];
            lanes = [0, 0];
            desiredVelocity = [30; 30];
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, x0, v0, ...
                lanes, desiredVelocity, isConnected)
            
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
            x0 = zeros(nV, 1);
            v0 = zeros(nV, 1);
            lanes = zeros(nV, 1);
            for n = 1:nV
                names{n} = ['veh' num2str(n)];
                x0(n) = (nV-n) * (v0(n) + 1 + 5);
            end
            desiredVelocity = 30*ones(nV, 1);
            isConnected = true;
            obj.vehicleArray = BicycleVehicleArray(nV);
            obj.vehicleArray.createVehicles(names, obj.simTime, x0, v0, ...
                lanes, desiredVelocity, isConnected)
            
            % Run
            amplitude = 3;
            for k = 1:(length(obj.simTime)-1)
                leaderAccel = amplitude*sin(obj.simTime(k)*2*pi/period);
                obj.vehicleArray.singleStepUpdate([leaderAccel, 0]);
            end
            obj.vehicleArray.plotStates([1 nV], {'gap', 'vx', 'ax'});
            obj.cbfCheck();
        end

        function [] = laneChange(obj)
            clear Vehicle % reset vehicle ids
            finalTime = 30;
            gapGenerationTime = 1;
            obj.setStopTime(finalTime);

            names = {'lo', 'ego', 'fo', 'vd1', 'vd2', 'vd3', 'vd4'};
            leaderSpeed = 15;
            otherSpeed = 20;

            nV = length(names);
            nVOrigLane = 3;
            % nVDestLane = nV - nVOrigLane;
            x0 = zeros(nV, 1);
            v0 = leaderSpeed*ones(nV, 1);
            lanes = zeros(nV, 1);
            for n = 1:nVOrigLane
                x0(n) = (nVOrigLane - n) * (leaderSpeed + 1 + 5);
            end
            for n = nVOrigLane+1:nV
                x0(n) = (nV - n - 2) * (leaderSpeed + 1 + 5);
                lanes(n) = 1;
            end
            
            desiredVelocity = otherSpeed * ones(nV, 1);
            desiredVelocity(1) = leaderSpeed;
            desiredVelocity(nVOrigLane + 1) = leaderSpeed;

            obj.vehicleArray = BicycleVehicleArray(nV);
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, x0, v0, ...
                lanes, desiredVelocity, isConnected)

            % For nicer plots
            for n = 1:nV
                obj.vehicleArray.vehs(n).accelBounds(2) = 2;
                obj.vehicleArray.vehs(n).comfAccelBounds = [-2; 2];
                obj.vehicleArray.vehs(n).accelBoundsDuringLC(2) = 1;
            end
            
            % Run
            ego = obj.vehicleArray.getVehByName('ego');
            simulationGapCreationTime = -1;
            for k = 1:(length(obj.simTime))
                
                if abs(obj.simTime(k) - gapGenerationTime) ...
                        < obj.samplingPeriod/2
                    ego.setLaneChangeDirection(1);
                end

                leaderInputs = [0; 0];
                obj.vehicleArray.singleStepUpdate(leaderInputs);
                
                if (ego.maneuverState == VehicleStates.laneChanging ...
                        && simulationGapCreationTime < 0)
                    simulationGapCreationTime = obj.simTime(k);
                end
            end
            fprintf('Simulation T: %.2f\n', simulationGapCreationTime)
%             obj.vehicleArray.plotStatesAllVehs({'gap', 'vx', 'ax'});
%             obj.vehicleArray.plotStatesAllVehs({'y', 'theta', 'delta'});
            obj.vehicleArray.createAnimation();
%             figure; hold on; grid on; plot(ego.simTime, ego.yRefLog-ego.y);
        end

        function [] = lateralTest(obj)
            finalTime = 15;
            lcStartTime = 5;
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
            desiredVelocity = vx0*ones(nV, 1);
            obj.vehicleArray = BicycleVehicleArray(nV);
            isConnected = true;
            obj.vehicleArray.createVehicles(names, obj.simTime, q0, ...
                desiredVelocity, isConnected)

            % For nicer plots
            for n = 1:nV
                obj.vehicleArray.vehs(n).accelBounds(2) = 2;
                obj.vehicleArray.vehs(n).comfAccelBounds = [-2; 2];
            end

            ego = obj.vehicleArray.getVehByName('ego');
            for k = 1:(length(obj.simTime)-1)
                if abs(obj.simTime(k) - lcStartTime) ...
                        < obj.samplingPeriod/2
                    ego.setLaneChangeDirection(1);
                end
                obj.vehicleArray.singleStepUpdate();
            end
            
%             figure; plot(ego.x, ego.y)
            obj.vehicleArray.plotStatesAllVehs({'y', 'theta', 'delta'});
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