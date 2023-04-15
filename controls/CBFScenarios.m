classdef CBFScenarios < LongitudinalScenario 
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    methods
%         function obj = CBFScenarios(inputArg1,inputArg2)
%             %UNTITLED2 Construct an instance of this class
%             %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
%         end

        function [] = test1(obj)
            %test1 Leader starts far away. After follower catches up,
            %leader brakes as hard as possible
            obj.setStopTime(200);
            obj.vehicleArray = SafeVehicleArray(2);

            names = {'leader', 'follower'};
            % Create vehicles
            q0L = [50, 20]';
            q0F = [0, 1]';
            q = [q0L, q0F];
            desiredVelocity = [30; 30];
            obj.vehicleArray.createVehicles(names, obj.simTime, q, ...
                desiredVelocity)
            
            % Run
            leader = obj.vehicleArray.getVehByName(names(1));
            follower = obj.vehicleArray.getVehByName(names(2));
            followerAccel = max(follower.accelBounds);
            % how long after the follower has caught up the leader brakes
            waitTimeBeforeBrake = 5;
            catchUpTime = 0;
            for k = 1:(length(obj.simTime)-1)
%                 if follower.safeDistanceCBF() < 0.1
%                     catchUpTime = catchUpTime + obj.samplingPeriod;
%                 end
                if obj.simTime(k) > 80  % TODO: make based on gap
                    leaderAccel = -leader.maxBrake;
                else
                    leaderAccel = 0;
                end
                obj.vehicleArray.singleStepUpdate( ...
                    [leaderAccel, followerAccel]);
            end
            obj.vehicleArray.plotStatesAllVehs({'x', 'vx'});
        end
    end
end