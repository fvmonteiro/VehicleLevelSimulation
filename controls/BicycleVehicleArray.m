classdef BicycleVehicleArray < VehicleArray
    %BicycleVehicleArray Class to easily deal with several Bicycle Vehicles

    methods
        function obj = BicycleVehicleArray(nv)
            %SafeVehicleArray Construct an instance of this class
            %   We just create an array of vehicles
            obj.vehs = BicycleVehicleModel.empty;
            obj.vehs(nv) = BicycleVehicleModel();
        end

        function createVehicles(obj, names, simTime, ...
                x0, v0, initialLanes, desiredVelocity, ...
                isConnected)
            if length(desiredVelocity) == 1
                desiredVelocity = desiredVelocity * ones(length(obj.vehs));
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n) = BicycleVehicleModel(names{n}, simTime, ...
                    x0(n), v0(n), initialLanes(n), desiredVelocity(n), ...
                    isConnected);
            end
        end

        function [] = singleStepUpdate(obj, leaderInputs)
            for n = 1:length(obj.vehs)
                obj.vehs(n).findLeader(obj.vehs);
                if nargin > 1 && n == 1
                    obj.vehs(n).computeInput(leaderInputs);
                else
                    obj.vehs(n).computeInput();
                end
            end
            for n = 1:length(obj.vehs)
                obj.vehs(n).updateStates();
            end
        end

        function [] = createAnimation(obj)
            %createAnimation Creates animation with all vehicles
            
            ego = obj.getVehByName('ego');
            
            % Simulation time
            simTime = ego.simTime;
            
            % Precompute X and Y moving areas during video
            minX = ego.x - 25;
            videoWidth = 50; % TODO: make based on initial (final?) vEgo
            videoXLims = [minX, minX + videoWidth];
            videoYLims = [min(ego.y) - ego.width, ...
                max(ego.y) + ego.width*2];
            
            % Lane marks coordinates
            laneLineYCoord = ones(1,2)*(ego.y(end) + ego.width ...
                + ego.y(1))/2;
            laneLineXCoord = [0, videoXLims(end, 2)+15];
            
            % Create figure and subplots, set axes limits, labels etc
            figure();
            
            animationAxis = subplot(1, 1, 1);
            axis equal
            xlabel('x [m]');
            ylim(videoYLims);
            hold on; grid on;

            nSamples = length(ego.simTime);
            skipFrame = 1;
            frames = 1:skipFrame:nSamples;
            movieVector(length(frames)) = struct('cdata',[], ...
                'colormap',[]);
            for k = frames
                % Clear the figure
%                 clf;
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
                for nV = 1:obj.nv
                    veh = obj.vehs(nV);
                    vehX = veh.x(k);
                    vehY = veh.y(k);
                    vehTheta = veh.theta(k)*180/pi;
                    
                    vehImg = patch(animationAxis, ...
                        [vehX vehX+veh.len vehX+veh.len vehX vehX], ...
                        [vehY vehY vehY+veh.width vehY+veh.width vehY], ...
                        veh.getColor());
                    rotate(vehImg, [0 0 1], vehTheta, [vehX vehY 0])
                end
                
                % Take a snapshot
                movieVector(k) = getframe(gcf);
            end
            
        end
        
    end
end