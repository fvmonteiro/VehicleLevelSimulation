classdef VehicleDataReader
    %VehicleDataReader Reads data generated from simulations
    
    properties
        resultsFolder = 'numerical_results';
    end
    
    methods
        function [origLane, destLane] = loadVehiclesFromFile(obj, simName,...
                scenario, relVel, hasTruck, lcStrategy)
            % loadVehiclesFromFile Loads VehicleArray objects. Must run
            % this function before plotting any results.
            % @simName: 'full_maneuver' or 'gap_generation'
            % @scenario: 'single_veh' or 'platoon'
            % @relVel: -5, 5, 0 for 'full_maneuver'; -4, 4, 0 for 'gap_generation'
            % @lcStrategy: only used with 'platoon' option. Check
            % SimulinkVehicle.possibleStrategies for options
            
            if hasTruck
                truckString = 'truckLC_';
            else
                truckString = '';
            end
            if strcmp(scenario, 'platoon')
                strategyString = ['_strategy_' num2str(lcStrategy)];
            else
                strategyString = '';
            end
            simulationFolder = sprintf(...
                '%s/%s/%s/', obj.resultsFolder, simName, scenario);
            simulationName = sprintf('%srel_vel_%d%s_results.mat', ...
                truckString, relVel, strategyString);
            
            dataStruct = load([simulationFolder simulationName]);
            origLane = dataStruct.origLane;
            destLane = dataStruct.destLane;
        end
        
        function [allVehicles] = loadVideoVehicles(obj, scenario, relVel, ...
                hasTruck, lcStrategy)
            %loadVideoVehicles Loads vehicle objects that contain only data
            %obtained from simulation
            % @scenario: 'single_veh' or 'platoon'
            % @relVel: -5, 5, 0
            % @lcStrategy: synchronous, leaderFirst, lastFirst,
            % leaderFirstInvert
            
            if hasTruck
                truckString = 'truckLC_';
            else
                truckString = '';
            end
            if strcmp(scenario, 'platoon')
                strategyString = ['_strategy_' lcStrategy];
            else
                strategyString = '';
            end
            simulationFolder = sprintf(...
                '%s/video_vehicles/%s/', obj.resultsFolder, scenario);
            simulationName = sprintf('%srel_vel_%d%s_results.mat', ...
                truckString, relVel, strategyString);
            dataStruct = load([simulationFolder simulationName]);
            allVehicles = dataStruct.allVehicles;
        end
    end
    
    methods (Static)
        function [X, Y, Psi, lcState] = getTrajectories(allVehs)
            %getTrajectories Returns x, y, psi and maneuver states of all 
            % vehicles as cells of timeseries
            
            X = cell(1, length(allVehs));
            Y = cell(1, length(allVehs));
            Psi = cell(1, length(allVehs));
            lcState = cell(1, length(allVehs));
            
            t = allVehs(1).simTime;
            for nV = 1:length(allVehs)
                veh = allVehs(nV);
                X{nV} = timeseries(veh.x, t);
                Y{nV} = timeseries(veh.y, t);
                Psi{nV} = timeseries(veh.psi, t);
                lcState{nV} = veh.lcState;
                if isempty(lcState)
                    lcState{nV} = timeseries(0, t);
                end
            end
        end
    end
end