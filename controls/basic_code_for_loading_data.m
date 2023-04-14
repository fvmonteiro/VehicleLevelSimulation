%% Basic start

% States during simulation:
% 0 - Lane keeping
% 1 - Longitudinal Adjustment
% 2 - Steady State
% 3 - Safe to start LC
% 4 - Performing LC
% 5 - LC done
% 6 - Back to original gap

clearvars;

% Create instance of the class with plotting and animation methods
reader = VehicleDataReader();

% Load from file: check method code for details on parameter options
relVel = 5; % options so far: -5, 0, 5
allVehicles = reader.loadVideoVehicles('single_veh', relVel);

% Load variables from previously run simulations to the workspace
[X, Y, Psi, lcState] = VehicleDataReader.getTrajectories(allVehicles);

finalTime = X{1}.Time(end);
newTime = 0:0.02:finalTime;

for i = 1:length(X)
    
    currentVehicleX = resample(X{i}, newTime);
    currentVehicleY = resample(Y{i}, newTime);
    currentVehiclePsi = resample(Psi{i}, newTime);
    currentVehicleStates = resample(lcState{i}, newTime);
    % write to some file (remember to use .Data to get data)
    
end