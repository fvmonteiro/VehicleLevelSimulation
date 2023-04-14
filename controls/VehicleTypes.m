classdef VehicleTypes
    % Database class to contain typical parameters for different vehicle 
    % types
    % Possible types are:
    % PV: passenger vehicle
    % HDT: heavy duty vehicles (trucks)
    % Bus: almost the same as HDT. Included here just for compatibility
    % with old code
    
    properties (Constant)
        possibleTypes = {'PV', 'HDV', 'Bus'}
        
        % Dimensions
        m = struct('PV', 2000, 'HDV', 18000, 'Bus', 13000); % [kg]
        len = struct('PV', 5, 'HDV', 18, 'Bus', 13); % [m]
        width = struct('PV', 1.8, 'HDV', 2.4, 'Bus', 2.4); % [m]
        
        % Dynamics
        tau = struct('PV', 0.5, 'HDV', 1, 'Bus', 1); % actuator lag
        
        % Input constraints. 
        % Accelerations in [m/s^2] and jerks in [m/s^3]
        accelBounds = struct('PV', [-8; 3], 'HDV', [-4; 2], 'Bus', [-4, 2])
        comfAccelBounds = struct('PV', [-0.5; 0.5], 'HDV', [-0.5; 0.5],...
            'Bus', [-0.5, 0.5]);
        accelBoundsDuringLC = struct('PV', [-8/2; 0], 'HDV', [-4/2; 0],...
            'Bus', [-0.5, 0.5]);
        jerkBounds = struct('PV', [-50; 50], 'HDV', [-30; 30],...
            'Bus', [-30, 30])
        comfJerkBounds = struct('PV', [-10; 10], 'HDV', [-10; 10],...
            'Bus', [-10, 10]);
        
    end
    
end