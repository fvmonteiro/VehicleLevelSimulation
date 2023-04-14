classdef Controller < handle
    properties
        type
        K
%         openLoopControl
%         openLoopIterCounter
        
        % ACC stuff (used in the Matlab similuation - commented out for
        % now)
        % To avoid high inputs when we start following a new vehicle
%         maxPosError = 10; %[m] - could be dependent on speed
%         minPosError = -100; %[m]
%         accelSaturationFilter
%         accLeaderVelFilter
%         accLeaderPosFilter
        
    end
    
    methods
        
        % Note: before implementing new control types, always check the
        % Controller for Matlab (not Simulink) simulations
        function [] = setGains(obj, controlType, controlParams, timeHeadway)
            obj.type = controlType;

            switch controlType
                case 'ACC PID'
                    poles = controlParams;
                    p1 = poles(1);
                    p2 = poles(2);
                    p3 = poles(3);
                    C = [1 timeHeadway 0; 0 1 timeHeadway; 0 0 1];
                    d = [sum(poles); p1*p2+p1*p3+p2*p3; prod(poles)];
                    A = [-2 -timeHeadway 0];
                    b = -2/timeHeadway;
                    lb = zeros(length(poles), 1);
                    ub = Inf*ones(length(poles), 1);
                    options = optimoptions('lsqlin','Display','off');
                    x = lsqlin(C, d, A, b, [], [], lb, ub, [], options);
                    % x = [Kd; Kp; Ki];
                    obj.K = flip(x');
                case 'ACC PD'
                    poles = controlParams;
                    C = [1 timeHeadway; 0 1];
                    d = [sum(poles); prod(poles)];
                    A = [-2 -veh.tg];
                    b = -2/veh.tg;
                    lb = zeros(length(poles), 1);
                    ub = Inf*ones(length(poles), 1);
                    options = optimoptions('lsqlin','Display','off');
                    x = lsqlin(C, d, A, b, [], [], lb, ub, [], options);
                    % x = [Kd Kp]
                    obj.K = flip(x');
                case 'CACC PID'
                    %%% TODO: these gains do not guarantee string stability 
                    %%% function can still be improved to check for that
                    poles = controlParams(1:3);
                    p1 = poles(1);
                    p2 = poles(2);
                    p3 = poles(3);
                    ka = controlParams(4);
                    C = [1 timeHeadway 0; 0 1 timeHeadway; 0 0 1];
                    d = [sum(poles); p1*p2+p1*p3+p2*p3; prod(poles)];
                    A = [-2 -timeHeadway 0];
                    b = 1 - ka;
                    lb = zeros(length(poles), 1);
                    ub = Inf*ones(length(poles), 1);
                    x = lsqlin(C, d, A, b, [], [], lb, ub);
                    % x = [Kd; Kp; Ki];
                    obj.K = flip(x');
                otherwise
                    error('Unknown controller type')
            end
        end
    end
    
end