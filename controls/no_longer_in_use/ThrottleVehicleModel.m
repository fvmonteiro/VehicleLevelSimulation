classdef VehicleModel
    %VehicleModel: vehicle with throttle control
    % TODO: create main vehicle model class and derive from there
    properties
        acc_bounds % acceleration bounds
        x0 % initial state
        veh_length = 5 % vehicle length
    end
    properties (SetAccess = private)
        order = 2;
        n_input = 1;
        A
        B
        C
        D
        alpha = 0.1;
        beta = 1;
    end
        
    methods
        function obj = VehicleModel(x0) % TODO: add parameters alpha and beta?           
            obj.A = [0 1; 0 -obj.alpha];           
            obj.B = [0; obj.beta];
            obj.C = eye(obj.order);
            obj.D = zeros(obj.n_input);
            
            if nargin == 1
                obj.x0 = x0;
            else
                obj.x0 = zeros(obj.order, 1);
            end
        end

        function [xt] = trajectory_no_control(obj, t)
            % Vehicle trajectory with no control
            sys = ss(obj.A, obj.B, obj.C, obj.D);
            [~, ~, xt] = initial(sys, obj.x0, t);
            xt(:, obj.order+1) = zeros(length(t), 1);
        end
        
        function [xt] = trajectory_cl(obj, K, xr0, t)
            % Vehicle trajectory given a state feedback gain K
            Ac = obj.A-obj.B*K;
            
            Bc = zeros(obj.order,1);
            Bc(end) = 1; % not so sure
            
            sys_cl = ss(Ac, Bc, obj.C, obj.D);
            
            e0 = obj.x0 - xr0;
            [~, ~, et] = lsim(sys_cl, zeros(length(t),1), t, e0);
%             figure; plot(t, et); grid; title('Errors');
            % Note: there's probably a better way to obtain the original
            % state...
            xt = zeros(length(t), obj.order+1);
            u = -K*et';
            xt(:, end) = u;
            for k = size(xt, 2)-1:-1:1
                xt(:,k) = obj.x0(k) + cumtrapz(t, xt(:, k+1));
            end
        end
        
        function [xt] = trajectory_ol(obj, u, t)
            % Vehicle trajectory given an open loop control u
            sys_ol = ss(obj.A, obj.B, obj.C, obj.D);
            [~, ~, xt] = lsim(sys_ol, u, t, obj.x0);
            xt(:, obj.order+1) = u;
        end
        
        function obj = set.acc_bounds(obj, bounds)
            if length(bounds)~=2
                error('Bounds array must contain exactly 2 elements.')
            end
            if bounds(1)>bounds(2)
                error('First element (lower bound) must be smaller than second element (upper bound)');
            end
            obj.acc_bounds = bounds;
        end
        
        function x0 = get.x0(obj)
            x0 = obj.x0(1:obj.order);
        end
        
       
    end

end
