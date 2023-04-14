classdef AccTrackingModel
    %TODO: describe this class model
    %TODO: redo the math... results not looking good.
    properties %(SetAccess = private)
        A
        B
        alpha = 0.1; % according to paper 0<alpha<=0.2
        beta = 1; % according to paper beta>0
        C = eye(2);
        D = 0;
        x0
    end
    
    methods
        function obj = AccTrackingModel(x0, alpha_beta)
            
            disp('Class code is not proper. Don''t use for now.')
            
            if length(x0) > 2
                error('This system only has two states')
            end
            obj.x0 = x0;
            
            if nargin > 1
                alpha = alpha_beta(1);
                beta = alpha_beta(2);
                if alpha < 0
                    error('Choose alpha > 0. Current value: %f\n', alpha)
                elseif alpha >0.2
                    warning('Recommended range for alpha is (0,0.2]')
                end
                if beta< 0
                    error('Choose beta > 0. Current value: %f\n', beta)
                end
                obj.alpha = alpha;
                obj.beta = beta;
            end
            obj.A = [-obj.alpha 0; 0 0];
            obj.B = [obj.beta; 1];
        end
        
        function [xt] = trajectory_cl(obj, K, u, t)
            % Vehicle trajectory given a state feedback gain K
            Ac = obj.A-obj.B*K;
            
            Bc = [1;0];
            
            sys_cl = ss(Ac, Bc, obj.C, obj.D);
            
%             e0 = obj.x0 - xr0;
            [~, ~, xt] = lsim(sys_cl, u, t, obj.x0);
            figure; plot(t, xt); grid;
            hold on; plot(t, u);
            legend('x1 (acc)', 'x2 (theta dot)', 'reference')
        end
        
    end
end