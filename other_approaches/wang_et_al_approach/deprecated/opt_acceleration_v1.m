function [u] = opt_acceleration_v1(t, x0, s_ref, ul, betas)
%opt_acceleration_v1 Solving the opt controller for the 2012 paper
%   Detailed explanation goes here

% System:
% x = [s; v], 
% where s and v are the differences in position and speed between vehicles
% u = acc_leader - acc_ego_vehicle - INCORRECT
% dx/dt = Ax + Bu; A = [0 1;0 0], B = [0;1].

% Cost
% L = 1/2 u^2 + beta1/2 (x1-s*)^2 + beta2/2 x2^2
% H = L + lambda*(Ax+Bu)
% Optimal control
% u = lambda(2)

% Costate equations:
% dlambda1/dt = -beta1(x1-s*)
% dlambda2/dt = -beta2*x2-lambda1
% lambda(T) = [0;0]

% Parameters (not given in paper...)
tolerance = 0.01; % total guess
alpha = 0.0001; % 0<alpha<1 - has to change according to problem
iter_limit = 10000;


% tspan = [0 T];

beta1 = betas(1);
beta2 = betas(2);
lambdaT = [0;0];

Lambda = [zeros(1, length(t)); zeros(1, length(t))];
lambda = Lambda; %just to enter the while loop; these values are discarded
x = [zeros(1, length(t)); zeros(1, length(t))];
k=1;
error = [tolerance+1 zeros(1, iter_limit-1)];
% Stopping criteria not well specified in paper...
while (error(k) > tolerance) && (k<iter_limit)
    % Optimal controller
    u = Lambda(2, :);
        
    % Test 1: analytical plus numerical integration
    % Solve the state dynamic equation forward
    x(2, :) = x0(2) + cumtrapz(t, ul-u);
    x(1, :) = x0(1) + cumtrapz(t, x(2,:));
    
    % Solve the costate dynamic equation backward
    % first we do it "dum" and right; then improve algebra
    term1 = beta1*(x(1,:) - s_ref);
    lambda(1, :) = lambdaT(1) + trapz(t, term1) - cumtrapz(t,term1);
    term2 = beta2*x(2,:) + lambda(1,:);
    lambda(2, :) = lambdaT(2) + trapz(t, term2) - cumtrapz(t, term2);
    
    % Test 2: using ode45

    % Update Lambda
    Lambda = (1-alpha)*Lambda + alpha*lambda;

    
    % Update iteration index
    k = k + 1;
%     error(k) = sum((Lambda(:) - lambda(:)).^2);
    error(k) = max((Lambda(:) - lambda(:)).^2);
end

disp(k)
disp(error(k))

% shift = 15;
% figure; plot(shift:iter_limit, error(shift:end));
figure; 
plot(t, x(1,:)); hold on;
plot(t, lambda(1,:));
yyaxis right
ylim([min(0, 1.1*min(lambda(1,:))) 1.1*max(lambda(1, :))])
legend('s', 'co-state \lambda_1');

figure;
plot(t, x(2,:)); hold on;
plot(t, lambda(2,:));
yyaxis right
ylim([min(0, 1.1*min(lambda(2,:))) 1.1*max(lambda(2, :))])
legend('\Delta_v', 'co-state \lambda_2');

end

