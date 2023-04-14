function [u] = opt_acceleration(t, td, s0, d0, dend, xf0, xl0, ll, vd, betas)
%opt_acceleration Solving the opt controller for the 2015 paper
%   Detailed explanation goes here

% System:
% x = [s; v], 
% where s and v are the position and speed of the vehicles
% u = acc
% dx/dt = Ax + Bu; A = [0 1;0 0], B = [0;1].

% Cost
% L = too longe - check paper
% H = L + lambda*(Ax+Bu)
% Optimal control
% u = -lambda(2)/(2.beta_ctr) - note lambda of the respective v_i

% Costate equations:

% Parameters (not given in 2012 paper)
tolerance = 0.01; % total guess
alpha = 0.01; % 0<alpha<1 - has to change according to problem
iter_limit = 10000;

% Other parameters
dr = 1000; % 1km

% Cost weights
b_safe = betas(1);
b_eq = betas(2);
b_cntr = betas(3);
b_route = betas(5);

% Leader kinematics;
vl = xl0(2);
sl = xl0(1) + t*xl0(2);

lambdaT = [0;0];

Lambda = [zeros(1, length(t)); zeros(1, length(t))];
lambda = Lambda; %just to enter the while loop; these values are discarded
x = [zeros(1, length(t)); zeros(1, length(t))];
k=1;
error = [tolerance+1 zeros(1, iter_limit-1)];
% Stopping criteria not well specified in paper...
while (error(k) > tolerance) && (k<iter_limit)
    % Optimal controller
    u = -Lambda(2, :)/(2*b_cntr);
        
    % Test 1: analytical plus numerical integration
    % Solve the state dynamic equation forward
    x(2, :) = xf0(2) + cumtrapz(t, u);
    x(1, :) = xf0(1) + cumtrapz(t, x(2,:));
    
    % Solve the costate dynamic equation backward
    % first we do it "dum" and right; then improve algebra
    delta_v = vl - x(2,:); 
    gap = sl - x(1,:)-ll;
    
    safety = b_safe./(gap.^2) .* delta_v.^2 .* theta(-delta_v);
    eq = 2*b_eq*(v_eq(gap) - x(2,:))/td;
    route = b_route*d0/(dend^2)*exp(d0/dend)*(dend<dr);
    dHds = route + safety - eq;
    lambda(1, :) = lambdaT(1) + trapz(t, dHds) - cumtrapz(t,dHds);
    
    
    dHdv = -2*b_safe./gap.*delta_v.*theta(-delta_v) + 2*b_eq*(x(2, :)-v_eq(gap)) + lambda(1,:);
    lambda(2, :) = lambdaT(2) + trapz(t, dHdv) - cumtrapz(t, dHdv);
    
    % Test 2: using ode45

    % Update Lambda
    Lambda = (1-alpha)*Lambda + alpha*lambda;

    
    % Update iteration index
    k = k + 1;
%     error(k) = sum((Lambda(:) - lambda(:)).^2);
    error(k) = max((Lambda(:) - lambda(:)).^2);
end

% disp(k)
% disp(error(k))

figure;
plot(t, x(1,:)); hold on;
yyaxis right
plot(t, lambda(1,:));
ylim([min(0, 1.1*min(lambda(1,:))) 1.1*max(lambda(1, :))])
legend('s_f', 'co-state \lambda_1');

figure;
plot(t, x(2,:)); hold on;
yyaxis right
plot(t, lambda(2,:));
ylim([min(0, 1.1*min(lambda(2,:))) 1.1*max(lambda(2, :))])
legend('v_f', 'co-state \lambda_2');

% Function theta is not defined in the paper
% My assumption is: output 1 if the vehicle is approaching its leader
    function [b] = theta(arg) 
        b = arg>=0; 
    end

% Local equilibrium speed
    function [output_v] = v_eq(gap)
        sf = vd*td + s0;
        idx = gap>sf;
        output_v = vd*(idx) + (gap-s0)/td.*(~idx);
%         if gap > sf
%             output_v = vd;
%         else
%             output_v = (gap-s0)/td;
%         end
    end

end

