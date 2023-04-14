% Tests

clearvars;
close all;

sampling_interval = 0.001;
%% Modeling Driver Driver Support and Cooperative Systems with Dynamic Optimal Control

% T = 10;

% t = 0:sampling_interval:T;
% % betas = [0.1 1];
% % betas = [0.1 100];
% betas = [1 1];
% s_ref = 75;
% xf0 = [0 20];
% xl0 = [100 20];
% ul = -3*(t>=1 & t<3) + 2*(t>=5 & t<7);
% 
% uf = opt_acceleration_v1(t, xl0-xf0, s_ref, ul, betas);
% 
% 
% xf = veh_dynamics(xf0, uf, t);
% xl = veh_dynamics(xl0, ul, t);
% % X = zeros(size(xf,1), size(xf,2), 2);
% X(:,:,1) = [xf; uf];
% X(:,:,2) = [xl; ul];
% plot_kinematics(t, X, 'ACC', {'Follower', 'Leader'});

%% Game theoretic approach for predictive lane-changing and car-following control

% load('sim_parameters.mat');
% 
% T = 1;
% dend = 2000;
% xf0 = [0 25];
% xl0 = [50+l 25];
% 
% 
% v_lanei = 25;
% % % a = 0;
% total_lanes = 2;
% lane_number = 2;
% lane_switch = 0;
% 
% %%%%%%%%%%%%%
% % Single step
% t = 0:sampling_interval:T;
% 
% % Compute optimal acceleration
% acc = opt_acceleration(t, td, s0, d0, dend, xf0, xl0, l, vd, betas);
% 
% % Compute vehicle dynamics with optimal control
% xf = veh_dynamics(xf0, acc, t);
% xl = veh_dynamics(xl0, zeros(size(acc)), t);
% 
% % Compute costs
% % gap = xl(1,:)-xf(1,:)-l;
% [L, individual_costs] = running_cost(betas, td, s0, d0, dend, xf, xl, vd, l, v_lanei, ...
%     acc, total_lanes, lane_number, lane_switch);
% individual_costs_single_step = trapz(t, individual_costs');
% total_cost_single_step = trapz(t, L);


%% Using new classes
test = TestScenarios();
[cost, individual_cost] = test.solve_scenario1();