%% Parameters

clearvars;

vd = 30; %Desired speed in m/s
td = 1.2; %Desired time gap in s
s0 = 2; % Min gap at standstill conditions in m
tlc = 5; % Lane change duration in s
tinter = 2; % Minimum interval between successive lane chanes in s
p = 1; % frequency of lane chanfe decisions in Hz
Tp = 8; % Prediction horizon in s
betas = [2, 0.02, 0.5, 0.1, 1, 1, 0.2]; % costs' weights in the following order:
% safe, eq, , ctrl, eff, route, pref, swit 
% NOTE 1: beta_route is not given in the paper
% NOTE 2: beta_switch is said to be one, but results indicate it's 0.2
l = 4; % Vehicle length in m
d0 = 350; % distance parameter of route cost in m
v_max = 40; % Max vehicle speed in m/s
a_max = 2; % Max acceleration value in m/s^2
a_min = -8;% Min acceleration value in m/s^2

save('sim_parameters')