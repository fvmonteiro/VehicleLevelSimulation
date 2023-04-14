%%%%% TESTING FUNCTIONS %%%

%%%%% TODO: change order in conditions vectors: make f, l, m to simplify
%%%%% the case choice.

clearvars;
close all;
%% Initialization
% Parameters
ll = 5; % length of leading vehicle
weights = [0.5 0.5 0.5]; % optimization jerk weights
g_min = 10; % minimum gap
omega = 0.2; % fraction of the minimum gap between leading and main vehicles
tf = 3.5; % maneuver time
sampling_period = .001;
t = 0:sampling_period:tf;
vmax = 30; % maximum legal speed (m/s)
legends = {'Fd', 'Ld', 'M'};

% Leading vehicle initial conditions
xl0 = 8; % leading vehicle initial position (m)
vl0 = 20; % leading vehicle initial (and constant) speed (m/s)
al0 = 0; % leading vehicle initial acceleration (m/s^2) 
jerkl0 = 0; % leading vehicle initial jerk (m/s^3)
% Main vehicle initial conditions
xm0 = 0; % main vehicle initial position (m)
vm0 = 18; % main vehicle initial speed (m/s)
am0 = 0; % main vehicle initial acceleration (m/s^2)
jerkm0 = 1; % main vehicle initial jerk (m/s^3)
% Following vehicle initial conditions
xf0 = 0; % following vehicle initial position (m)
vf0 = 20; % following vehicle initial speed (m/s)
af0 = 1; % following vehicle initial acceleration (m/s^2)
jerkf0 = -1; % following vehicle initial jerk (m/s^3)


% Gap generation constraints
dlf = g_min + ll - (xl0 - xf0) - (vl0-vf0)*tf; % difference in distance traveled between leading and following
dlm = omega*g_min + ll - (xl0 - xm0) - (vl0-vm0)*tf; % difference in distance traveled between leading and main

% Leading vehicle terminal constraints
vlf = vl0; % desired leading vehicle speed by the end of the maneuver (equal to initial)
alf = 0; % desired leading vehicle acceleration by the end of the maneuver
jerklf = 0; % desired leading vehicle jerk by the end of the maneuver
% Main vehicle terminal constraints
vmf = vlf; % desired leading vehicle speed by the end of the maneuver (equal to initial)
amf = 0; % desired leading vehicle acceleration by the end of the maneuver
jerkmf = 0; % desired leading vehicle jerk by the end of the maneuver
% Following vehicle terminal constraints
vff = vlf; % desired following vehicle speed by the end of the maneuver (equal to initial)
aff = 0; % desired following vehicle acceleration by the end of the maneuver
jerkff = 0; % desired follwing vehicle jerk by the end of the maneuver

%% Choose case
d = [dlf, dlm]; % first two elements are indeed repeated (the final
% distance between L and F is the same regardless of controlling one or 
% both vehicles) - this is a work around for code conciseness ahead
x0 = [xf0, xl0, xm0];
v0 = [vf0, vl0, vm0];
vf = [vff, vlf, vmf];
a0 = [af0, al0, am0];
af = [aff, alf, amf];
jerk0 = [jerkf0, jerkl0, jerkm0];
jerkf = [jerkff, jerklf, jerkmf];

n_vehicles = 3;
legends = legends(1:n_vehicles);
d = d(1:max(1, n_vehicles-1));
x0 = x0(1:n_vehicles);
v0 = v0(1:n_vehicles);
vf = vf(1:n_vehicles);
a0 = a0(1:n_vehicles);
af = af(1:n_vehicles);
jerk0 = jerk0(1:n_vehicles);
jerkf = jerkf(1:n_vehicles);
% %% Case 5: 5th order polynomial
% [at5] = case5_a(t, tf, d, v0, vf, a0, af, jerk0, jerkf);
% [xt5, vt5, jerkt5] = kinematic_curves(at5, x0, v0, sampling_period);
% title_string = 'Case 5';
% plot_kinematics(t, at5, vt5, xt5, ll, title_string, legends);
% 
% cost5 = sum(trapz(sampling_period, jerkt5.^2));
% %% Case 5.1: nth order polynomial with optimization
% np = 5; % polynomial order
% [at5opt] = case5opt_a(t, tf, d, v0, vf, a0, af, jerk0, jerkf, weights, np);
% [xt5opt, vt5opt, jerkt5opt] = kinematic_curves(at5opt, x0, v0, sampling_period);
% title_string = ['Case 5 optimized (order ' num2str(np) ')'];
% plot_kinematics(t, at5opt, vt5opt, xt5opt, ll, title_string, legends);
% 
% cost5opt = sum(trapz(sampling_period, jerkt5opt.^2));
 
%% Case 5.2: nth order polynomial with optimization and constraints
vf_bounds = [0 30];
af_bounds = [-5 5];
jerkf_bounds = [-10 10];
np = 6;

at5constr = case5constr_a(t, tf, d, v0, vf, a0, af, jerk0, jerkf, ...
    vf_bounds, af_bounds, jerkf_bounds, weights, np);
[xt5constr, vt5constr, jerkt5constr] = kinematic_curves(at5constr, x0, v0, sampling_period);
title_string = ['Case 5 constrained (order ' num2str(np) ')'];
plot_kinematics(t, at5constr, vt5constr, xt5constr, ll, title_string, legends);
 
% % checking optimization
 cost5constr = sum(trapz(sampling_period, jerkt5constr.^2));