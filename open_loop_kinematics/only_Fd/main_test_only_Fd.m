%%%%%% TESTING FUNCTIONS %%%%%

clearvars;
close all;
%% Parameters
ll = 5; % length of leading vehicle
g_min = 10; % minimum gap
tf = 3.5; % maneuver time
sampling_period = .001;
t = 0:sampling_period:tf;

%% Initial conditions
xf0 = 0; % following vehicle initial position (m)
xl0 = 8; % leading vehicle initial position (m)
vf0 = 25; % following vehicle initial speed (m/s)
vl0 = 25; % leading vehicle initial (and constant) speed (m/s)
af0 = 1; % following vehicle initial acceleration (m/s^2)
al0 = 0; % leading vehicle initial acceleration (m/s^2) 
jerkf0 = -1; % following vehicle initial jerk (m/s^2)

xlt = xl0 + vl0*t; % leading vehicle position under constant speed
xlf = xlt(end); % leading vehicle final position
xff = xlf - ll - g_min; % following vehicle desired final position
s = xff - xf0 - vf0*tf;
%% Case 1: constant acceleration

aft1 = case1_a(t, s, tf);
[xft1, vft1, ~] = kinematic_curves(aft1, xf0, vf0, sampling_period);
gt1 = xlt - xft1 - ll;
% gt1_2 = g0 + v0*tf + cumtrapz(sampling_period, delta_v); %same as line
% above
title_string = 'Case 1';
plot_kinematics(t, aft1, vft1, gt1, title_string);

%% Case 2: constant jerk

jerkf2 = case2_jerk(s, tf, af0);
aft2 = case2_a(t, jerkf2, af0);
[xft2, vft2, ~] = kinematic_curves(aft2, xf0, vf0, sampling_period);
gt2 = xlt - xft2 - ll;

title_string = 'Case 2';
plot_kinematics(t, aft2, vft2, gt2, title_string);

%% Case 3.1: constant jerk up to a given acceleration when final acceleration is given
aff_c = -3;

if abs(aff_c) >= abs(aft2(end))
    fprintf('Given final acceleration for case 3 is more than enough.\nUsing case 2 instead. \n')
    aff_c = aft2(end);    
elseif abs(aff_c)<abs(aft1(1))
    fprintf('Given final acceleration for case 3 is not enough.\nUsing case 2 instead. \n')
    aff_c = aft2(end);
end

jerkf3 = case3_jerk(s, tf, af0, aff_c);
% Only one of the solutions is valid: its magnitude has to be greater or 
% equal than jerkf2 magnitude
jerkf3 = jerkf3(abs(jerkf3)-abs(jerkf2)>=-0.0001);
t1 = (aff_c-af0)/jerkf3;
%t1 = t1(t1<=tf); % only one of the solutions for jerkf3 is valid

aft3 = case3_a(t, jerkf3, af0, aff_c, t1);
[xft3, vft3, ~] = kinematic_curves(aft3, xf0, vf0, sampling_period);
gt3 = xlt - xft3 - ll;
title_string = 'Case 3 given a_f';
plot_kinematics(t, aft3, vft3, gt3, title_string);

%% Case 3.2: constant jerk up to a given acceleration when jerk is given
jerk_c = -3;

if abs(jerk_c)<abs(jerkf2)
    fprintf('Given jerk is not enough.\nUsing jerk from case 2 instead.\n');
    jerk_c = jerkf2;
end

af3 = case3_af(tf, s, af0, jerk_c);
t1 = (af3-af0)/jerk_c;
aft3_2 = case3_a(t, jerk_c, af0, af3, t1);
[xft3_2, vft3_2, ~] = kinematic_curves(aft3_2, xf0, vf0, sampling_period);
gt3_2 = xlt - xft3_2 - ll;
title_string = 'Case 3 given jerk';
plot_kinematics(t, aft3_2, vft3_2, gt3_2, title_string);

%% Case 4: constant jerk up to given acceleration and then constant jerk back to null acceleration 
% Lots of algebra and nothing to add w.r.t. to previous cases.

%% Parameters for following cases
vff = vl0; % desired following vehicle speed by the end of the maneuver (equal to leader speed)
aff = 0; % desired following vehicle acceleration by the end of the maneuver
jerkff = 0; % desired follwing vehicle jerk by the end of the maneuver

%% Case 5: 5th order polynomial
[aft5, alpha5] = case5_a(t, tf, s, vf0, vff, af0, aff, jerkf0, jerkff);
[xft5, vft5, jerk5] = kinematic_curves(aft5, xf0, vf0, jerkf0, sampling_period);
gt5 = xlt - xft5 - ll;
title_string = 'Case 5';
plot_kinematics(t, jerk5, aft5, vft5, gt5, title_string);

cost5 = trapz(sampling_period, jerk5.^2);
%% Case 5.1: nth order polynomial with optimization
np = 10; % polynomial order

[aft5opt, alpha5opt] = case5opt_a(t, tf, s, vf0, vff, af0, aff, jerkf0, jerkff, np);
[xft5opt, vft5opt, jerk5opt] = kinematic_curves(aft5opt, xf0, vf0, jerkf0, sampling_period);
gt5opt = xlt - xft5opt - ll;
title_string = ['Case 5 optimized (order ' num2str(np) ')'];
plot_kinematics(t, jerk5opt, aft5opt, vft5opt, gt5opt, title_string);

cost5opt = trapz(sampling_period, jerk5opt.^2);
%% Case 5.2: nth order polynomial with optimization and constraints
clc
vf_bounds = [0 30];
af_bounds = [-10 4];
jerkf_bounds = [-100 100];
np = 10;

aft5constr = case5constr_a(t, tf, s, vf0, vff, af0, aff, jerkf0, jerkff, vf_bounds, af_bounds, jerkf_bounds, np);
[xft5constr, vft5constr, jerk5constr] = kinematic_curves(aft5constr, xf0, vf0, sampling_period);
gt5constr = xlt - xft5constr - ll;
title_string = ['Case 5 constrained (order ' num2str(np) ')'];
plot_kinematics(t, jerk5constr, aft5constr, vft5constr, gt5constr, title_string);

cost5constr = trapz(sampling_period, jerk5constr.^2);