% Script to obtain transfer functions

%% No derivative of gap error:
% Easier and simpler conditions but without the extra degree of freedom it
% might not be possible to obtain conditions for heterogeneous platoons
clearvars

syms kg kv ka real
syms tau h real
syms s

% u = kg*x1 + kv(vl-x2) + ka(al-x3)

A = [0, -1, -h;...
    0, 0, 1;...
    0, 0,  -1/tau];
Bu = [0; 0; 1/tau];
K = [kg -kv -ka];

Acl = A+Bu*K;

pol_coeff = charpoly(Acl);

% Leader velocity to gap and vel errors
Bvl = [1; 0; kv/tau];
invABal = (s*eye(3)-Acl)\Bvl;
H11 = [1 0 0]*invABal;
H12 = [0 1 0]*invABal;

% Leader accel to gap and vel errors
Bal = [0; 0; ka/tau];
invABal = (s*eye(3)-Acl)\Bal;
H21 = [1 0 0]*invABal;
H22 = [0 1 0]*invABal;

% Initial conditions to gap error
syms eg0 ev0 ea0 real
q0 = [eg0; ev0; ea0];
Hq0 = [1 0 0]*((s*eye(3)-Acl)\q0);


%% with derivative of the gap error

clearvars

syms kg kd kv ka real
syms tau h real
syms s

% u = kg*x1 + (kv+kd)(vl-x2) + ka(al-x3) - kd.h.x3

A = [0, -1, -h;...
    0, 0, 1;...
    0, 0,  -1/tau];
Bu = [0; 0; 1/tau];
K = [kg -kv-kd -ka-h*kd];

Acl = A+Bu*K;

pol_coeff = charpoly(Acl);

% Leader velocity to gap error, vel and accel
Bvl = [1; 0; (kv+kd)/tau];
invABal = (s*eye(3)-Acl)\Bvl;
H11 = [1 0 0]*invABal; % eg/vl
H12 = [0 1 0]*invABal; % vE/vl
H13 = [0 0 1]*invABal; % aE/vl

% Leader accel to gap error, vel and accel
Bal = [0; 0; ka/tau];
invABal = (s*eye(3)-Acl)\Bal;
H21 = [1 0 0]*invABal; % eg/al
H22 = [0 1 0]*invABal; % vE/al
H23 = [0 0 1]*invABal; % aE/al

% Initial conditions to gap error
syms eg0 ev0 ea0 real
q0 = [eg0; ev0; ea0];
Hq0 = [1 0 0]*((s*eye(3)-Acl)\q0);

%% with filtered velocity 1
clearvars

syms kg kd kv ka real
syms tau h p real
syms s

% u = kg*x1 + (kv+kd)(vl-x2) + ka(al-x3) - kd.h.x3

A = [0, -1, -h, 1; ...%0, -1, -h, 0;...
    0, 0, 1, 0;...
    0, 0,  -1/tau, 0; ...
    0, 0, 0, -p];
Bu = [0; 0; 1/tau; 0];
% K = [kg -kv-kd -ka-kd*h kv-ka*p];
K = [kg, -kv-kd, -ka-kd*h, kd+kv-ka*p];

Acl = A+Bu*K;

pol_coeff = charpoly(Acl);

% Leader velocity to gap error, vel and accel
% Bvl = [1; 0; (kd+ka*p)/tau; p];
Bvl = [0; 0; ka*p/tau; p];
invABal = (s*eye(4)-Acl)\Bvl;
H11 = [1 0 0 0]*invABal; % eg/vl
H12 = [0 1 0 0]*invABal; % vE/vl


%% with filtered velocity 2
% eg = g - hvE
% ev = vL_hat - vE
% ea = aL - aE
% ef = vL - vL_hat
% u = kg.eg + kd(ev - h.a_E) + kv.ev + ka.ea
% u = kg.x1 + (kd + kv).x2 + (kd.h + ka).x3 - kd.h.aL
clearvars

syms kg kd kv ka real
syms tau h p real
syms s

A = [0, 1, h, 1; ...%0, -1, -h, 0;...
    0, 0, 1, p;...
    0, 0,  -1/tau, 0; ...
    0, 0, 0, -p];
Bu = [0; 0; -1/tau; 0];
K = [kg, kv+kd, ka+kd*h, 0];

Acl = A+Bu*K;

pol_coeff = charpoly(Acl);

% Leader velocity to gap error, vel and accel
Bal = [-h; -1; 1+kd*h; 1];
Bal_dot = [0; 0; 1; 0];
invABal = (s*eye(4)-Acl)\Bal;
H11 = [1 0 0 0]*invABal; % eg/al
H12 = [0 1 0 0]*invABal; % ev/al
invABal_dot = (s*eye(4)-Acl)\Bal_dot;
H21 = [1 0 0 0]*invABal_dot; % eg/al_dot
H22 = [0 1 0 0]*invABal_dot; % ev/al_dot

%% with filtered velocity 3
% eg = g - hvE
% ev = vL_hat - vE
% ea = dot(ev)
% ef = vL - vL_hat
% u = kg.eg + kd.dot(eg) + kv.ev + ka.ea
% u = kg.x1 + (kd + kv).x2 + (kd.h + ka).x3 - [p^2 + kd(1-hp)]x4
clearvars

syms kg kd kv ka real
syms tau h p real
syms w real
syms s

A = [0, 1, h, 1-h*p; ...%0, -1, -h, 0;...
    0, 0, 1, 0;...
    0, 0,  -1/tau, -p^2+p/tau; ...
    0, 0, 0, -p];
Bu = [0; 0; -1/tau; 0];
K = [kg, kv+kd, ka+kd*h, kd*(1-h*p)];

Acl = A+Bu*K;

pol_coeff = charpoly(Acl);

% Leader velocity to gap error, vel and accel
Bal = [0; 0; p; 1];
invABal = (s*eye(4)-Acl)\Bal;
H11 = [1 0 0 0]*invABal; % eg/al
H12 = [0 1 0 0]*invABal; % ev/al

% After checking results from above, we get L(s) = v_E/v_L
num = (p*ka + kd)*s^2 + (kg + p*(kd + kv))*s + p*kg;
den = (p + s)*(tau*s^3 + (ka + h*kd + 1)*s^2 + (kd + kv + h*kg)*s + kg);

num_wj = subs(num, s, 1i*w);
den_wj = subs(den, s, 1i*w);

norm_num = real(num_wj)^2+imag(num_wj)^2;
norm_den = real(den_wj)^2+imag(den_wj)^2;

ineqLHS = collect(norm_den-norm_num, w);
disp(simplify(ineqLHS))
%% String stability conditions: uniform vehicles

clearvars

syms kg kd kv ka real
syms tau h real
syms w real
syms s
num = ka*s^2 + (kv+kd)*s + kg;
den = tau*s^3 + (ka + h*kd + 1)*s^2 + (kd + kv + h*kg)*s + kg;

num_wj = subs(num, s, 1i*w);
den_wj = subs(den, s, 1i*w);

norm_num = real(num_wj)^2+imag(num_wj)^2;
norm_den = real(den_wj)^2+imag(den_wj)^2;

ineqLHS = collect(norm_den-norm_num, w);
disp(simplify(ineqLHS))
%% String stability conditions: heterogeneous vehicles
% We need the same conditions from homogeneous vehicles plus some analysis
% of the gap error to gap TFs
clearvars

syms kg kd kv ka kgl kdl kvl kal real
syms tau h taul hl real
syms w real
syms s

kv = 1/h;
kvl = 1/hl;

num = (tau - ka*h)*(kal*s^2+(kdl+kvl)*s+kgl);
den = (taul - kal*hl)*(ka*s^2+(kd+kv)*s+kg);

num = subs(num, s, 1i*w);
den = subs(den, s, 1i*w);

norm_num = real(num)^2+imag(num)^2;
norm_den = real(den)^2+imag(den)^2;

ineqLHS = collect(norm_den-norm_num, w);
disp(simplify(ineqLHS))

% G = gapTF(s, tau, h, kg, kd, kv, ka);
% Gl = gapTF(s, taul, hl, kgl, kdl, kvl, kal);
% function [G] = gapTF(s, tau, h, kg, kd, kv, ka)
%     num = tau*s^2 + (kd*h+1)*s + kg*h;
%     den = (tau-ka*h)*s^2 + (1-kv*h)*s;
%     G = num/den;
% end