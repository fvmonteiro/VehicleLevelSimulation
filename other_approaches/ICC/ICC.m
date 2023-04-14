% ICC.m initializes the ICC (intelligent cruise control) simulation and 
% opens the corresponding simulink file. 
%
% The vehicle following controller is designed using the MRAC 
% (model reference adaptive control) method.

%
% Jianlong Zhang
% Department of Electrical Engineering-Systems
% University of Southern California
% June 09, 2005
%


clearvars
close all

disp(' ');
disp('****************************************************************!');
disp(' ');
disp('Initializing the ICC simulation ...');
disp(' ');
disp('****************************************************************!');
disp(' ');


% initial speed and corresponding throttle angle
v0=0;
thrt_init=0;

% spacing parameters
s0=4.5;
h=1.0;

% control parameters
am=1.0;
k=0.4;
lambda=0.1;

k1_0=6.0;
k1_max=14.0;
k1_min=4.0;

k2_0=2.0;
k2_max=4.0;
k2_min=0.5;

k3_0=0.0;
k3_max=30.0;
k3_min=-30.0;

gamma1=0.8;
gamma2=0.3;
gamma3=0.3;

e_min=-10;
e_max=5;

open ICC_MRAC.mdl;      

disp(' ');
disp('You can start the simulation now.');
disp(' ');