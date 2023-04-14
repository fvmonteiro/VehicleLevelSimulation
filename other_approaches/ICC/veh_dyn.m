% This is the M-file to calculate engine speed and vehicle acceleration
% when engine torque, brake torque, vehicle speed and gear position are
% given.

%
% Jianlong Zhang
% Department of Electrical Engineering-Systems
% University of Southern California
% June 09, 2005
%

function signal_out=veh_dyn(u)

%--------------------------------------------
% vehicle parameters
%--------------------------------------------
M=1701;                 % vehicle mass
hr=0.323;               % wheel radius
Jeng=0.169;             % engine inertia
Jwheel=2.603;           % wheel inertia
Fr=250;                 % rolling resistance force
c_a=0.3693;             % air drag coeff.
rd=1.0/3.06;            % driveline ratio


%--------------------------------------------
%  system input
%--------------------------------------------
T_engine=u(1);
T_brake=u(2);
vel=u(3);
gear_eng=u(4);

%--------------------------------------------
%  gear ratio
%--------------------------------------------
if (gear_eng==1)
    rg=0.3423;
elseif (gear_eng==2)
    rg=0.6378;
elseif (gear_eng==3)
    rg=1.0;
elseif (gear_eng==4)
    rg=1.4184;
else
    error('An error occured in veh_dyn.m: invalid gear!')
end

%--------------------------------------------
%  system_dynamics
%--------------------------------------------
rstar=rg*rd*hr;
we = vel/rstar;                                        %engine speed
T_eff = T_engine - T_brake*rg*rd;
J_eff = M*(rg*rd*hr)^2 + Jwheel*(rg*rd)^2 + Jeng;
v_dot = (T_eff - (Fr+c_a*vel^2)*rstar)*rstar/J_eff;   %acceleration
if (vel<=1e-3)&&(v_dot<=0)
    v_dot=0;
end

signal_out=[we, v_dot];