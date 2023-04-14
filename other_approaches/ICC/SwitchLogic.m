% SwitchLogic.m decides whether the fuel subsystem or the brake
% subsystem should be active.
%
% 0: fuel is on
% 1: brake is on
%

%
% Jianlong Zhang
% Department of Electrical Engineering-Systems
% University of Southern California
% June 09, 2005
%

function b_on=SwitchLogic(u)

xr=u(1);    % inter-vehicle spacing
uc=u(2);    % controller output
b_on_old=u(3);  % last decision

xr_max=40;
xr_min=4;
u0=4;

if xr>xr_max
    b_on=0;
elseif xr<xr_min
    b_on=1;
elseif uc>0
    b_on=0;
elseif uc<-u0
    b_on=1;
else
    b_on=b_on_old;
end

    
    