% This function indicates which gear to be used for a given speed

%
% Jianlong Zhang
% Department of Electrical Engineering-Systems
% University of Southern California
% June 09, 2005
%

function gear_eng=gearshift(u)

if (u>=0)&(u<=9.65)
    gear_eng=1;
elseif (u>9.65)&(u<=17.97)
    gear_eng=2;
elseif (u>17.97)&(u<=28.95)
    gear_eng=3;
elseif (u>28.95)
    gear_eng=4;
else
    error('An error occured in gearshift.m: invalid speed signal!')
end