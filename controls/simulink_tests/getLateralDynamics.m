function [A, B] = getLateralDynamics(vx, Cf, Cr, lr, lf, m, Iz)
% Not sure if there's a x2 before all of them
a1 = 2*(Cf+Cr)/m;
a2 = 2*(Cf*lf-Cr*lr)/m;
a3 = 2*(Cf*lf-Cr*lr)/Iz;
a4 = 2*(Cf*lf^2+Cr*lr^2)/Iz;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;

A = [0 1 vx 0; 
    0 -a1/vx 0 -vx-a2/vx; 
    0 0 0 1; 
    0 -a3/vx 0 -a4/vx];

B = [0; -b1; 0; -b2];
end