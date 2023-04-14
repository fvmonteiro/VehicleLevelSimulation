%% Analytically check continuity of the collision time
% The symbolical analytical solutions are too complex for Matlab to 
%simplify

clearvars

%% Variable definitions
syms jE aE dE dL tauD % parameters
syms vL vE g0 lambda0 lambda1% initial conditions

deltaV = vL - vE;
tauJ = (aE + dE)/jE;
% lambda0 = -(aE + dE)/2 * (tauD^2 + tauD*tauJ + tauJ^2/3);
% lambda1 = (aE + dE)*(tauD + tauJ/2);
%% Collision time cases
tc1 = (deltaV + sqrt(deltaV^2 + 2*g0*(aE+dL)))/(aE+dL);
tc3_root1 = (deltaV - lambda1 + sqrt((lambda1 + deltaV)^2 ...
    + 2*(g0-lambda0)*(dL-dE)))/(dL - dE);
tc3_root2 = (deltaV - lambda1 - sqrt((lambda1 + deltaV)^2 ...
    + 2*(g0 - lambda0)*(dL-dE)))/(dL - dE);
tc4 = (vE + lambda1 - sqrt((vE + lambda1)^2 ...
    - 2*dE*(g0 + vL^2/2/dL - lambda0)))/dE;

%% Thresholds
g1 = tauD*(tauD/2 * (aE + dL) - deltaV);
g2 = (tauD + tauJ)*(lambda1 - deltaV - (tauD+tauJ)*(dE-dL)/2) + lambda0;
g3 = vL/dL * (lambda1 + vE - (dE + dL)*vL/2/dL) + lambda0;

%% Continuity from cases 3 to 4
tc3_root1_upper = subs(tc3_root1, g0, g3);
tc3_root2_upper = subs(tc3_root2, g0, g3);
tc4_lower = subs(tc4, g0, g3);

check_root1 = simplify(tc4_lower - tc3_root1_upper);
check_root2 = simplify(tc4_lower - tc3_root2_upper);