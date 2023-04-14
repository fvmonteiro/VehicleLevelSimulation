%% Numerical checks for string stability
clearvars

tau = 0.5;
h = 0.6;
p = 10;

% Gain conditions without filter:
% ka > max(0, tau/h-1/2)
% kd > 0
% kv = 1/h
% codition: kg > 0
ka = tau/h-0.2;
kd = 0.1;
kv = p/(p*h-1); %1/h;
kg = 0.1;

G = gapErrorToGapErrorTF(tau, h, p, ka, kd, kv, kg);

function G = gapErrorToGapErrorTF(tau, h, p, ka, kd, kv, kg)

s = tf('s');
num = (p*ka + kd)*s^2 + (kg + p*(kd + kv))*s + p*kg;
den = (p + s)*(tau*s^3 + (ka + h*kd + 1)*s^2 + (kd + kv + h*kg)*s + kg);

G = num/den;

[mag, ~, wout] = bode(G);
[maxMag, maxIdx] = max(mag);
if maxMag > 1
    fprintf(['Longitudinal controller is not string stable.\n' ...
             '|G(jw)| = %g at w = %g\n'], maxMag, wout(maxIdx));
    bode(G)
else
    disp('All good')
end


end