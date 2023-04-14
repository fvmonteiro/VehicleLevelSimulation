function [MSS] = computeMSS(vf0, af, l, vl0, al, maxDeltaV)
%computeMSS computes the Minimum Safety Spacing in vehicle following
%according to Spacing And Capacity Evaluations For Different Ahs Concepts

global Ts

vf = vf0 + cumtrapz(Ts, af);
vl = vl0 + cumtrapz(Ts, al);

sf0 = 0;
sl0 = l;
sf = sf0 + cumtrapz(Ts, vf);
sl = sl0 + cumtrapz(Ts, vl);

delta = sf - sl - sf0 + sl0;

if nargin < 6 % no collision
    MSS = max(delta);
else % bounded relative speed collision
    deltaV = vf-vl;
    indMin = find(abs(deltaV-maxDeltaV)<0.1, 1);
    indMax = find(abs(deltaV-maxDeltaV)<0.1, 1, 'Last');
    MSS = delta([indMin, indMax]);
end

end