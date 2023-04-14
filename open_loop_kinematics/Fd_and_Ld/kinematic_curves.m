function [xt, vt, jerkt] = kinematic_curves(at, x0, v0, sampling_period)
%kinematic_curves Numerically compute jerk, velocity and position from
%acceleration and initial conditions
%   Detailed explanation goes here

jerkt = diff(at)/sampling_period;
delta_vt = cumtrapz(sampling_period, at);
vt = v0 + delta_vt;
xt = x0 + cumtrapz(sampling_period, vt);
end

