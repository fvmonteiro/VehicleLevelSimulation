function Y_ts = computeLateralReferenceTrajectory(vx0, tlc, sim_time)

v0 = vx0;
X0 = [0, v0, 0]';
Xf = [v0*tlc, v0, 0]';
Y0 = [0, 0, 0]';
Yf = [4, 0, 0]';

alpha = find_path_coeffs(tlc, X0, Xf);
beta = find_path_coeffs(tlc, Y0, Yf);

delta_t = 0.01;
t = 0:delta_t:tlc;
X = zeros(1, length(t));
Y = zeros(1, length(t));
for i = 1:length(alpha)
    X = X + alpha(i)*t.^(i-1);
end
for i = 1:length(beta)
    Y = Y + beta(i)*t.^(i-1);
end

% vX = [v0, diff(X)/delta_t];
% vY = [0, diff(Y)/delta_t];
% V = sqrt(vX.^2 + vY.^2);
% aX = [0, diff(vX)/delta_t];
% aY = [0, diff(vY)/delta_t];
% theta = atan2(vY, vX);
% omega = [diff(theta)/delta_t, 0];

start_time = 1;
Y_ts = make_timeseries(Y', t, start_time, sim_time);
% theta_ts = make_timeseries(theta, t, start_time, sim_time);

function ts = make_timeseries(vector, time, t0, tf)
    ts = timeseries(vector, time + t0);
    ts = addsample(ts, 'Data', vector(1), 'Time', 0);
    ts = addsample(ts, 'Data', vector(end), 'Time', tf);
end

end