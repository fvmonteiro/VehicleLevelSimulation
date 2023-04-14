clearvars;

lat_sim_parameters

tlc = 5;
v0 = vx0;  % from lat_sim_parameters
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

vX = [v0, diff(X)/delta_t];
vY = [0, diff(Y)/delta_t];
V = sqrt(vX.^2 + vY.^2);
aX = [0, diff(vX)/delta_t];
aY = [0, diff(vY)/delta_t];
theta = atan2(vY, vx0);  % NOTE: using vx0 here
omega = [diff(theta)/delta_t, 0];

vyr = vY - vx0*theta;

% L = lf + lr;
% tan_phi = L * (aY.*vX - aX.*vY) ./ V.^3;
% phi = atan(tan_phi);

start_time = 1;
sim_time = 10;

% phi_ts = make_timeseries(phi, t, start_time, sim_time);

% References
y_ts = make_timeseries(Y, t, start_time, sim_time);
% aY_ts = make_timeseries(aY, t, start_time, sim_time);
vyr_ts = make_timeseries(vyr, t, start_time, sim_time);
theta_ts = make_timeseries(theta, t, start_time, sim_time);
omega_ts = make_timeseries(omega, t, start_time, sim_time);
V_ts = make_timeseries(V, t, start_time, sim_time);

% Lat vel in body ref frame
% vy_brf = vY.*cos(theta) - vX.*sin(theta);

function ts = make_timeseries(vector, time, t0, tf)
    ts = timeseries(vector, time + t0);
    ts = addsample(ts, 'Data', vector(1), 'Time', 0);
    ts = addsample(ts, 'Data', vector(end), 'Time', tf);
end