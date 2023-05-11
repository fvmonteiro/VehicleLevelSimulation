clearvars;

lat_sim_parameters

tlc = 5;
v0 = vx0;  % from lat_sim_parameters
X0 = [0, v0, 0]';
Xf = [v0*tlc, v0, 0]';
Y0 = [0, 0, 0]';
Yf = [4, 0, 0]';

alpha = find_path_coeffs(tlc, [v0, 0]', [v0*1.2, 0]');
beta = find_path_coeffs(tlc, Y0, Yf);

delta_t = 0.01;
t = 0:delta_t:tlc;
V = zeros(1, length(t));
Y = zeros(1, length(t));
for i = 1:length(alpha)
    V = V + alpha(i)*t.^(i-1);
end
for i = 1:length(beta)
    Y = Y + beta(i)*t.^(i-1);
end

% vX = [v0, diff(X)/delta_t];
vY = [diff(Y)/delta_t, 0];
% V = sqrt(vX.^2 + vY.^2);
dotV = [diff(V)/delta_t, 0];
% aX = [0, diff(vX)/delta_t];
aY = [diff(vY)/delta_t, 0];
theta = atan2(vY, vx0);  % NOTE: using vx0 here
omega = [0, diff(theta)/delta_t];

vyr = vY - vx0*theta;

% L = lf + lr;
% tan_phi = L * (aY.*vX - aX.*vY) ./ V.^3;
% phi = atan(tan_phi);

start_time = 2;
sim_time = 10;

% phi_ts = make_timeseries(phi, t, start_time, sim_time);

% References
y_ts = make_timeseries(Y, t, start_time, sim_time);
vY_ts = make_timeseries(vY, t, start_time, sim_time);
% aY_ts = make_timeseries(aY, t, start_time, sim_time);
vyr_ts = make_timeseries(vyr, t, start_time, sim_time);
theta_ts = make_timeseries(theta, t, start_time, sim_time);
omega_ts = make_timeseries(omega, t, start_time, sim_time);
V_ts = make_timeseries(V, t, start_time, sim_time);
dotV_ts = make_timeseries(dotV, t, start_time, sim_time);

% Lat vel in body ref frame
% vy_brf = vY.*cos(theta) - vX.*sin(theta);

function ts = make_timeseries(vector, time, t0, tf)
    ts = timeseries(vector, time + t0);
    ts = addsample(ts, 'Data', vector(1), 'Time', 0);
    ts = addsample(ts, 'Data', vector(end), 'Time', tf);
end