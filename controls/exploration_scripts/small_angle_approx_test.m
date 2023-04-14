%%% Check delta as a function of v^2
close all;

max_ay = 0.67;
theta = 0;
veh_len = 5;
a = 0;
v_mph = 10:70; %mph
v = v_mph*1609/3600;
delta = atan2(veh_len*(max_ay - a*sin(theta)), v.^2*cos(theta));

figure('Name', 'Steering Angle');
plot(v_mph, delta)
xlabel('v(mph)'); ylabel('\delta')
grid

figure('Name', 'Small Angle Approx.')
plot(delta, [delta; tan(delta)])
xlabel('\delta');
legend({'\delta', 'tan(\delta)'});
grid
max(abs(delta-sin(delta)))