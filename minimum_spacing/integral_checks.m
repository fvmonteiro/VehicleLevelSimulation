clearvars;
close all;
%% Speed variation during lane change check

% tf = 10;
% v0 = 5;
% t = 0:0.01:tf;
% ac = 1.5;
% jc = 4;
% t1 = ac/jc;
% 
% jt = zeros(length(t), 1);
% jt(t<=t1) = jc;
% jt(t>=tf-t1) = -jc;
% at = cumtrapz(t, jt);
% vt = v0 + cumtrapz(t, at);
% 
% deltaVf = trapz(t, at);
% deltaVfcheck = ac*tf - ac^2/jc;
% disp(['Integral - analytical = ' num2str(deltaVf-deltaVfcheck)]);
% 
% figure;
% subplot(3, 1, 1); plot(t, jt);
% ylabel('jerk')
% subplot(3, 1, 2); plot(t, at);
% ylabel('accel')
% subplot(3, 1, 3); plot(t, vt);
% ylabel('vel')

%% Speed ajustment before lane change check
tf = 5;
x0 = 0;
v0 = 10;
delta = 0.01;
t = 0:delta:tf;
ac = 1.5;
bc = 1;
jc = 5;
t1 = bc/jc;
t5 = ac/jc;
t3 = t1+t5;
% fixing t2 and t4 to check, test finding them out later
t2 = 2;
t4 = tf-t1-t2-t3-t5;

interval{1} = t>0 & t<=t1;
interval{2} = t>t1 & t<=(t1+t2);
interval{3} = t>(t1+t2) & t<=(t1+t2+t3);
interval{4} = t>(t1+t2+t3) & t<=(t1+t2+t3+t4);
interval{5} = t>(t1+t2+t3+t4);

idx(1) = find(abs(t-t1)<delta, 1);
idx(2) = find(abs(t-t1-t2)<delta, 1);
idx(3) = find(abs(t-t1-t2-t3)<delta, 1);
idx(4) = find(abs(t-t1-t2-t3-t4)<delta, 1);


jt = zeros(length(t), 1);
jt(interval{1}) = -jc;
jt(interval{3}) = jc;
jt(interval{5}) = -jc;

at = cumtrapz(t, jt);
% at2 = zeros(length(t), 1);
% at2(interval{1}) = -jc*t(interval{1});


vt = v0 + cumtrapz(t, at);
vt2 = zeros(length(t), 1);
vt2(1) = v0;
vt2(interval{1}) = v0-jc*t(interval{1}).^2/2;
vt2(interval{2}) = v0-jc*t1^2/2 - bc*(t(interval{2})-t1);
vt2(interval{3}) = v0-jc*t1^2/2 - bc*(t2) + jc*(t(interval{3})-t1-t2).^2/2 - bc*(t(interval{3})-t1-t2);
vt2(interval{4}) = v0-jc*t1^2/2 - bc*(t2+t3) + jc*(t3).^2/2 + ac*(t(interval{4})-t1-t2-t3);
vt2(interval{5}) = v0-jc*t1^2/2 - bc*(t2+t3) + jc*(t3).^2/2 + ac*(t4) + ...
    ac*(t(interval{5})-t1-t2-t3-t4) - jc*(t(interval{5})-t1-t2-t3-t4).^2/2;

xt = cumtrapz(t, vt);
xt2 = zeros(length(t), 1);
xt2(1) = x0;
xt2(interval{1}) = xt2(1) + vt2(1)*t(interval{1}) - ...
    jc*t(interval{1}).^3/6;
xt2(interval{2}) = xt2(idx(1)) + vt2(idx(1))*(t(interval{2})-t1) - ...
    bc*(t(interval{2})-t1).^2/2;
xt2(interval{3}) = xt2(idx(2)) + vt2(idx(2))*(t(interval{3})-t1-t2) - ...
    bc*(t(interval{3})-t1-t2).^2/2 + jc*(t(interval{3})-t1-t2).^3/6;
xt2(interval{4}) = xt2(idx(3)) + vt2(idx(3))*(t(interval{4})-t1-t2-t3) + ...
    ac*(t(interval{4})-t1-t2-t3).^2/2;
xt2(interval{5}) = xt2(idx(4)) + vt2(idx(4))*(t(interval{5})-t1-t2-t3-t4) + ...
    ac*(t(interval{5})-t1-t2-t3-t4).^2/2 - jc*(t(interval{5})-t1-t2-t3-t4).^3/6;

figure;
subplot(4, 1, 1); plot(t, jt);
ylabel('jerk')
subplot(4, 1, 2); hold on;
plot(t, at); %plot(t, at2);
ylabel('accel')
subplot(4, 1, 3); hold on;
plot(t, vt); plot(t, vt2)
ylabel('vel')
subplot(4, 1, 4); hold on;
plot(t, xt); plot(t, xt2);
ylabel('x');
