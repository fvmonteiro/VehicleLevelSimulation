%%%% Quick TF tests
clearvars;
close all;

s = tf([1, 0],1);
Kp = 1;
Kd = 0.5;
Ki = 2;

ka = 0.5;
tau = 0.6;

%% Const spacing, integral controller
A = [0 1 0; 0 0 1; -Ki -Kp -Kd];
B = [0; 0; 1];
C = [0, 1, 0];
D = 0;
systf = tf(ss(A, B, C, D));
inputtf = (Ki/s + Kp + Kd*s);
acceltf = inputtf;
Ge = systf*acceltf;% 'i' to 'i-1' error
figure;bode(Ge);

%% Const spacing with engine dynamics
inputtf = (Ki/s + Kp + Kd*s);
acceltf = inputtf/(tau*s+1);
Ge = systf*acceltf;% 'i' to 'i-1' error
figure; bode(Ge);

% Checking analytical magnitude
omega = logspace(-1, 2, 100);

magCheck = ((-Kd*omega.^2+Ki).^2 + (Kp*omega).^2)./...
    ((tau*omega.^4-Kd*omega.^2+Ki).^2 + (-omega.^3+Kp*omega).^2);
figure;
plot(log10(omega), 10*log10(magCheck));

%% Const spacing with leader acceleration
systf = (1-ka)*systf;
acceltf = (Ki/s + Kp + Kd*s);
Ge = systf*acceltf;% 'i' to 'i-1' error
bode(Ge)

% Checking analytical magnitude
% omega = logspace(-1, 2, 100);
% 
% magCheck = (1-ka)^2 * ((-Kd*omega.^2+Ki).^2 + (Kp*omega).^2)./...
%     ((-Kd*omega.^2+Ki).^2 + (-omega.^3+Kp*omega).^2);
% figure;
% plot(log10(omega), 10*log10(magCheck));

%% Const time headway
% h = 1;
A = [0 1 0; -h*Ki -h*Kp 1-h*Kd; -Ki -Kp -Kd];
B = [0; 0; 1];
C = [0 1 0];
D = 0;
sysss = ss(A, B, C, D);
systf = tf(sysss);

% x0 = [0; 1; -1];
% t = 0:0.1:20;
% uExt = zeros(length(t), 1);
% [y, t, x] = lsim(sysss, uExt, t, x0);
% 
% K = [Ki, Kp, Kd];
% u = K*x';
% plot(t, x); hold on;
% plot(t, u);
% legend('\int e_g', 'e_g', 'e_v', 'u');

% figure; bode(systf);

acceltf = 1/(1-Kd*h) * (Ki/s + Kp + Kd*s);
errorstf = systf*acceltf;
figure; bode(errorstf);

% omega = logspace(-2, 3, 100);
% 
% magCheck = (1/(1-h*Kd)^2) * ((-h*Kd*omega.^3+(h*Ki+Kp)*omega).^2 + (-(h*Kp+Kd)*omega.^2+Ki).^2)./...
%     ((-omega.^3+(h*Ki+Kp)*omega).^2 + (-(h*Kp+Kd)*omega.^2+Ki).^2);
% figure;
% plot(log10(omega), 10*log10(magCheck));

%% Const headway with leader acceleration
h = 0.33;
ka = 0.3;
Kd = 2;
Kp = 5;
Ki = 1;

A = [0 1 0; -h*Ki -h*Kp 1-h*Kd; -Ki -Kp -Kd];
B = [0; -h*ka; 1-ka];
C = [0 1 0];
D = 0;
sysss = ss(A, B, C, D);
systf = tf(sysss);

if (ka+2*h*Kd)<2 && h*Kd*(1+ka)<1 && (ka+h*Kd)<1
    disp('Could be string stable')
end

acceltf = 1/(1-Kd*h) * (Ki/s + Kp + Kd*s);
errorstf = systf*acceltf;
figure; bode(errorstf);

omega = logspace(-2, 3, 100);

% magCheck = (1/(1-h*Kd)^2) * ((-h*Kd*omega.^3+(h*Ki+Kp)*omega).^2 + (-(h*Kp+Kd)*omega.^2+Ki).^2)./...
%     ((-omega.^3+(h*Ki+Kp)*omega).^2 + (-(h*Kp+Kd)*omega.^2+Ki).^2);
% figure;
% plot(log10(omega), 10*log10(magCheck));
%% Const headway with filtered leader acceleration
h = 0.3;
ka = 0.3;
Kd = 2.5;
Kp = 2;
Ki = 0.5;
F = ka/(h*s+1);
% newSystf = tf([1-h*Kd-h*ka -h*Kd*ka 0], conv([1 ka], systf.den{1}));
newSystf = (-(h*s+1)*F+1-h*Kd)*s * tf(1, systf.den{1}); % code won't work because the numerator is zero
acceltf = 1/(1-Kd*h) * (Ki/s + Kp + Kd*s);

newErrorstf = newSystf*acceltf;
figure; bode(newErrorstf);

%% Const headway with accel dynamics
h = 0.5;
tau = 0.5;
tau2 = 1;
A = [0 1 0 0; 0 0 1 -h; 0 0 0 -1; Ki/tau Kp/tau Kd/tau -1/tau];
B = [0; 0; 1; 0];
C = [0 1 0 0];
D = 0;

sysss = ss(A,B,C,D);
systf = tf(sysss);

acceltf = (Kd*s+Kp+Ki/s)/(tau2*s+1-h*Kd);

errortf = systf*acceltf;
figure; bode(errortf)
figure; step(errortf)

%% KP controller with double integrator
% G = tf(1, [1 0 0]);
% C = tf([Kd Kp], 1);
% CG = series(C, G);
% sys = feedback(C*G, 1);
% 
% n = 5;
% figure; hold on;
% leg = cell(n, 1);
% % legStr = '';
% for k = 1:n
%     step(sys)
%     sys = sys*sys;
% %     legStr = [legStr ];
%     leg{k} = ['G_' num2str(k) '(s)'];
% end
% grid;
% legend(leg);



%% Closed-loop Response to vl in const spacing
% % C = -(Kp + Kd*s);
% C =  -(Kp + Kd*s+ Ki/s);
% Gvl = 1/s;
% Hvl = -1/s*C;
% Gcl = feedback(Gvl, Hvl);
% bode(Gcl)