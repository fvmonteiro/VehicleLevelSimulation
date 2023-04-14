clearvars; 
close all;

td = 0.5;
vf0 = 20;
af0 = 3;
df = -8;
jf = -20;
vl0 = 25.5;
dl = -5;

t1 = -(af0-df)/jf;
vfT = vf0+af0*td;
vft1 = vfT + af0*t1+1/2*jf*t1^2;
t2 = -vft1/df;

tL = -vl0/dl;

sampling = 0.01;
t = 0:sampling:2*(td+t1+t2);
al = zeros(length(t), 1);
af = zeros(length(t), 1);

al(t<=tL) = dl;
af(t<=td) = af0;
af(t>td & t<=td+t1) = af0 + jf*(t(t>td & t<=td+t1)-td);
af(t>td+t1 & t<=td+t1+t2) = df;

vl = vl0 + cumtrapz(sampling, al);
vf = vf0 + cumtrapz(sampling, af);

deltaGap = cumtrapz(sampling, vl-vf);

figure; hold on;
plot(t, al); plot(t, af);
legend('leader', 'foll')

figure; hold on;
plot(t, vl); plot(t, vf);
legend('leader', 'foll');

figure; plot(t, deltaGap)