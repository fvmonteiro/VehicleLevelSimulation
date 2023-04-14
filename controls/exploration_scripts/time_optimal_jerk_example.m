lw = 4;
finalTime = 20;
sampling = 0.1;

t = 0:sampling:finalTime;
lcTimes = 1;

delta1 = min(maxLatAcc/maxLatJerk, (lw/2/maxLatJerk)^(1/3));
delta2poly = delta1*maxLatJerk*[1 3*delta1 2*delta1^2] - [0 0 lw];
delta2candidates = roots(delta2poly);
delta2 = min(delta2candidates(delta2candidates>0));

idealJerk = zeros(length(t), 1);
for k = 1:length(lcTimes)
    t0 = lcTimes(k);
    t1 = t0+delta1;
    t2 = t1+delta2;
    t3 = t2+2*delta1;
    t4 = t3+delta2;
     t5 = t4+delta1;
     idealJerk((t>=t0 & t<t1) | (t>=t4 & t<t5)) = (-1)^(k+1)*maxLatJerk;
     idealJerk(t>=t2 & t<t3) = (-1)^(k)*maxLatJerk;
end

idealAccel = cumtrapz(sampling, idealJerk); 
idealSpeed = cumtrapz(sampling, idealAccel);
idealPos = cumtrapz(sampling, idealSpeed);

inputAccel = [t', idealAccel];

figure;
subplot(2, 2, 1)
plot(t, idealJerk, 'LineWidth', 1.5); grid on;
xlabel('t')
ylabel('j_{ref}', 'FontSize', 16)

subplot(2, 2, 2)
plot(t, idealAccel, 'LineWidth', 1.5); grid on;
xlabel('t')
ylabel('a_{ref}', 'FontSize', 16)

subplot(2, 2, 3)
plot(t, idealSpeed, 'LineWidth', 1.5); grid on;
xlabel('t')
ylabel('v_{ref}', 'FontSize', 16)

subplot(2, 2, 4)
plot(t, idealPos, 'LineWidth', 1.5); grid on;
xlabel('t')
ylabel('y_{ref}', 'FontSize', 16)