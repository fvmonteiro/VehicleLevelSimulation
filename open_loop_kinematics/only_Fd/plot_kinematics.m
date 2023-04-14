function [h] = plot_kinematics(t, jerk, a, v, x, title_string)
%plt_kinematics Plot a(t), v(t) and x(t)
%   Detailed explanation goes here

if exist('title_string', 'var')==1
    h = figure('Name', title_string);
else
    h = figure();
end
subplot(4, 1, 1);
plot(t, jerk); grid on;
xlabel('t');
ylabel('jerk(t)');
subplot(4, 1, 2);
plot(t, a); grid on;
xlabel('t');
ylabel('a(t)');
% ylim([0, round(max(a))]);
subplot(4, 1, 3);
plot(t, v); grid on;
xlabel('t');
ylabel('v(t)');
%ylim([0, round(max(v))]);
subplot(4, 1, 4);
plot(t, x); grid on;
xlabel('t');
ylabel('gap(t)');
ylim([0, round(max(x))]);

end

