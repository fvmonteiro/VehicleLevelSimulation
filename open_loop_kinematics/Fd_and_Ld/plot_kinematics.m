function [h] = plot_kinematics(t, a, v, x, v_length, title_string, legends)
%plt_kinematics Plot a(t), v(t) and x(t)
%   Detailed explanation goes here

if exist('title_string', 'var')==1
    h = figure('Name', title_string);
else
    h = figure();
end
subplot(3, 1, 1);
plot(t, a); grid on;
xlabel('t');
ylabel('a(t)');
legend(legends)
subplot(3, 1, 2);
plot(t, v); grid on;
xlabel('t');
ylabel('v(t)');
legend(legends)
subplot(3, 1, 3);
plot(t, x); grid on;
xlabel('t');
ylabel('x(t)');
ylim([0, round(max(max(x)))]);
if size(x, 2)>1 
    yyaxis right
    gap = x(:,2)-x(:,1)-v_length;
    plot(t, gap, '--k'); grid on;
    legend([legends(:)', {'gap'}])
else
    legend(legends)
end

end

