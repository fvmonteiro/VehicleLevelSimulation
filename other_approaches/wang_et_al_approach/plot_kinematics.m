function [h] = plot_kinematics(t, x, title_string, legends)
%plot_kinematics Plot kinematic variables contained in state vector x(t): 
% jerk(t), a(t), v(t) and s(t).
% x(t) is t by k by v where t is the number of time samples, k is the
% number of kinematic variables and v is the number of vehicles.

if exist('title_string', 'var')==1
    h = figure('Name', title_string);
else
    h = figure();
end

nv = size(x, 1); % number of variables: from 1 (only position) to 4 (including jerk)
kinematic_variables = {'s(t)', 'v(t)', 'a(t)', 'jerk(t)'};

for k = 1:nv
    subplot(nv, 1, k);
    y = squeeze(x(nv-k+1,:, :));
    plot(t, y);%, 'LineWidth', 1.5);
    grid on;
    xlabel('t');
    ylabel(kinematic_variables{nv-k+1});
    ymin = min(y(:));
    ymax = max(y(:));
    ylim([ymin-0.1*(ymax-ymin) ymax+0.1*(ymax-ymin)]);
    legend(legends);
    set(gca,'fontsize',14)
    lgd.FontSize = 14;
end

% Plot the gap if at least two trajectories are given
% if size(x, 3)>1 
%     yyaxis right
%     gap = x(:, 1, 2)-x(:, 1, 1)-v_length;
%     plot(t, gap, '--k', 'LineWidth', 1.5); grid on;
%     ylim([0 1.1*max(gap)]);
%     legend([legends(:)', {'gap'}])
% end

end

