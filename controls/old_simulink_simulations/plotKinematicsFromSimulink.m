function [h] = plotKinematicsFromSimulink(t, x, vLength, timeHeadway, g0, title_string, legends)
%plot_kinematics Plot kinematic variables contained in state vector x(t): 
% jerk(t), a(t), v(t) and s(t).
% x(t) is t by k*v where t is the number of time samples, k is the
% number of kinematic variables and v is the number of vehicles.

warning('Outdated: use VehiclePlotsClass')

if exist('title_string', 'var')==1
    h = figure('Name', title_string);
else
    h = figure();
end

% We make x a 3D matrix (easier to index for plotting)
nVar = 3; % number of variables: 3 (pos, vel, acc)
nVeh = size(x,2)/nVar;
x3d = reshape(x, [length(t), nVar, nVeh]);


kinematic_variables = {'headway error[m]', 'v[m/s]', 'a[m/s^2]', 'jerk[m/s^3]'};

gaps = -diff(x3d(:,1,:), 1, 3)-vLength;
headwayError = gaps-g0-timeHeadway*x3d(:, 2, 2:end);

for k = 1:nVar
    subplot(nVar, 1, k);
    if k ~= nVar
        y = squeeze(x3d(:, nVar-k+1, :));
    else
        y = squeeze(headwayError);
        legends = legends(2:end); % not proper coding
    end
    plot(t, y, 'LineWidth', 1.5); grid on;
    xlabel('t');
    ylabel(kinematic_variables{nVar-k+1});
    ymin = min(y(:));
    ymax = max(y(:));
    if ymax-ymin>0.25
        ylim([ymin-0.1*(ymax-ymin) ymax+0.1*(ymax-ymin)]);
    else
        ylim([ymin-1 ymax+1]);
    end
    legend(legends, 'Location', 'southeast');
    set(gca,'fontsize',14)
    lgd.FontSize = 14;
end

% Plot gaps if at least two trajectories are given
% if size(x, 3)>1 
%     yyaxis right
%     gap = x(:, 1, 1)-x(:, 1, 2)-v_length;
%     plot(t, gap, '--k', 'LineWidth', 1.5); grid on;
%     ylim([min([0, 1.1*min(gap)]) 1.1*max(gap)]);
%     ylabel('gap(t)')
%     legend([legends(:)', {'gap'}], 'Location', 'southeast')
% end

end

