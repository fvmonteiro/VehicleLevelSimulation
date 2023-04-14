close all;
clearvars;


%% Case 1: homogeneous vehicles, different follower and leader speed 
foll = Car();
mainV = Car();
leader = Car();

vf = 5:10:35;
vm = 0:0.1:35;
vl = 5:10:35;

df = zeros(length(vf), length(vm));
dl = zeros(length(vl), length(vm));

legFoll = cell(length(vf), 1);
for k = 1:length(vf)
    df(k, :) = foll.minGap(vf(k), vm, abs(mainV.minAccel)); %main to follower
    legFoll{k} = sprintf('v_F = %d', vf(k));
end
legLeader = cell(length(vl), 1);
for k = 1:length(vl)
    dl(k, :) = mainV.minGap(vm, vl(k), abs(leader.minAccel)); %main to leader
    legLeader{k} = sprintf('v_L = %d', vl(k));
end
% dl = mainV.minGap(vm, vl, abs(leader.minAccel)); %main to leader

figTitle = sprintf('follower type: %s; lc vehicle type: %s', foll.name, mainV.name);
figFoll = myMinGapPlot(vm, df, figTitle, legFoll);

figTitle = sprintf('lc vehicle type: %s; leader type: %s', mainV.name, leader.name);
figLeader = myMinGapPlot(vm, dl, figTitle, legLeader);

fig_name = sprintf('follower_gaps_varied_speeds');
mySaveFig(figFoll, fig_name);
                
fig_name = sprintf('leader_gaps_varied_speeds');
mySaveFig(figLeader, fig_name);

%% Case 2: follower and leader keep v=20m/s, heterogenous vehicles
followers = {Car(), Truck()};
mainVs = {Car(), Truck()};
leaders = {Car(), Truck()};

vf = 20;
vm = 0:0.1:35;
vl = 20;

nf = length(followers);
nm = length(mainVs);
nl = length(leaders);

df = zeros(nf*nm, length(vm));
dl = zeros(nl*nm, length(vm));
legFoll = cell(nf*nm, 1);
legLeader = cell(nl*nm, 1);
for m = 1:nm
    for f = 1:nf
        index = (m-1)*nf+f;
        df(index, :) = followers{f}.minGap(vf, vm, abs(mainVs{m}.minAccel)); %main to follower
        legFoll{index} = sprintf('follower: %s; lc vehicle: %s', followers{f}.name, mainVs{m}.name);
    end
    
    for l = 1:nl
        index = (m-1)*nl+l;
        dl(index, :) = mainVs{m}.minGap(vm, vl, abs(leaders{l}.minAccel)); %main to leader
        legLeader{index} = sprintf('lc vehicle: %s; leader: %s', mainVs{m}.name, leaders{l}.name);
    end
end

figTitle = 'v foll = 20m/s';
figFoll = myMinGapPlot(vm, df, figTitle, legFoll);

figTitle = 'v leader = 20m/s';
figLeader = myMinGapPlot(vm, dl, figTitle, legLeader);

fig_name = sprintf('follower_gaps_varied_vehs');
mySaveFig(figFoll, fig_name);
                
fig_name = sprintf('leader_gaps_varied_vehs');
mySaveFig(figLeader, fig_name);

%% Case 1a: 3D plos
% vfRange = 0:35;
% vmRange = 0:35;
% vlRange = 0:35;
% 
% [Vf, Vm] = meshgrid(vfRange, vmRange);
% df = foll.minGap(Vf, Vm, abs(mainV.minAccel)); %main to follower
% [Vl, Vm] = meshgrid(vlRange, vmRange);
% dl = mainV.minGap(Vm, Vl, abs(leader.minAccel)); %main to leader
% 
% % 3D plot
% figure; surf(Vf, Vm, df);
% xlabel('v_f');
% ylabel('v_m');
% zlabel('min gap');
% figure; surf(Vl, Vm, dl);
% xlabel('v_l');
% ylabel('v_m');
% zlabel('min gap');
% 
% % Contour lines ("isogaps")
% figure; contour(Vf, Vm, df, 20, 'showtext', 'on')
% xlabel('v_f');
% ylabel('v_m');
% figure; contour(Vl, Vm, dl, 20, 'showtext', 'on')
% xlabel('v_l');
% ylabel('v_m');

%% plot function

function [fig] = myMinGapPlot(vm, gaps, figTitle, leg)

fig = figure; plot(vm, gaps, 'LineWidth', 1.5); grid on;
ylim([0 inf])
title(figTitle)
xlabel('vehicle speed in original lane [m/s]')
ylabel('min gap[m]')
legend(leg, 'Location', 'Best')
set(gca,'fontsize',14)
lgd.FontSize = 18;

end

function [] = mySaveFig(figHandle, figName)
figHandle.WindowState = 'maximized';
% saveas(figHandle, ['G:/My Drive/Lane Change/images/' figName '.eps'], 'epsc');
saveas(figHandle, ['G:/My Drive/Lane Change/images/' figName '.pngclose']);
end
