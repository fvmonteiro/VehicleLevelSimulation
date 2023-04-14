%%% Tests with simulink call inside loop

clearvars;
sampling = 0.01;
t = (0:sampling:1)';
accel = 1*ones(length(t), 1);
accel(rem(1:length(accel),2)==0)=0;
delta = 0*ones(length(t), 1);
% set_param('vehicleModel', 'StopTime', '5');
paramNameValStruct.SaveOutput = 'on';
paramNameValStruct.OutputSaveName = 'yout';
% paramNameValStruct.ExternalInput = '[t, accel, delta]';
x0 = 5;
v0 = 0;
% simOut = sim('vehicleModel', paramNameValStruct);

paramNameValStruct.ExternalInput = '[dt, da, dd]';
paramNameValStruct.SolverType = 'Fixed-Step';
paramNameValStruct.FixedStep = num2str(sampling);

x = zeros(length(t), 1);
v = zeros(length(t), 1);
x(1) = x0;
v(1) = v0;
set_param('vehicleModel', 'StartTime', '0',...
                'StopTime', num2str(sampling), 'SaveOutput', 'on', ...
                'OutputSaveName', 'yout', 'ExternalInput', '[dt, da, dd]',...
                'SolverType', 'Fixed-Step', 'FixedStep', num2str(sampling));
            
for k = 1:(length(t)-1)
    x0 = x(k);
    v0 = v(k);
%     set_param('vehicleModel', 'StartTime', num2str(t(k)),'StopTime', num2str(t(k)+sampling));
%     set_param('vehicleModel', 'StartTime', '0','StopTime', num2str(sampling));
%     dt = [t(k); t(k)+sampling];
    dt = 0;%; sampling];
    da = accel(k); %[accel(k-1);accel(k)];
    dd = delta(k); %[delta(k-1);delta(k)];
%     simOut = sim('vehicleModel', paramNameValStruct);
    simOut = sim('vehicleModel','ReturnWorkspaceOutputs','on');
    x(k+1) = simOut.yout{1}.Values.Data(2);
    v(k+1) = simOut.yout{3}.Values.Data(2);
%     x(k+1) = simOut(1);
%     v(k+1) = simOut(2);
    
end

