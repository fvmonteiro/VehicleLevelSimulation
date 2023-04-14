function [] = plotMSSstruct(MSSstruct, vM, specialIdx)
%plotMSSstruct Quickly plot all fields of an Minimum Safety Spacing
%Structure
%specialIdx input being used for very specific check, can be
%deleted later on

fieldNames = fields(MSSstruct);
for k = 1:length(fieldNames)
    figure; hold on; grid on;
    plot(vM, MSSstruct.(fieldNames{k}));
    xlabel('v_M [m/s]');
    ylabel('min gap [m]');
    title(fieldNames{k})
    
    if nargin>2
        plot(vM, specialIdx.(fieldNames{k}));
    end
    
end
end

