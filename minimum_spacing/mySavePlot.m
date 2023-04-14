function [] = mySavePlot(figHandle, figName, fileType)

% userName = char(java.lang.System.getProperty('user.name'));
% if strcmp(userName,'fvall') %office computer
%     imgFolder = 'C:\Users\fvall\Google Drive\Lane Change\images\';
% else % home computer
    imgFolder = 'G:/My Drive/Lane Change/images/';
% end

% figHandle.WindowState = 'maximized';
if nargin<3
    fileType = 'eps';
end
if strcmp(fileType, 'eps')
    saveas(figHandle, [imgFolder figName '.' fileType], 'epsc');
else
    saveas(figHandle, [imgFolder figName '.' fileType]);
end


end