function [] = saveBigPlot(figHandle, figName)

imgFolder = 'G:/My Drive/Lane Change/images/';
figHandle.WindowState = 'maximized';
saveas(figHandle, [imgFolder figName '.eps'], 'epsc');

end