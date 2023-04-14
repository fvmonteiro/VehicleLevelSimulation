classdef VehiclePlots
    %VehiclePlots Class with methods to plot vehicle's states from
    %different simulation models.
    
    properties
        imgFolder = 'C:\Users\fvall\Google Drive\Lane Change\images\';
        %imgFolder = 'G:/My Drive/Lane Change/images/';
    end
    
    methods
        function obj = VehiclePlots( )
            %VehiclePlots Construct an instance of this class
            %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
        end
        
        function [figs] = plotDataStruct(obj, dataStruct, titleString)
            %plotDataStruct: Plot all variables in a dataStruct as defined 
            % in the runLCSimulation function.
            t = dataStruct.simTime;
            fields = fieldnames(dataStruct);
            figs = cell(length(fields)-1, 1);
            for k = 2:length(fields)
                ignoreLeader = 0;
                switch fields{k}
%                     case 'simTime'
%                         % nothing to plot
                    case 'longStates'
                        varNames = {'gap[m]', 'v[m/s]', 'a[m/s^2]'};
                        data = dataStruct.longStates;
                    case 'longErrors'
                        varNames = {'e_{headway}[m]', 'e_{v}[m/s]'};
                        data = dataStruct.longErrors;
                        ignoreLeader = 1;
                    case 'latStates'
                        varNames = {'Y', 'v_y', '\theta', '\omega'};
                        data = dataStruct.latStates;
                    case 'obsErrors'
                        varNames = {'Y', 'v_y', '\theta', '\omega'};
                        data = dataStruct.obsErrors;
                    case 'latErrors'
                        varNames = {'Lat error'};
                        data = dataStruct.latErrors;
                        ignoreLeader = 1;
                    case 'inputAndAccel'
                        varNames = {'\delta', 'Accel y'};
                        data = dataStruct.inputAndAccel;
                    otherwise
                        error('Unkown variable in dataStruct: %s', fields{k})
                end
                %data = getfield(dataStruct, fields{k});
                figs{k-1} = obj.allVariablesPlot(t, data, varNames, ignoreLeader,...
                        sprintf('%s_%s', titleString, fields{k}));
            end
        end
            
        
        function [figs] = plotGapGenerationData(~, dataStruct, mainVehName, vehNames)
            t = dataStruct.simTime;
            fields = fieldnames(dataStruct);
            figs = cell(length(fields)-1, 1);
            
            for k = 2:length(fields)
                switch fields{k}
                    case 'ggGap'
                        varNames = {'gap [m]'};
                        % ggGap: 1st column is the perceived (used) one,
                        % 2nd column is the gap to original leader, 3rd
                        % column is gap to virtual leader
                        data = dataStruct.ggGap;
                        figs{k-1} = figure('Name', ['Gaps for ' mainVehName]);
                        hold on; grid on;
                        plot(t, data(:, 1), 'LineWidth', 1.5); 
                        plot(t, data(:, 2:3), '--', 'LineWidth', 1.5); 
                        xlabel('t');
                        ylabel(varNames);
                        legend({'Perceived', ['To ' vehNames{1}], ['To ' vehNames{2}]}, 'Location', 'southeast');
                        set(gca,'fontsize',14)
                        lgd.FontSize = 14;
                    case 'ggErrors'
                        varNames = {'e_{headway}[m]', 'e_{v}[m/s]'};
                        data = dataStruct.ggErrors;
                        figs{k-1} = figure('Name', ['Errors for ' mainVehName]);
                        hold on;
                        
                        for n = 1:length(varNames)
                            subplot(length(varNames), 1, n);
                            plot(t, data(:, n), 'LineWidth', 1.5); grid on;
                            xlabel('t');
                            ylabel(varNames{n});
                        %legend({'Perceived Error', 'To L_o', 'To L_d'}, 'Location', 'southeast');
                            set(gca,'fontsize',14)
                        %lgd.FontSize = 14;
                        end
                end
            end
        end
        
        
        function [fig] = allVariablesPlot(~, t, variables, labels, ...
                ignoreLeader, titleString)
            if exist('titleString', 'var')==1
                fig = figure('Name', titleString);
            else
                fig = figure();
            end
            
            % Legends
            nPlottedVehicles = size(variables, 3);
            legends = cell(nPlottedVehicles, 1);
            if ignoreLeader; offset = 1; else; offset = 0; end
            for n = 1:nPlottedVehicles
                legends{n} = ['Veh ' num2str(n+offset)];
            end
            
            nVar = size(variables, 2);
            
            % Plot
            for k = 1:nVar
                subplot(nVar, 1, k);
                y = squeeze(variables(:, k, :));
                plot(t, y, 'LineWidth', 1.5); grid on;
                xlabel('t');
                ylabel(labels{k});
                legend(legends, 'Location', 'southeast');
                set(gca,'fontsize',14)
                lgd.FontSize = 14;
            end
        end
        
        function [] = saveBigPlot(obj, figHandle, figName)
            figHandle.WindowState = 'maximized';
            saveas(figHandle, [obj.imgFolder figName '.eps'], 'epsc');
        end
        
        function [] = mySavePlot(obj, figHandle, figName, fileType)
            % figHandle.WindowState = 'maximized';
            if nargin<4
                saveas(figHandle, [obj.imgFolder figName '.' 'eps'], 'epsc');
            else
                saveas(figHandle, [obj.imgFolder figName '.' fileType]);
            end
        end
        
    end
end

