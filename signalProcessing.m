%SIGNALPROCESSING Post-process rotor system response data and generate visualizations
%
% Syntax:
%   signalProcessing(q, dq, t, Parameter, SwitchFigure)
%   signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan)
%   signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan, samplingFrequency, NameValueArgs)
%
% Input Arguments:
%   q - [n×m double]                 Displacement time history (DOFs × time)
%   dq - [n×m double]                Velocity time history
%   t - [1×m double]                 Time vector (seconds)
%   Parameter - System configuration structure containing:
%       .Mesh: [1×1 struct]          Discretization data with:
%           .nodeNum: integer         Total number of nodes
%           .dofOnNodeNo: integer     DOF indices per node
%       .Status: [1×1 struct]        Operational parameters:
%           .vmax: double            Maximum rotational speed (rad/s)
%   SwitchFigure - [1×1 struct]      Visualization control flags:
%       .displacement: logical       Enable displacement plots
%       .axisTrajectory: logical     Enable 2D axis trajectory plots
%       .phase: logical             Enable phase portraits
%       .fftSteady: logical         Enable steady-state FFT
%       .fftTransient: logical      Enable transient FFT
%       .poincare: logical          Enable Poincaré maps
%       .saveFig: logical           Save .fig files
%       .saveEps: logical           Save .eps files
%       .axisTrajectory3d: logical  Enable 3D shaft trajectories
%       .poincare_phase: logical    Enable phase+Poincaré composite plots
%   tSpan - [1×2 double]             Processing time range [start, end] (seconds)
%   samplingFrequency - double      Original sampling frequency (Hz)
%   NameValueArgs - Optional parameters:
%       .fftXlim: double            Maximum FFT frequency display (default: 500Hz)
%       .T_window: double           STFT window duration (default: (tSpan(2)-tSpan(1))/20s)
%       .overlap: double            STFT window overlap ratio (default: 2/3)
%       .fftisPlot3DTransient: logical 3D transient FFT visualization
%       .fftSteadyLog: logical      Logarithmic FFT amplitude scale
%       .reduceInterval: integer    Data downsampling factor
%       .isPlotInA4: logical        A4 paper formatting
%       .f: [1×k double]            Custom frequency vector for STFT
%
% Description:
%   Performs comprehensive post-processing of rotor system dynamics data:
%   - Generates time-domain plots: displacement histories, velocity profiles
%   - Creates phase space visualizations: 2D/3D trajectories, Poincaré sections
%   - Conducts spectral analysis: steady-state/transient FFT, STFT spectrograms
%   - Supports multi-shaft systems with interactive 3D trajectory visualization
%   - Implements advanced signal processing techniques:
%       * Short-Time Fourier Transform (STFT) with customizable windows
%       * Automated Poincaré section extraction at rotational periods
%       * Multi-resolution analysis through data downsampling
%
% Features:
%   - Automated directory management for output organization
%   - Intelligent DOF labeling for bearings and shaft components
%   - Adaptive unit handling (meters/radians) based on DOF type
%   - Publication-quality figure formatting with LaTeX-style annotations
%   - Multi-format output support (.fig, .png, .eps)
%
% Examples:
%   % Basic usage with default visualization parameters
%   load simulationData.mat
%   Switch = struct('displacement',true, 'fftSteady',true);
%   signalProcessing(q, dq, t, sysParams, Switch);
%
%   % Advanced transient analysis with custom STFT parameters
%   signalProcessing(q, dq, t, sysParams, Switch, [1 5], 2000, ...
%       'T_window', 0.2, 'overlap', 0.75, 'fftXlim', 1000);
%
% See also CALCULATERESPONSE, SPECTROGRAM, PLOT2DSTANDARD, SAVEFIGURE
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.


function signalProcessing(q, dq, t, Parameter, tSpan, samplingFrequency, SwitchFigure, NameValueArgs)

arguments % name value pair
    q
    dq
    t
    Parameter
    tSpan
    samplingFrequency
    SwitchFigure = struct('displacement', true, 'axisTrajectory', false, 'phase', false, 'fftSteady', true, 'fftTransient', false, 'poincare', false, 'axisTrajectory3d', false, 'poincare_phase', false)
    NameValueArgs.fftXlim = 500; % (Hz)
    NameValueArgs.T_window = (tSpan(2) - tSpan(1)) / 20; % (s), for transient FFT
    NameValueArgs.overlap = 2/3 % for transient FFT
    NameValueArgs.fftisPlot3DTransient = true;
    NameValueArgs.fftSteadyLog = false;
    NameValueArgs.reduceInterval = 1;
    NameValueArgs.isPlotInA4 = false;
    NameValueArgs.f = 1:0.2:200;
end

% input parameter
if nargin < 7 && SwitchFigure.fftTransient == false && SwitchFigure.fftSteady == false
    samplingFrequency = [];
elseif nargin < 7 && (SwitchFigure.fftTransient == true || SwitchFigure.fftSteady == true)
    error('must input samplingFrequency in signalProcessing( )')
end

if nargin < 6 
    tSpan = [t(1), t(end)];
end

%%

% initialize the directory
refreshDirectory('signalProcess');

%%

% find the index in t to match tSpan
[~, tStartIndex] = min(abs(t - tSpan(1)));
[~, tEndIndex]   = min(abs(t - tSpan(2))); 

%%

%name the label
dofNum          = Parameter.Mesh.dofNum;
nodeNum         = Parameter.Mesh.nodeNum;
dofOnNodeNo     = Parameter.Mesh.dofOnNodeNo;
Node            = Parameter.Mesh.Node;
figureIdentity  = cell(1,dofNum);
nodeNo          = 1;
dofInThisNode   = 0;
for iDof = 1:1:dofNum
    if nodeNo == dofOnNodeNo(iDof)
        dofInThisNode = dofInThisNode + 1;
    else
        nodeNo = nodeNo + 1;
        dofInThisNode = 1;
    end
    figureIdentity{iDof}=['Node-',num2str(dofOnNodeNo(iDof)),'-DOF-',num2str(dofInThisNode)];
end


% save figure name
dofNo = 1; % default value for app: postprocess
save("postProcessData", 'figureIdentity', 'dofNo')

%% Part I: Displacement

if SwitchFigure.displacement
    refreshDirectory('signalProcess/displacement')
    xspan = t(tStartIndex:tEndIndex);
    yspan = q(:,tStartIndex:tEndIndex);
    for iDof=1:1:dofNum
        figureName = ['Displacement ',figureIdentity{iDof}];
        
        isBearing = Node(dofOnNodeNo(iDof)).isBearing;
        isTranslation = rem(iDof, 4)==1 || rem(iDof, 4)==2;
        if isBearing
            ylabelname = '$q$ (m)';
        elseif isTranslation
            ylabelname = '$q$ (m)';
        else
            ylabelname = '$q$ (rad)';
        end
        
        xlabelname = '$t$ (s)';
        h = figure('Visible', 'off');
        isUsedInA4 = NameValueArgs.isPlotInA4;
        [~] = plot2DStandard(xspan, yspan(iDof,:), xlabelname, ylabelname, isUsedInA4);
        title(figureName,'Fontname', 'Arial');
        figurePath = ['signalProcess/displacement/', figureIdentity{iDof}];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close(h)
    end
end

%% Part II: Axis Trajectory

if SwitchFigure.axisTrajectory
    refreshDirectory('signalProcess/axisTrajectory')
    xspan = zeros(nodeNum,size(q,2));
    yspan = xspan;
    dofInThisNode = 0;
    nodeNo = 1;
    for iDof=1:1:dofNum
        if nodeNo == dofOnNodeNo(iDof)
            dofInThisNode = dofInThisNode + 1;
        else
            nodeNo = nodeNo + 1;
            dofInThisNode = 1;
        end % end if
        
        if dofInThisNode  == 1
            xspan(nodeNo, :) =  q(iDof, :);
        elseif dofInThisNode == 2
            yspan(nodeNo, :) = q(iDof, :);
        end % end if
    end 
    yspan = yspan(:,tStartIndex:tEndIndex);
    xspan = xspan(:,tStartIndex:tEndIndex);
    
    for iNode = 1:1:nodeNum
        figureName = ['AxisTrajectory ','Node-',num2str(iNode)];
        ylabelname = '$W$ (m)';
        xlabelname = '$V$ (m)';
        h = figure('Visible', 'off');
        isUsedInA4 = NameValueArgs.isPlotInA4;
        [~] = plot2DStandard(xspan(iNode,:),yspan(iNode,:), xlabelname, ylabelname, isUsedInA4);
        set(gcf,'Units','centimeters','Position',[6 6 7.2 4]);%Set the size of figure(for A4)
        title(figureName, 'Fontname', 'Arial');
        figurePath = ['signalProcess/axisTrajectory/',['Node-',num2str(iNode)]];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close
    end 
end

%% Part III: Phase Diagram 

if SwitchFigure.phase
    refreshDirectory('signalProcess/phase')
    xspan = q(:,tStartIndex:tEndIndex);%Extract the x
    yspan = dq(:,tStartIndex:tEndIndex);%Extract the y
    %cspan = t(tStartIndex:tEndIndex);
    for iDof = 1:1:dofNum
        figureName = ['Phase ',figureIdentity{iDof}];%name the figure
        yspan(iDof,end) = NaN;%set the NaN at the end of data in order to control curve not close
        
        isBearing = Node(dofOnNodeNo(iDof)).isBearing;
        isTranslation = rem(iDof, 4)==1 || rem(iDof, 4)==2;
        if isBearing
            ylabelname = '$\dot{q}$ (m/s)';
            xlabelname = '$q$ (m)';
        elseif isTranslation
            ylabelname = '$\dot{q}$ (m/s)';
            xlabelname = '$q$ (m)';
        else
            ylabelname = '$\dot{q}$ (rad/s)';
            xlabelname = '$q$ (rad)';
        end
        
        h = figure('Visible', 'off'); 
        isUsedInA4 = NameValueArgs.isPlotInA4;
        [~] = plot2DStandard(xspan(iDof,:),yspan(iDof,:), xlabelname, ylabelname, isUsedInA4);
        set(gcf,'Units','centimeters','Position',[6 6 7.2 4]);%Set the size of figure(for A4)
        title(figureName,'Fontname', 'Arial');
        figurePath = ['signalProcess/phase/',figureIdentity{iDof}];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close
    end 
end

%% Part IV: FFT--Steady State

if SwitchFigure.fftSteady
    refreshDirectory('signalProcess/fftSteady')
    signal = q(:,tStartIndex:tEndIndex);
    signallength = length(signal);   
    for iDof=1:1:dofNum
        % calculate
        Y  = fft(signal(iDof,:)); 
        P2 = abs(Y/signallength); 
        P1 = P2(1:floor(signallength/2)+1);
        P1(2:end-1) = 2*P1(2:end-1); 
        f  = samplingFrequency/NameValueArgs.reduceInterval...
             *(0:(signallength/2))/signallength;
        xspan=f;
        yspan=P1;
        % plot
        figureName=['FFT ',figureIdentity{iDof}];
        ylabelname='$|$P1$|$';
        xlabelname='$f$ (Hz)';
        h=figure('Visible', 'off');
        if NameValueArgs.fftSteadyLog
            plot(xspan,yspan,'-','LineWidth',0.5,'color',[0 0.30078125 0.62890625]);
            set(gca, 'YScale', 'log')
        else
            stem(xspan,yspan,'-','LineWidth',0.5,'color',[0 0.30078125 0.62890625],'Marker','none');
        end
        isUsedInA4 = NameValueArgs.isPlotInA4;
        isOnlySet = true;
        [~] = plot2DStandard([], [], xlabelname, ylabelname, isUsedInA4, isOnlySet);
        xlim([0 NameValueArgs.fftXlim])
        title(figureName,'Fontname', 'Arial');
        % save figure
        figurePath = ['signalProcess/fftSteady/',figureIdentity{iDof}];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close(h)
    end
end

%% Part V: FFT--transient State

if SwitchFigure.fftTransient
    
    refreshDirectory('signalProcess/fftTransient')
    
    % load constant
    T_window = NameValueArgs.T_window;
    overlap = NameValueArgs.overlap;
    % plot setting
    window = round(T_window*samplingFrequency); % time window [s]
    noverlap = round(window*overlap); % overlap point number
    f = NameValueArgs.f;


    % plot for all dofs
    for iDof = 1:1:dofNum
        % define data here
        data = q(iDof, tStartIndex:tEndIndex)';
        
        % create figre
        h = figure('Visible', 'off');
        
        if NameValueArgs.fftisPlot3DTransient
            % plot 3d fft [Hz]
            [s_fft3d, f_fft3d, t_fft3d] = spectrogram(data ,window, noverlap, f, samplingFrequency);
            waterfall(f_fft3d(5:end), t_fft3d+tSpan(1), abs(s_fft3d(5:end,:))'.^2)

            set(gca, ...
                    'Box'         , 'on'                        , ...
                    'TickDir'     , 'in'                        , ...
                    'XMinorTick'  , 'off'                       , ...
                    'YMinorTick'  , 'off'                       , ...
                    'TickLength'  , [.01 .01]                   , ...
                    'LineWidth'   , 0.5                         , ...
                    'XGrid'       , 'off'                       , ...
                    'YGrid'       , 'off'                       , ...
                    'FontSize'    , 7                           , ... 
                    'FontName'    ,'Times New Roman'            ,...
                    'layer'       , 'top'                       , ...
                    'LooseInset'  , [0,0,0,0]) 
                
            xlabelname = '$f$ (Hz)';
            ylabelname = '$t$ (s)';
            zlabelname = '$dB/Hz$';
            xlabel(xlabelname, 'FontName', 'Times New Roman', 'Interpreter','latex', 'FontSize',9);
            ylabel(ylabelname, 'FontName', 'Times New Roman', 'Interpreter','latex', 'FontSize',9);
            zlabel(zlabelname, 'FontName', 'Times New Roman', 'Interpreter','latex', 'FontSize',9);
            xlim([0 NameValueArgs.fftXlim])
            if NameValueArgs.isPlotInA4
                set(gcf,'Units','centimeters','Position',[6 6 7.2 4]);%Set the size of figure(for A4)
            else
                set(gcf,'Units','centimeters','Position',[6 6 10 5]);%Set the size of figure(for viewing)
            end
            view(30, 30)
        else
            % plot 2d fft [Hz]
            spectrogram(data ,window, noverlap, f, samplingFrequency, 'yaxis');        
            h = change2dTransientFormat(h, NameValueArgs.fftXlim);
        end % end if
        figureName=['FFT ',figureIdentity{iDof}];
        title(figureName,'Fontname', 'Arial');

        % save figure
        figurePath = ['signalProcess/fftTransient/',figureIdentity{iDof}];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close

    end % end for iDof
   
end % end if

%% Part VI: Poincare surface of section

if SwitchFigure.poincare
    
    refreshDirectory('signalProcess/poincare')

    shaftNum = Parameter.Shaft.amount;
    % get revolution start point
    tk = get_tk_from_simulation2(Parameter.Status, t(tStartIndex:tEndIndex), shaftNum);


    %Initialize to save data
    saveDisplacement = cell(shaftNum,1); 
    saveSpeed = cell(shaftNum,1);


    % calculate poincare point
    for iSection = 1:1:shaftNum
        q_section = interp1(t, q', tk{iSection});
        dq_section = interp1(t, dq', tk{iSection});
        saveDisplacement{iSection} = q_section';
        saveSpeed{iSection} = dq_section';
    end % end for iSection
    
    
    % plot
    xspan=saveDisplacement; % cell data
    yspan=saveSpeed;
    for iDof=1:1:dofNum

        figureName = ['Poincare ',figureIdentity{iDof}];
        
        isBearing = Node(dofOnNodeNo(iDof)).isBearing;
        isTranslation = rem(iDof, 4)==1 || rem(iDof, 4)==2;
        if isBearing
            ylabelname = '$\dot{q}$ (m/s)';
            xlabelname = '$q$ (m)';
        elseif isTranslation
            ylabelname = '$\dot{q}$ (m/s)';
            xlabelname = '$q$ (m)';
        else
            ylabelname = '$\dot{q}$ (rad/s)';
            xlabelname = '$q$ (rad)';
        end
        
        h=figure('Visible', 'off');
        for iSection = 1:1:shaftNum
            plot(xspan{iSection}(iDof,:),yspan{iSection}(iDof,:),'.'); hold on%plot
        end
        hold off
        xMin = min(q(iDof,tStartIndex:tEndIndex))-0.15*abs(min(q(iDof,tStartIndex:tEndIndex))-max(q(iDof,tStartIndex:tEndIndex)));
        xMax = max(q(iDof,tStartIndex:tEndIndex))+0.15*abs(min(q(iDof,tStartIndex:tEndIndex))-max(q(iDof,tStartIndex:tEndIndex)));
        yMin = min(dq(iDof,tStartIndex:tEndIndex))-0.15*abs(min(dq(iDof,tStartIndex:tEndIndex))-max(dq(iDof,tStartIndex:tEndIndex)));
        yMax = max(dq(iDof,tStartIndex:tEndIndex))+0.15*abs(min(dq(iDof,tStartIndex:tEndIndex))-max(dq(iDof,tStartIndex:tEndIndex)));
        xlim([xMin xMax]);
        ylim([yMin yMax]);
        isUsedInA4 = NameValueArgs.isPlotInA4;
        isOnlySet = true;
        [~] = plot2DStandard([], [], xlabelname, ylabelname, isUsedInA4, isOnlySet);
        set(gcf,'Units','centimeters','Position',[6 6 7.2 4]);%Set the size of figure(for A4)
        title(figureName,'Fontname', 'Arial');
        % save figure
        figurePath = ['signalProcess/poincare/',figureIdentity{iDof}];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close
    end
end

%% Part VII: 3D Axis Trajectory
if SwitchFigure.axisTrajectory3d
    refreshDirectory('signalProcess/axisTrajectory3d')
    xspan = zeros(nodeNum,size(q,2));
    yspan = xspan;
    dofInThisNode = 0;
    nodeNo = 1;
    for iDof=1:1:dofNum
        if nodeNo == dofOnNodeNo(iDof)
            dofInThisNode = dofInThisNode + 1;
        else
            nodeNo = nodeNo + 1;
            dofInThisNode = 1;
        end % end if
        
        if dofInThisNode  == 1
            xspan(nodeNo, :) =  q(iDof, :);
        elseif dofInThisNode == 2
            yspan(nodeNo, :) = q(iDof, :);
        end % end if
    end 
    yspan = yspan(:,tStartIndex:tEndIndex);
    xspan = xspan(:,tStartIndex:tEndIndex);
    
    nodeStart = 1;
    dis = Parameter.Mesh.keyPointsDistance;
    for iShaft = 1:1:Parameter.Shaft.amount
        nodeEnd = nodeStart + length(dis{iShaft}) - 1;
        figureName = ['AxisTrajectory ','Shaft-',num2str(iShaft)];
        ylabelname = '$V$ (m)';
        zlabelname = '$W$ (m)';
        xlabelname = '$l$ (m)';
        h = figure('Visible', 'off');
        for iNode = nodeStart:1:nodeEnd
            disSpan = dis{iShaft}(iNode - nodeStart + 1) * ones(1,size(xspan,2));
            plot3(disSpan, xspan(iNode,:), yspan(iNode,:),'-','LineWidth',0.5,'color',[0 0.30078125 0.62890625]); hold on
        end
        xlabel(xlabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',9);
        ylabel(ylabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',9);
        zlabel(zlabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',9);
        view([8.49766859099648 23.795098933806]);
        set(gcf,'Units','centimeters','Position',[6 6 13 5]);
        set(gca, ...
            'Box'         , 'on'                        , ...
            'LooseInset'  , get(gca,'TightInset')       , ...
            'TickDir'     , 'in'                        , ...
            'XMinorTick'  , 'off'                       , ...
            'YMinorTick'  , 'off'                       , ...
            'TickLength'  , [.01 .01]                   , ...
            'LineWidth'   , 0.5                         , ...
            'XGrid'       , 'on'                        , ...
            'YGrid'       , 'on'                        , ...
            'FontSize'    , 7                          , ... 
            'FontName'    ,'Times New Roman'            ) 
        title(figureName,'Fontname', 'Arial');
        legend off
        figurePath = ['signalProcess/axisTrajectory3d/',['Shaft-',num2str(iShaft)]];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close
        
        nodeStart = nodeEnd + 1;
    end
end


%% Part VIII: Poincare Section with Phase
if SwitchFigure.poincare_phase
    refreshDirectory('signalProcess/poincare_phase')

    shaftNum = Parameter.Shaft.amount;
    % get revolution start point
    tk = get_tk_from_simulation2(Parameter.Status, t(tStartIndex:tEndIndex), shaftNum);


    %Initialize to save data
    saveDisplacement = cell(shaftNum,1); 
    saveSpeed = cell(shaftNum,1);


    % calculate poincare point
    for iSection = 1:1:shaftNum
        q_section = interp1(t, q', tk{iSection});
        dq_section = interp1(t, dq', tk{iSection});
        saveDisplacement{iSection} = q_section';
        saveSpeed{iSection} = dq_section';
    end % end for iSection
    
    
    % plot
    xspan=saveDisplacement; % cell data
    yspan=saveSpeed;
    xspan2 = q(:,tStartIndex:tEndIndex);%Extract the x
    yspan2 = dq(:,tStartIndex:tEndIndex);%Extract the y
    for iDof=1:1:dofNum

        figureName = ['Poincare ',figureIdentity{iDof}];
        
        isBearing = Node(dofOnNodeNo(iDof)).isBearing;
        isTranslation = rem(iDof, 4)==1 || rem(iDof, 4)==2;
        if isBearing
            ylabelname = '$\dot{q}$ (m/s)';
            xlabelname = '$q$ (m)';
        elseif isTranslation
            ylabelname = '$\dot{q}$ (m/s)';
            xlabelname = '$q$ (m)';
        else
            ylabelname = '$\dot{q}$ (rad/s)';
            xlabelname = '$q$ (rad)';
        end
        
        legends = cell(shaftNum+1,1);
        legends{1} = 'Phase';

        h=figure('Visible', 'off');
        % plot phase
        plot(xspan2(iDof,:),yspan2(iDof,:)); hold on
        % plot poincare section
        for iSection = 1:1:shaftNum
            plot(xspan{iSection}(iDof,:),yspan{iSection}(iDof,:),'.'); hold on%plot
            legends{iSection+1} = ['Section ', num2str(iSection)];
        end
        hold off
        xMin = min(q(iDof,tStartIndex:tEndIndex))-0.15*abs(min(q(iDof,tStartIndex:tEndIndex))-max(q(iDof,tStartIndex:tEndIndex)));
        xMax = max(q(iDof,tStartIndex:tEndIndex))+0.15*abs(min(q(iDof,tStartIndex:tEndIndex))-max(q(iDof,tStartIndex:tEndIndex)));
        yMin = min(dq(iDof,tStartIndex:tEndIndex))-0.15*abs(min(dq(iDof,tStartIndex:tEndIndex))-max(dq(iDof,tStartIndex:tEndIndex)));
        yMax = max(dq(iDof,tStartIndex:tEndIndex))+0.15*abs(min(dq(iDof,tStartIndex:tEndIndex))-max(dq(iDof,tStartIndex:tEndIndex)));
        xlim([xMin xMax]);
        ylim([yMin yMax]);
        isUsedInA4 = NameValueArgs.isPlotInA4;
        isOnlySet = true;
        [~] = plot2DStandard([], [], xlabelname, ylabelname, isUsedInA4, isOnlySet);
        set(gcf,'Units','centimeters','Position',[6 6 7.2 4]);%Set the size of figure(for A4)
        title(figureName,'Fontname', 'Arial');
        legend(legends, 'Location', 'northeastoutside');
        % save figure
        figurePath = ['signalProcess/poincare_phase/',figureIdentity{iDof}];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close
    end % end for iDof (plot)

end % end if SwitchFigure.poincare_phase

%% 

% sub function 1
function refreshDirectory(pathName)
hasFolderSubFun = exist(pathName,'dir');
    if hasFolderSubFun
        rmdir(pathName,'s');
        mkdir(pathName);
    else
        mkdir(pathName);
    end
end

% sub function 2
function saveFigure(figHandle, figureName, isSaveFig, isVisible, isSaveEps)
if isSaveFig
    if isVisible
        set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
    end
    figName = [figureName, '.fig'];
    savefig(figHandle,figName,'compact')
end

if isSaveEps
    epsName = [figureName, '.eps'];
    print(figHandle, epsName, '-depsc2');
end

pngName = [figureName, '.png'];
print(figHandle, pngName, '-dpng', '-r400');

%saveas(figHandle, pngName) 
end % end subFunciton

end % end function