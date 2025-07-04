%% signalProcessing - Post-process rotor system response data and generate visualizations
%
% This function performs comprehensive post-processing of rotor system dynamic 
% response data, generating publication-quality visualizations for transient 
% and steady-state analysis of multi-shaft systems.
%
%% Syntax
%   signalProcessing(q, dq, t, Parameter, SwitchFigure)
%   signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan)
%   signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan, samplingFrequency, NameValueArgs)
%
%% Input Arguments
% * |q| - Displacement time history [n×m double]:
%   * n: Number of degrees of freedom (DOFs)
%   * m: Number of time samples
% * |dq| - Velocity time history [n×m double]
% * |t| - Time vector [1×m double]:
%   * Time points corresponding to q and dq (seconds)
% * |Parameter| - System configuration structure containing:
%   * |Mesh|: Discretization data
%     .nodeNum: Total node count
%     .dofOnNodeNo: DOF indices per node
%     .Node: Node properties array
%   * |Status|: Operational parameters
%     .vmax: Maximum rotational speed (rad/s)
% * |SwitchFigure| - Visualization control flags [struct]:
%   * |displacement|: Displacement time plots
%   * |axisTrajectory|: 2D shaft trajectory plots
%   * |phase|: Phase portraits
%   * |fftSteady|: Steady-state FFT
%   * |fftTransient|: Transient FFT/spectrogram
%   * |poincare|: Poincaré sections
%   * |saveFig|: Save .fig files
%   * |saveEps|: Save .eps files
%   * |axisTrajectory3d|: 3D shaft trajectories
%   * |poincare_phase|: Phase+Poincaré composite plots
% * |tSpan| - (Optional) Processing time range [1×2 double]:
%   * [start_time, end_time] in seconds
%   * Default: Full time range
% * |samplingFrequency| - (Optional) Original sampling rate [Hz]:
%   * Required for FFT visualizations
% * |NameValueArgs| - (Optional) Processing parameters [name-value pairs]:
%   * |fftXlim|: FFT frequency display limit (default: 500 Hz)
%   * |T_window|: STFT window duration (default: (tSpan(2)-tSpan(1))/20 s)
%   * |overlap|: STFT window overlap ratio (default: 2/3)
%   * |fftisPlot3DTransient|: 3D transient FFT visualization
%   * |fftSteadyLog|: Logarithmic FFT amplitude scale
%   * |reduceInterval|: Data downsampling factor
%   * |isPlotInA4|: A4 paper formatting
%   * |f|: Custom frequency vector for STFT
%
%% Description
% |signalProcessing| provides comprehensive visualization capabilities for 
% rotor system dynamics analysis:
% * Time-domain analysis: Displacement histories, velocity profiles
% * Phase space visualization: 2D/3D trajectories, Poincaré sections
% * Frequency-domain analysis: Steady-state FFT, STFT spectrograms
% * Multi-shaft support: Shaft-specific trajectory visualizations
% * Automated output management: Directory creation, file saving
%
%% Visualization Capabilities
% 1. Displacement Time History:
%    * Plots displacement vs. time for each DOF
%    * Automatic unit selection (m/rad) based on DOF type
% 2. Axis Trajectory:
%    * 2D: Cross-sectional shaft orbits
%    * 3D: Spatial shaft deformation along length
% 3. Phase Portraits:
%    * Velocity vs. displacement for each DOF
% 4. Spectral Analysis:
%    * Steady-state FFT: Frequency content of full signal
%    * Transient FFT: STFT spectrograms for time-frequency analysis
% 5. Poincaré Sections:
%    * State-space intersections at rotational periods
%    * Multi-shaft synchronization
% 6. Composite Plots:
%    * Phase portraits with Poincaré sections
%
%% Implementation Features
% 1. Intelligent DOF Handling:
%    * Automatic DOF labeling (Node-X-DOF-Y)
%    * Unit adaptation based on DOF type (translation/rotation)
% 2. Adaptive Processing:
%    * Time-range selection (tSpan)
%    * Data downsampling (reduceInterval)
% 3. Advanced Signal Processing:
%    * Short-Time Fourier Transform (STFT) with customizable windows
%    * Automated Poincaré section extraction at rotational periods
%    * Multi-resolution analysis
% 4. Publication-Quality Output:
%    * LaTeX-style annotations
%    * A4 paper formatting option
%    * Multi-format export (.fig, .eps, .png)
%
%% Examples
% % Basic usage with displacement and FFT plots
% SwitchFig = struct('displacement', true, 'fftSteady', true);
% signalProcessing(q, dq, t, sysParams, SwitchFig);
%
% % Transient analysis with custom STFT parameters
% signalProcessing(q, dq, t, sysParams, SwitchFig, [1 5], 2000, ...
%     'T_window', 0.2, 'overlap', 0.75, 'fftXlim', 1000);
%
% % Full 3D visualization suite
% SwitchFig = struct('axisTrajectory3d', true, 'poincare_phase', true);
% signalProcessing(q, dq, t, sysParams, SwitchFig);
%
%% Directory Structure
% Creates and manages the following output directories:
%   signalProcess/
%   ├── displacement/
%   ├── axisTrajectory/
%   ├── phase/
%   ├── fftSteady/
%   ├── fftTransient/
%   ├── poincare/
%   ├── axisTrajectory3d/
%   └── poincare_phase/
%
%% Algorithm Details
% 1. Time Range Selection:
%    * Automatically clips data to tSpan range
% 2. Poincaré Section Extraction:
%    * Uses |get_tk_from_simulation2| to find rotational periods
%    * Interpolates state variables at period boundaries
% 3. STFT Implementation:
%    * Uses MATLAB's |spectrogram| function
%    * Customizable window size and overlap
% 4. 3D Trajectory Reconstruction:
%    * Spatial mapping along shaft length
%
%% Application Notes
% 1. FFT Requirements:
%    * samplingFrequency must be provided for spectral analysis
% 2. Large Dataset Handling:
%    * Use reduceInterval for downsampling
%    * Select specific tSpan ranges
% 3. Publication Figures:
%    * Enable isPlotInA4 for standardized formatting
%    * Use saveEps for vector graphics
%
%% See Also
% get_tk_from_simulation2, spectrogram, plot2DStandard, fft
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


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
    dis = Parameter.Mesh.nodeDistance;
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

end % end subFunciton

end % end function