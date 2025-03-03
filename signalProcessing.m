%% signalProcessing
% process the signal calculated by calculateResponse.m
%% Syntax
% signalProcessing(q, dq, t, Parameter, SwitchFigure)
%
% signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan)
%
% signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan, samplingFrequency)
%% Description
% q is time history of displacemnet (2D matrix, node * tNum)
%
% dq is time history of velocity (2D matrix, node * tNum)
% 
% t is time series (row)
% 
% Parameter: is a struct saving the model data
%
% SwitchFigure: is a struct with fields: displacement, axisTrajector,
% phase, fftSteady, fftTransient, poincare, saveFig. all of these fieldes
% save the boolean data
%
% tSpan = [tStart, tEnd] denotes the start and end points where the signal
% would be processed (default: [t(1), t(end)])
%
% samplingFrequency:  is a integer equaling to 1/step (default = [ ])
%
% NameValue: 
%
% fftXlim, the left boundary of xTick in the fftSteady figure, default
% 3*Vmax/(2*pi)
%
% fftTimeInterval, the length of pre time piece in the fftTransient figure, 
% unit (s), default (tSpan(2) - tSpan(1)) / 20
%
% fftSuperpositionRatio, the superposition ratio of each time piece in the
% fftTransient figure, default 0.5
% 
% fftisPlot3DTransient, is a boolean contonling the kind of fftTransient
% figure
%
% reduceInterval: denotes the re-sampling interval (scaler)
% 
% isPlotInA4: is a boolean controling the size of figure
%
% fftSteadyLog: is a boolean controling the Y-axis (log)


function signalProcessing(q, dq, t, Parameter, SwitchFigure, tSpan, samplingFrequency, NameValueArgs)

arguments % name value pair
    q
    dq
    t
    Parameter
    SwitchFigure
    tSpan
    samplingFrequency 
    NameValueArgs.fftXlim = Parameter.Status.vmax/(2*pi) * 3; % (Hz)
    NameValueArgs.fftTimeInterval = (tSpan(2) - tSpan(1)) / 20; % (s)
    NameValueArgs.fftSuperpositionRatio = 0.5;
    NameValueArgs.fftisPlot3DTransient = true;
    NameValueArgs.fftSteadyLog = false;
    NameValueArgs.reduceInterval = 1;
    NameValueArgs.isPlotInA4 = false;
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
timeStart   = tSpan(1); 
timeEnd     = tSpan(2);
FINDERROR   = 0.00005;
tStartIndex = find(( (timeStart-FINDERROR)<t & t<(timeStart+FINDERROR) ),1); 
tEndIndex   = find(( (timeEnd-FINDERROR)<t & t<(timeEnd+FINDERROR) ),1); 

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
        %title(figureName,'Fontname', 'Arial');
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
        %title(figureName, 'Fontname', 'Arial');
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
        %title(figureName,'Fontname', 'Arial');
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
        %title(figureName,'Fontname', 'Arial');
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
    
    timeInterval = NameValueArgs.fftTimeInterval;
    superpositionRatio = NameValueArgs.fftSuperpositionRatio;
    tUse = t(:, tStartIndex:tEndIndex);
    % length of per divided area (index)
    intervalLength = floor(timeInterval * samplingFrequency/NameValueArgs.reduceInterval); 
    intervalLength = intervalLength - mod(intervalLength,2); % even number
    intervalNum = floor(((timeEnd-timeStart)-timeInterval)/(timeInterval*(1-superpositionRatio)));
    for iDof = 1:1:dofNum
        signal = q(iDof,tStartIndex:tEndIndex);
        
        % divide signal into pieces
        signalPieces  = zeros(intervalNum,intervalLength);
        tPieces       = zeros(intervalNum,intervalLength);
        startIndex  = 1;
        for iInterval=1:1:intervalNum
            tPieces(iInterval,:) = tUse(startIndex:startIndex+intervalLength-1);
            signalPieces(iInterval,:) = signal(startIndex:startIndex+intervalLength-1);
            startIndex = floor(startIndex+intervalLength*(1-superpositionRatio));%update locs
        end  
        % fft
        saveF  = zeros(intervalNum, intervalLength/2+1);
        saveP1 = zeros(intervalNum, intervalLength/2+1);
        saveT  = zeros(intervalNum, intervalLength/2+1);
        for iInterval = 1:1:intervalNum
            iSignalPiece = signalPieces(iInterval,:);
            iTPiece = tPieces(iInterval,:);
            signallength = size(tPieces,2);
            Y  = fft(iSignalPiece);%FFT to displacement
            P2 = abs(Y/signallength);%two-sided spectrum
            P1 = P2(1:signallength/2+1);
            P1(2:end-1) = 2*P1(2:end-1);%one-sided spectrum
            f  = samplingFrequency/NameValueArgs.reduceInterval...
                *(0:(signallength/2))/signallength;%frequency domain
            saveF(iInterval,:) = f;
            saveP1(iInterval,:) = P1;
            saveT(iInterval,:) = (max(iTPiece)+min(iTPiece))/2;
        end
        % plot
        h=figure('Visible', 'on');
        if NameValueArgs.fftisPlot3DTransient
            for iInterval = 1:1:intervalNum
                plot3(saveF(iInterval,:),saveT(iInterval,:),saveP1(iInterval,:),'k'); hold on
                %stem3(saveF(iInterval,:),saveT(iInterval,:),saveP1(iInterval,:),'color', 'k', 'Marker', 'none'); hold on
            end % end for
            view([-7.52762430939227 62.6971962616822]);
        else
            %construct my color
            mycolorpoint=[[0 0 16];...
                [8 69 99];...
                [57 174 156];...
                [198 243 99];...
                [222 251 123];...
                [239 255 190]];
            mycolorposition=[1 11 33 50 57 64];
            mycolormap_r=interp1(mycolorposition,mycolorpoint(:,1),1:64,'linear','extrap');
            mycolormap_g=interp1(mycolorposition,mycolorpoint(:,2),1:64,'linear','extrap');
            mycolormap_b=interp1(mycolorposition,mycolorpoint(:,3),1:64,'linear','extrap');
            mycolor=[mycolormap_r',mycolormap_g',mycolormap_b']/255;
            mycolor=round(mycolor*10^4)/10^4;%保留4位小数
            % plot
            h1 = pcolor(saveF,saveT,saveP1);
            shading flat
            colormap(turbo); % mycolor or jet
             h1.EdgeColor='none';
             h1.FaceColor='interp';
%             view([0,90])
            
        end % end if 
        figureName=['FFT ',figureIdentity{iDof}];
        xlabelname = 'Frequency (Hz)';
        ylabelname = 'Time (s)';
        zlabelname = '$|$P1$|$';
        xlabel(xlabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',10);
        ylabel(ylabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',10);
        zlabel(zlabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',10);
        xlim([0 NameValueArgs.fftXlim])
        %title(figureName,'Fontname', 'Arial');
        % change size
        h = change2dTransientFormat(h, NameValueArgs.fftXlim);
        % colorbar the order of these code can't move
        c = colorbar;
        ax = gca;
        prePosition = ax.Position;
        c.Position=[c.Position(1),c.Position(2),c.Position(3)*0.4,c.Position(4)];
        ax.Position = prePosition;
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
    
    
    %Initialize to save data
    saveDisplacement = cell(dofNum,1); 
    saveSpeed = cell(dofNum,1); 
    
    
    % calculate poincare point
    domega = Parameter.Status.vmax * [1; abs(Parameter.Status.ratio)]';
    Node = Parameter.Mesh.Node;
    dofOnShaftNo = [Node(dofOnNodeNo).onShaftNo];
    speed = 0;
    for iDof = 1:1:dofNum
        % check the speed
        if speed ~= domega(dofOnShaftNo(iDof))
            speed = domega(dofOnShaftNo(iDof));
            tCutPeriod = (2*pi)/speed;
            tCutPeriodIndex = floor(tCutPeriod*samplingFrequency/NameValueArgs.reduceInterval);
            tCutLoopNum = floor( (timeEnd-timeStart)/tCutPeriod )-1;
            tCutStartpoint = timeStart; % the start time for cutting signal (s) 
            tCutStartpointIndex = find(( (tCutStartpoint)<=t & t<(tCutStartpoint+0.0005) ),1); % find the index of time near the cut point
            [~, temporary] = max(  q(:,tCutStartpointIndex:(tCutStartpointIndex+tCutPeriodIndex)),[], 2 ); % find the maximum value near the start point
            temporary = temporary + tCutStartpointIndex - 1;
            tCutStartpointIndex = temporary; % the index of cuting time for each row
            tCutStartpoint = t(tCutStartpointIndex)'; % cut time for each row(displacement)
        end
        saveDisplacement{iDof} = zeros(tCutLoopNum,1); 
        saveSpeed{iDof} = zeros(tCutLoopNum,1); 
        for iLoop = 1:1:tCutLoopNum
            indexThisLoop = find((tCutStartpoint(iDof)+(iLoop-1)*tCutPeriod)<=t , 1);
            saveDisplacement{iDof}(iLoop) =  q(iDof,indexThisLoop);%t_cut_loop*dof
            saveSpeed{iDof}(iLoop) =  dq(iDof,indexThisLoop);%t_cut_loop*dof
        end % end for iLoop
    end % end for iDof
    
    
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
        plot(xspan{iDof},yspan{iDof},'.','color',[0 0.30078125 0.62890625]);%plot
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
        %title(figureName,'Fontname', 'Arial');
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
        legend off
        figurePath = ['signalProcess/axisTrajectory3d/',['Shaft-',num2str(iShaft)]];
        isVisible = true;
        saveFigure(h, figurePath, SwitchFigure.saveFig, isVisible, SwitchFigure.saveEps);
        close
        
        nodeStart = nodeEnd + 1;
    end
end

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