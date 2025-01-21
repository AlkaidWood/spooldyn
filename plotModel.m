%% plotModel
% plot the geometry diagram about the whole rotor model
%% Syntax
%  plotModel(InitialParameter)
%% Description
% InitialParameter including the whole geometry parameters of the rotor
% 
% The diagram for each shafts and the whole rotor wil be output in figures


function plotModel(InitialParameter)

Shaft = InitialParameter.Shaft;
Disk = InitialParameter.Disk;
Bearing = InitialParameter.Bearing;

%%

%calculate the offset of each shafts due to the intermediate bearing
offsetPosition = zeros(Shaft.amount,1);
if isfield(InitialParameter,'IntermediateBearing')
    InterBearing = InitialParameter.IntermediateBearing; % short the variable
    for iShaft = 1:1:Shaft.amount
        if iShaft ==1
            offsetPosition(1) = 0; % position of shaft 1 is reference
        else
            for iInterBearing = 1:1:InterBearing.amount
                if InterBearing.betweenShaftNo(iInterBearing,2) == iShaft
                   basicShaftD = InterBearing.positionOnShaftDistance(iInterBearing, 1);
                   laterShaftD = InterBearing.positionOnShaftDistance(iInterBearing, 2);
                   basicShaftNo = InterBearing.betweenShaftNo(iInterBearing,1);
                   offsetPosition(iShaft) = basicShaftD - laterShaftD...
                                            + offsetPosition(basicShaftNo);
                else
                   offsetPosition(iShaft) = 0;
                end % if InterBearing.betweenShaftNo(iInterBearing,2) == iShaft
            end % for iInterBearing = 1:1:InterBearing.amount
        end % if iShaft ==1
    end % for iShaft = 1:1:Shaft.amount 
end % if isfiled(InitialParameter,'IntermediateBearing')

%%

% gnerate folder to save figures
hasFolder = exist('modelDiagram','dir');
if hasFolder
    delete modelDiagram/*.fig;
    delete modelDiagram/*.png;
else
    mkdir('modelDiagram');
end

%%
%allFigure = cell(Shaft.amount,1);
figureName = cell(Shaft.amount,1);


for iShaft = 1:1:Shaft.amount
    h = figure('visible','off');
    
    
    % shaft
    positionX = Shaft.totalLength(iShaft)/2 + offsetPosition(iShaft);
    position = [positionX, 0, 0]; % [x, y, z]
    outerRadius = Shaft.outerRadius(iShaft);
    innerRadius = Shaft.innerRadius(iShaft);
    length = Shaft.totalLength(iShaft);
    NODES = 20;
    axisName = 'x';
    addCylinder(position, outerRadius, innerRadius, length, NODES, axisName);
    
    
    % disk
    for iDisk = 1:1:Disk.amount
        if Disk.inShaftNo(iDisk) == iShaft
            positionX = Disk.positionOnShaftDistance(iDisk)...
                        + offsetPosition(iShaft);
            position = [positionX, 0, 0]; % [x, y, z]
            outerRadius = Disk.outerRadius(iDisk);
            innerRadius = Disk.innerRadius(iDisk);
            length = Disk.thickness(iDisk);
            NODES = 30;
            axisName = 'x';
            addCylinder(position, outerRadius, innerRadius, length, NODES, axisName);
        end % end if
    end % end for iDsk
    
    
    % bearing
    for iBearing = 1:1:Bearing.amount
        if Bearing.inShaftNo(iBearing) == iShaft
            positionX = Bearing.positionOnShaftDistance(iBearing)...
                         + offsetPosition(iShaft);
            position = [positionX, 0, 0]; % [x, y, z]
            radius = Shaft.outerRadius(iShaft);
            height = max(Disk.outerRadius) * 1.25;
            width = height;
            thickness = min(Disk.thickness) * 0.6;
            NODES = 15;
            axisName = 'x';
            RotateInfo.isRotate = true;
            RotateInfo.oringin = [0,0,0];
            RotateInfo.direction = [1,0,0];
            RotateInfo.angle = 90;
            addTriangularBlock(position,radius,height,width,thickness,NODES,axisName,RotateInfo);     
        end % end if
    end % end for iBearing
    
    
    % save figure for each shaft
    figureName{iShaft} = ['modelDiagram/diagramOfShaft',num2str(iShaft),'.fig'];
    savefig(h,figureName{iShaft})
    pngName = ['modelDiagram/diagramOfShaft',num2str(iShaft),'.png'];
    saveas(h, pngName) 
    close(h)
    
end % end for iShaft


% save the whole figure
wholeFigure = CombFigs('theWholeModel',figureName(:));
savefig(wholeFigure,'modelDiagram/theWholeModel.fig')
saveas(wholeFigure,'modelDiagram/theWholeModel.png')
close(wholeFigure)
end