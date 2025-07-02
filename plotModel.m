%% PLOTMODEL - Visualize rotor system geometry
% Generates 3D schematic diagrams of multi-shaft rotor systems with components.
%
%% Syntax
%   plotModel(InitialParameter)
%
%% Description
% |PLOTMODEL| creates visualization of rotor system components including:
% * Shaft geometry with inner/outer radii
% * Disk positions and dimensions
% * Bearing locations and housing structures
% * Intermediate bearing alignment offsets
%
% Outputs include:
% * Individual shaft diagrams (.fig/.png)
% * Composite system diagram (.fig/.png)
% * Automatic directory creation ('modelDiagram')
%
%% Input Arguments
% *InitialParameter* - System configuration structure containing:
%   .Shaft              % Shaft properties (struct array)
%     .totalLength      % [N×1] Axial lengths [m]
%     .outerRadius      % [N×1] Outer radii [m]
%     .innerRadius      % [N×1] Inner radii [m]
%   .Disk               % Disk parameters (struct array)
%     .inShaftNo        % [M×1] Parent shaft indices
%     .positionOnShaftDistance % [M×1] Axial positions [m]
%     .outerRadius      % [M×1] Disk radii [m]
%     .thickness        % [M×1] Disk thicknesses [m]
%   .Bearing            % Bearing parameters (struct array)
%     .inShaftNo        % [K×1] Parent shaft indices
%     .positionOnShaftDistance % [K×1] Axial positions [m]
%   .IntermediateBearing % Intermediate bearing parameters (optional)
%     .betweenShaftNo   % [L×2] Connected shaft indices
%     .positionOnShaftDistance % [L×2] Connection positions [m]
%
%% Output
% Generates in './modelDiagram' directory:
% * diagramOfShaft[n].fig/png    % Individual shaft visualizations
% * theWholeModel.fig/png         % Composite system diagram
%
%% Visualization Features
% 1. Cylindrical shaft/disk representations
% 2. Triangular bearing housing models
% 3. Automatic intermediate bearing alignment:
%    - Calculates shaft position offsets
% 4. Multi-resolution component rendering:
%    - 20 nodes for shafts
%    - 30 nodes for disks
%    - 15 nodes for bearings
%
%% Example
% % Generate and view system diagrams
% sysParams = inputEssentialParameterBO();
% plotModel(sysParams);
% winopen('modelDiagram/theWholeModel.png');
%
%% See Also
% addCylinder, addTriangularBlock, CombFigs
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


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

% Create persistent figure for final composite
wholeFig = figure('Visible', 'off');
wholeAxes = axes(wholeFig);
hold(wholeAxes, 'on');

% Process each shaft
figureName = cell(Shaft.amount,1);


for iShaft = 1:1:Shaft.amount
    h = figure('visible','off');
    ax_h = axes(h);
    
    % shaft
    positionX = Shaft.totalLength(iShaft)/2 + offsetPosition(iShaft);
    position = [positionX, 0, 0]; % [x, y, z]
    outerRadius = Shaft.outerRadius(iShaft);
    innerRadius = Shaft.innerRadius(iShaft);
    length = Shaft.totalLength(iShaft);
    NODES = 20;
    axisName = 'x';
    addCylinder(ax_h, position, outerRadius, innerRadius, length, NODES, axisName);
    
    
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
            addCylinder(ax_h, position, outerRadius, innerRadius, length, NODES, axisName);
        end % end if
    end % end for iDsk
    
    
    % bearing
    for iBearing = 1:1:Bearing.amount
        if Bearing.inShaftNo(iBearing) == iShaft
            positionX = Bearing.positionOnShaftDistance(iBearing)...
                         + offsetPosition(iShaft);
            position = [positionX, 0, 0]; % [x, y, z]
            radius = Shaft.outerRadius(iShaft);
            height_with_disk = max(Disk.outerRadius) * 1.25;
            height_with_shaft = max(Shaft.outerRadius) * 2.5;
            if height_with_disk >= height_with_shaft
                height = height_with_disk;
            else
                height = height_with_shaft;
            end % end if
            width = height;
            thickness = min(Disk.thickness) * 0.6;
            NODES = 15;
            axisName = 'x';
            RotateInfo.isRotate = true;
            RotateInfo.oringin = [0,0,0];
            RotateInfo.direction = [1,0,0];
            RotateInfo.angle = 90;
            addTriangularBlock(ax_h, position,radius,height,width,thickness,NODES,axisName,RotateInfo);     
        end % end if
    end % end for iBearing
    
    % Copy contents to composite figure
    allObjs = findall(h, 'type','axes');
    toCopy = allchild(allObjs);
    % Copy objects to composite figure
    copyobj(toCopy, wholeAxes);

    % add light
    light(ax_h, 'Position', [-0.5272   -0.6871    0.5000], 'Color', [0.8 0.8 1]);
    light(ax_h, 'Position', [-0.5272   -0.6871    0.5000], 'Color', [1 0.9 0.8]);
    lighting gouraud;
    
    % save figure for each shaft
    set(h,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
    figureName{iShaft} = ['modelDiagram/diagramOfShaft',num2str(iShaft),'.fig'];
    savefig(h,figureName{iShaft})
    pngName = ['modelDiagram/diagramOfShaft',num2str(iShaft),'.png'];
    saveas(h, pngName)
    close(h)
    
end % end for iShaft

% Set axis properties for full model
view(wholeAxes, 3); % 3D view
grid(wholeAxes, 'on');
axis(wholeAxes, 'equal');

% add light for full model
light(wholeAxes, 'Position', [-0.5272   -0.6871    0.5000], 'Color', [0.8 0.8 1]);
light(wholeAxes, 'Position', [-0.5272   -0.6871    0.5000], 'Color', [1 0.9 0.8]);
lighting gouraud;

% Save composite figure
set(wholeFig,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
savefig(wholeFig, 'modelDiagram/theWholeModel.fig');
saveas(wholeFig, 'modelDiagram/theWholeModel.png');

% Close composite figure
close(wholeFig);

% % save the whole figure
% wholeFigure = CombFigs('theWholeModel',figureName(:));
% savefig(wholeFigure,'modelDiagram/theWholeModel.fig')
% saveas(wholeFigure,'modelDiagram/theWholeModel.png')
% close(wholeFigure)
end