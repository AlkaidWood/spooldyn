%% plotModel - Visualize 3D geometry of multi-shaft rotor systems
%
% This function generates detailed 3D schematic diagrams of rotor systems,
% including shafts, disks, bearings, and intermediate bearings with
% automatic alignment based on system configuration.
%
%% Syntax
%   plotModel(InitialParameter)
%
%% Description
% |plotModel| creates comprehensive 3D visualizations of rotor systems:
% * Renders cylindrical shafts with inner/outer diameters
% * Displays disk geometries at specified positions
% * Visualizes bearing housings as triangular blocks
% * Automatically calculates intermediate bearing alignments
% * Generates both per-shaft and composite system diagrams
%
%% Input Arguments
% * |InitialParameter| - System configuration structure containing:
%   * |Shaft|: [1×1 struct]              % Shaft properties
%     .amount             % Number of shafts [scalar]
%     .totalLength        % Axial lengths [m] [N×1 vector]
%     .outerRadius        % Outer radii [m] [N×1 vector]
%     .innerRadius        % Inner radii [m] [N×1 vector]
%   * |Disk|: [1×1 struct]               % Disk parameters
%     .amount             % Number of disks [scalar]
%     .inShaftNo          % Parent shaft indices [M×1 vector]
%     .positionOnShaftDistance % Axial positions [m] [M×1 vector]
%     .outerRadius        % Disk radii [m] [M×1 vector]
%     .thickness          % Disk thicknesses [m] [M×1 vector]
%   * |Bearing|: [1×1 struct]            % Bearing parameters
%     .amount             % Number of bearings [scalar]
%     .inShaftNo          % Parent shaft indices [K×1 vector]
%     .positionOnShaftDistance % Axial positions [m] [K×1 vector]
%   * |IntermediateBearing|: [1×1 struct] % (Optional) Intermediate bearings
%     .amount             % Number of intermediate bearings [scalar]
%     .betweenShaftNo     % Connected shaft pairs [L×2 matrix]
%     .positionOnShaftDistance % Connection positions [m] [L×2 matrix]
%
%% Output
% Creates in './modelDiagram' directory:
% * |diagramOfShaft[n].fig|    % Individual shaft diagrams (MATLAB figures)
% * |diagramOfShaft[n].png|    % Individual shaft images
% * |theWholeModel.fig|        % Composite system diagram (MATLAB figure)
% * |theWholeModel.png|        % Composite system image
%
%% Visualization Features
% 1. Component Rendering:
%    * Shafts: Hollow cylinders with inner/outer diameters
%    * Disks: Solid cylinders with specified thickness
%    * Bearings: Triangular housing structures
% 2. Automatic Alignment:
%    * Calculates shaft position offsets from intermediate bearings
%    * Maintains geometric relationships between connected shafts
% 3. Lighting and Rendering:
%    * Dual directional lighting (cool/warm tones)
%    * Gouraud shading for smooth surfaces
% 4. Resolution Control:
%    * Shafts: 20 circumferential nodes
%    * Disks: 30 circumferential nodes
%    * Bearings: 15 surface nodes
%
%% Implementation Details
% 1. Offset Calculation:
%    * Computes shaft position adjustments based on intermediate bearings
%    * Maintains first shaft as reference (offset = 0)
% 2. Directory Management:
%    * Creates 'modelDiagram' directory if missing
%    * Clears previous outputs before generation
% 3. Visualization Pipeline:
%    a) Per-shaft figure creation
%    b) Component-by-component rendering:
%        - Shaft base cylinder
%        - Disk cylinders
%        - Bearing housings
%    c) Lighting and view configuration
%    d) Composite figure assembly
% 4. Composite Diagram:
%    * Combines all shafts into single visualization
%    * Preserves component colors and styles
%    * Enables full system inspection
%
%% Example
% % Configure and visualize rotor system
% rotorParams = inputEssentialParameter(); % Load system parameters
% plotModel(rotorParams);                   % Generate diagrams
% % View composite diagram
% winopen('modelDiagram/theWholeModel.png');
%
%% Component Rendering Notes
% * Shaft Dimensions:
%    Outer radius: Shaft.outerRadius
%    Inner radius: Shaft.innerRadius (for hollow shafts)
% * Disk Scaling:
%    Thickness: Disk.thickness
%    Radius: Disk.outerRadius
% * Bearing Housing:
%    Height: max(diskRadius×1.25, shaftRadius×2.5)
%    Width: Same as height
%    Thickness: min(diskThickness)×0.6
%
%% See Also
% addCylinder, addTriangularBlock, CombFigs, inputEssentialParameterBO
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