%% PLOTMESH - Visualize FEM mesh discretization results
% Generates schematic diagrams of mesh configuration with component annotations.
%
%% Syntax
%   plotMesh(Parameter)
%
%% Description
% |PLOTMESH| creates visualization of FEM mesh discretization showing:
% * Key node locations (bearings/disks/special components)
% * Mesh node distribution
% * Component-node relationships
% * Bearing type markers
%
% Outputs include:
% * Per-shaft mesh diagrams (.fig/.png)
% * Automatic directory creation ('meshDiagram')
%
%% Input Arguments
% *Parameter* - System configuration structure containing:
%   .Mesh               % Mesh data from |meshModel|
%     .keyPointsDistance% [N×1 cell] Key node positions per shaft
%     .nodeDistance     % [N×1 cell] Node positions per shaft  
%     .Node             % [M×1 struct] Node properties:
%       .name           % Node ID
%       .onShaftNo      % Parent shaft index
%       .onShaftDistance% Axial position [m]
%       .isBearing      % Bearing node flag
%   .Shaft              % Shaft geometry parameters
%   .ComponentSwitch   % Active component flags
%
%% Output
% Generates in './meshDiagram' directory:
% * MeshResultOfShaft[n].fig/png  % Individual shaft mesh diagrams
%
%% Visualization Features
% 1. Differentiated markers for:
%    - Key nodes (red circles)
%    - Regular nodes (black ticks)
%    - Bearing nodes (purple markers)
% 2. Component annotations:
%    - Disk locations (D#)
%    - Bearing types (B#/B# Loose)  
%    - Intermediate bearings (IB#)
%    - Rub impacts (Rub#)
% 3. Automatic layout adjustment for multi bearings
%
%% Example
% % Generate and view mesh diagrams
% Params = meshModel(baseParams);
% plotMesh(Params); 
% winopen('meshDiagram/MeshResultOfShaft1.png');
%
%% See Also
% meshModel, plotModel, establishModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function plotMesh(Parameter)

Shaft = Parameter.Shaft;
keyPointsDistance = Parameter.Mesh.keyPointsDistance;
nodeDistance = Parameter.Mesh.nodeDistance;
Node = Parameter.Mesh.Node;

%%

% gnerate folder to save figures
hasFolder = exist('meshDiagram','dir');
if hasFolder
    delete meshDiagram/*.fig;
    delete meshDiagram/*.png;
else
    mkdir('meshDiagram');
end


%%
% plot the key points on each shaft
for iShaft = 1:1:Shaft.amount
    % create figure
    figureName = ['Mesh Result of Shaft ', num2str(iShaft)];
    h = figure('name',figureName,'Visible', 'off');
    
    
    
    % create axes (main axes + annotation axes)
    mainAx = axes();

    % line
    plotLine = plot(mainAx, [0 keyPointsDistance{iShaft}(end)] , [0 0]); hold on
    plotLine.LineWidth = 3;
    plotLine.Color = '#5493BA';
    
    
    % key points
    plotKeyPoints = scatter(mainAx, keyPointsDistance{iShaft},...
                            zeros(length(keyPointsDistance{iShaft}), 1)); hold on
    plotKeyPoints.SizeData = 45;
    plotKeyPoints.MarkerFaceColor = '#CA3636';
    plotKeyPoints.Marker = 'o';
    
    
    % nodes
    nodeWithoutKeyPoints = setdiff(nodeDistance{iShaft}, keyPointsDistance{iShaft});
    plotNodes = scatter(mainAx, nodeWithoutKeyPoints,...
                        zeros(length(nodeWithoutKeyPoints), 1)); hold on
    plotNodes.SizeData = 45;
    plotNodes.MarkerFaceColor = '#000000';
    plotNodes.MarkerEdgeColor = '#000000';
    plotNodes.Marker = '|';
    plotNodes.LineWidth = 1.5;

    
    % find the nodes locating on iShaft (shaft node)
    nodeNum = length(Node);
    condition1 = zeros(1,nodeNum);
    for iNode = 1:1:nodeNum
        condition1(iNode) = ismember(iShaft,Node(iNode).onShaftNo);
    end 
    condition = condition1 & ([Node.isBearing] == false);
    NodeSegment = Node( condition ); % nodes on iShaft
    segmentNum = length(NodeSegment);
    
    
    % mark node name
    nodeName = cell(1,segmentNum);
    for iSegment = 1:1:segmentNum
            nodeName{iSegment} = num2str( NodeSegment(iSegment).name );
    end
    xText = [NodeSegment.onShaftDistance];
    yText = -0.2 * ones(1,segmentNum);
    text(mainAx, xText,yText,nodeName, 'HorizontalAlignment', 'center')
     
    
    % mark elements
    elementName = cell(1,segmentNum);
    for iSegment = 1:1:segmentNum
        % disk
        diskName = [];
        if ~isempty(NodeSegment(iSegment).diskNo)
            diskName = ['D ',num2str(NodeSegment(iSegment).diskNo)];
        end
        
        % bearing and loosing bearing
        bearingName = [];
        if ~isempty(NodeSegment(iSegment).bearingNo)
            
            if Parameter.ComponentSwitch.hasLoosingBearing
                isLoosing = NodeSegment(iSegment).isLoosingBearing;
                if isLoosing
                    bearingName = ['B ',num2str(NodeSegment(iSegment).bearingNo),' Loose'];
                else
                    bearingName = ['B ',num2str(NodeSegment(iSegment).bearingNo)];
                end % end if isLoosing
            else
                bearingName = ['B ',num2str(NodeSegment(iSegment).bearingNo)];
            end % end if hasLoosingBearing            
        end % end if hasBearing
          
        % intermediate bearing
        interBearingName = [];
        if Parameter.ComponentSwitch.hasIntermediateBearing
            if ~isempty(NodeSegment(iSegment).interBearingNo)
                interBearingName = ['IB ',num2str(NodeSegment(iSegment).interBearingNo)];
            end
        end
          
        % rub
        rubImpactName = [];
        if Parameter.ComponentSwitch.hasRubImpact
            if ~isempty(NodeSegment(iSegment).rubImpactNo)
                rubImpactName = ['Rub ',num2str(NodeSegment(iSegment).rubImpactNo)];
            end
        end
         
        % coupling misalignment
        couplingMisName = [];
        if Parameter.ComponentSwitch.hasCouplingMisalignment
            if ~isempty(NodeSegment(iSegment).couplingNo)
                couplingMisName = ['CpMis ',num2str(NodeSegment(iSegment).couplingNo)];
            end
        end

        % custom node
        customName = [];
        if Parameter.ComponentSwitch.hasCustom
            if ~isempty(NodeSegment(iSegment).customNo)
                customName = ['Cus ',num2str(NodeSegment(iSegment).customNo)];
            end
        end
           
        % assembling elementNaeme
        elementName{iSegment} = {   customName;...
                                    couplingMisName;...
                                    rubImpactName;...
                                    interBearingName;...
                                    bearingName;...
                                    diskName};
                                
        % delete the [ ] value
        nullIndex = cellfun(@isempty, elementName{iSegment});
        notNullIndex = ~nullIndex;
        elementName{iSegment} = elementName{iSegment}(notNullIndex);
    end % end for iSegment = 1:1:segmentNum
    xText = [NodeSegment.onShaftDistance];
    yText = 0.12 * ones(1,segmentNum);
    text(mainAx, xText,yText,elementName, 'HorizontalAlignment', 'center',...
         'VerticalAlignment', 'bottom');
    
     
    % mark bearingNode
    % find the nodes locating on iShaft (bearing node)
    condition1 = zeros(1,nodeNum);
    for iNode = 1:1:nodeNum
        condition1(iNode) = ismember(iShaft,Node(iNode).onShaftNo);
    end 
    condition = condition1 & ([Node.isBearing] == true);
    NodeSegmentB = Node( condition ); % nodes on iShaft
    segmentNumB = length(NodeSegmentB);
    nodeNameB = cell(1,segmentNumB); % mark bearing node name
    posRecorder = zeros(segmentNumB,1);
    noInColumnRecorder = zeros(segmentNumB,1);
    for iSegment = 1:1:segmentNumB
        nodeNameB{iSegment} = num2str( NodeSegmentB(iSegment).name ); % save node name
        % judge type of bearing
        condition1 = Parameter.ComponentSwitch.hasIntermediateBearing; % has InterBearing -> 1; or not ->0
        condition2 = ~isempty(NodeSegmentB(iSegment).interBearingNo); % is InterBearing -> 1; is not -> 0
        isInterBearing = condition1 && condition2;
        if isInterBearing
            shaftIndex = find(NodeSegmentB(iSegment).onShaftNo == iShaft); % find which shaft the InterBearing locating
            xText = NodeSegmentB(iSegment).onShaftDistance(shaftIndex); % calculate x positon of node name
        else
            xText = NodeSegmentB(iSegment).onShaftDistance;
        end
        % calculate the y-positon of this node to
        if iSegment==1
            noInColumn = 1; % to decide the y-position of this node
        else
            previousNum = length( find(posRecorder(1:iSegment-1)==xText) );
            noInColumn = previousNum + 1;
        end
        posRecorder(iSegment) = xText; % refresh the Recorder
        noInColumnRecorder(iSegment) = noInColumn;
        yText = -0.4 - (noInColumn-1)*0.25;
        xMark = xText;
        yMark = yText;
        % plot node name
        text(mainAx, xText,yText,nodeNameB{iSegment}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'cap')
        % plot mark
        plotBearingNodes = scatter(mainAx, xMark,yMark); hold on
        plotBearingNodes.SizeData = 45;
        plotBearingNodes.MarkerFaceColor = '#7E2F8E';
        plotBearingNodes.MarkerEdgeColor = '#7E2F8E';
        if isInterBearing
            plotBearingNodes.Marker = 'd';
        else
            plotBearingNodes.Marker = '^';
        end
    end

     
    % set axis, title
    xlim([0-0.1*Shaft.totalLength(iShaft), Shaft.totalLength(iShaft)*1.1])
    ylim([-0.8-(max([1;noInColumnRecorder])-1)*0.25, 0.8])
    title(figureName)
    % define some constants
    main_height = 6+(max([1;noInColumnRecorder])-1)*0.8; % cm
    bottom_height = 1.6; % cm
    gap_height = 1; %cm
    title_height = 0.8; % cm
    % set figure positon
    h.Units = 'centimeters';
    h.Position = [2 8 38 main_height+bottom_height+gap_height+title_height];
    % set main axes position
    mainAx.Units = 'centimeters';
    mainAx.Position = [1, bottom_height+gap_height, 36.5, main_height];
    % set legend axes position
    bottomAx = axes();
    bottomAx.Units = 'centimeters';
    bottomAx.Position = [1, 0.2, 36.5, bottom_height];
    bottomAx.XTick = [];
    bottomAx.YTick = [];
    bottomAx.XTickLabel = [];
    bottomAx.YTickLabel = [];
    bottomAx.XColor = 'none';
    bottomAx.YColor = 'none';
    bottomAx.Color = 'none';
    bottomAx.XLim = [0, 1];
    bottomAx.YLim = [0, 1];
    hold(bottomAx, 'on');

    % judge bearing type
    condition1 = [Node.isBearing];
    if isfield(Node, 'interBearingNo')
        n = numel(Node);
        values = cell(n, 1);
        for i = 1:n
            values{i} = Node(i).interBearingNo;
        end
        boolVector = double(~cellfun(@isempty, values));
        condition2 = boolVector(:)';
    else
        condition2 = boolean(zeros(size(condition1)));
    end
    hasOrdinaryBearing = sum(condition1 & ~condition2);
    hasInterBearing = sum(condition1 & condition2);

    % add here
    % 确定文字图例项
    textLegends = {};
    % 检查各组件是否存在
    hasDisk = any(~cellfun(@isempty, {NodeSegment.diskNo}));
    hasBearing = any(~cellfun(@isempty, {NodeSegment.bearingNo}));
    hasLoosingBearing = Parameter.ComponentSwitch.hasLoosingBearing;
    hasInterBearingText = Parameter.ComponentSwitch.hasIntermediateBearing;
    hasRubImpact = Parameter.ComponentSwitch.hasRubImpact;
    hasCouplingMis = Parameter.ComponentSwitch.hasCouplingMisalignment;
    hasCustom = Parameter.ComponentSwitch.hasCustom;
    
    % 添加存在的文字图例
    if hasDisk
        textLegends{end+1} = 'D: Disk';
    end
    if hasBearing
        textLegends{end+1} = 'B: Bearing';
    end
    if hasInterBearingText
        textLegends{end+1} = 'IB: Inter-shaft Bearing';
    end
    if hasCustom
        textLegends{end+1} = 'Cus: Customize Force';
    end
    if hasLoosingBearing
        textLegends{end+1} = 'B Loose: Loosening Bearing';
    end
    if hasRubImpact
        textLegends{end+1} = 'Rub: Rub Impact';
    end
    if hasCouplingMis
        textLegends{end+1} = 'CpMis: Coupling Misalignment';
    end

    % intergral all text
    full_text = strjoin(textLegends, '       ');

    
    % 绘制几何图例 (第一行)
    xlim(bottomAx, [-1,12])
    xPositions = 0;
    x_pos_gap = 1.5;
    x_text_gap = 0.1;
    yPos = 0.75; % 第一行y位置

    % 绘制关键节点图例
    s_key_point = scatter(bottomAx, xPositions, yPos);
    s_key_point.SizeData = 45;
    s_key_point.MarkerFaceColor = '#CA3636';
    s_key_point.Marker = 'o';
    s_key_point.MarkerEdgeColor = "none";
    text(bottomAx, xPositions+x_text_gap, yPos, 'Key Node', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'middle');
    xPositions = xPositions + x_pos_gap;

    % 绘制普通节点图例
    if ~isempty(nodeWithoutKeyPoints)
        s_node = scatter(bottomAx, xPositions, yPos);
        s_node.SizeData = 45;
        s_node.MarkerFaceColor = '#000000';
        s_node.MarkerEdgeColor = '#000000';
        s_node.Marker = '|';
        s_node.LineWidth = 1.5;
        text(bottomAx, xPositions+x_text_gap, yPos, 'Node', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'middle');
        xPositions = xPositions + x_pos_gap;
    end

    % 绘制普通轴承图例
    if hasOrdinaryBearing
        s_beairng = scatter(bottomAx, xPositions,yPos);
        s_beairng.SizeData = 45;
        s_beairng.MarkerFaceColor = '#7E2F8E';
        s_beairng.MarkerEdgeColor = '#7E2F8E';
        s_beairng.Marker = '^';
        text(bottomAx, xPositions+x_text_gap, yPos, 'Mass at Bearing', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'middle');
        xPositions = xPositions + x_pos_gap + 0.5;
    end
    
    % 绘制中介轴承图例
    if hasInterBearing
        s_interbeairng = scatter(bottomAx, xPositions,yPos);
        s_interbeairng.SizeData = 45;
        s_interbeairng.MarkerFaceColor = '#7E2F8E';
        s_interbeairng.MarkerEdgeColor = '#7E2F8E';
        s_interbeairng.Marker = 'd';
        text(bottomAx, xPositions+x_text_gap, yPos, 'Mass at Inter-shaft Bearing', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'middle');
        xPositions = xPositions + x_pos_gap;
    end
    
    
    
    % 绘制文字图例 (第二行)
    xPositionsText = 0;
    yPosText = 0.25; % 第二行y位置
    
    text(bottomAx, xPositionsText, yPosText, full_text, ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle');

    % save figure
    set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
    figureName2 = ['meshDiagram/MeshResultOfShaft', num2str(iShaft), '.fig'];
    savefig(h,figureName2)
    saveas(h, ['meshDiagram/MeshResultOfShaft', num2str(iShaft), '.png'])
    close(h)
end % end for iShaft

end % end function