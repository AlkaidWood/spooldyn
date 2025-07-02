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
    figureName = ['Mesh Result of Shaft ', num2str(iShaft)];
    % line
    h = figure('name',figureName,'Visible', 'off');
    plotLine = plot([0 keyPointsDistance{iShaft}(end)] , [0 0]); hold on
    plotLine.LineWidth = 3;
    plotLine.Color = '#5493BA';
    
    
    % key points
    plotKeyPoints = scatter(keyPointsDistance{iShaft},...
                            zeros(length(keyPointsDistance{iShaft}), 1)); hold on
    plotKeyPoints.SizeData = 45;
    plotKeyPoints.MarkerFaceColor = '#CA3636';
    plotKeyPoints.Marker = 'o';
    
    
    % nodes
    nodeWithoutKeyPoints = setdiff(nodeDistance{iShaft}, keyPointsDistance{iShaft});
    plotNodes = scatter(nodeWithoutKeyPoints,...
                        zeros(length(nodeWithoutKeyPoints), 1)); hold on
    plotNodes.SizeData = 45;
    plotNodes.MarkerFaceColor = '#000000';
    plotNodes.MarkerEdgeColor = '#000000';
    plotNodes.Marker = '|';

    
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
    text(xText,yText,nodeName, 'HorizontalAlignment', 'center')
     
    
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
    text(xText,yText,elementName, 'HorizontalAlignment', 'center',...
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
        text(xText,yText,nodeNameB{iSegment}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'cap')
        % plot mark
        plotBearingNodes = scatter(xMark,yMark); hold on
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
    h.Units = 'centimeters';
    h.Position = [2 8 38 6+(max([1;noInColumnRecorder])-1)*0.8];
    set(gca,'LooseInset',[0.01,0.01,0.01,0.01]);
    
    
    % save figure
    set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
    figureName2 = ['meshDiagram/MeshResultOfShaft', num2str(iShaft), '.fig'];
    savefig(h,figureName2)
    saveas(h, ['meshDiagram/MeshResultOfShaft', num2str(iShaft), '.png'])
    close(h)
end % end for iShaft

end % end function