%% meshModel - Generate discretized finite element mesh for multi-shaft rotor systems
%
% This function creates a comprehensive nodal mesh structure for finite element
% analysis of rotor-bearing systems with complex configurations. It handles
% automatic mesh generation, key component identification, and DOF mapping.
%
%% Syntax
%   Parameter = meshModel(InitialParameter)
%   Parameter = meshModel(InitialParameter, gridFineness)
%   Parameter = meshModel(InitialParameter, manualGrid)
%
%% Description
% |meshModel| performs sophisticated mesh generation for rotor dynamics models:
% * Automatically detects critical components (bearings, disks, etc.)
% * Implements adaptive mesh refinement strategies
% * Supports custom mesh specifications
% * Establishes node-component relationships
% * Manages degree-of-freedom (DOF) allocation
%
%% Input Arguments
% * |InitialParameter| - System configuration structure containing:
%   * |Shaft|: Shaft properties [struct array]
%     .totalLength         % Length of each shaft [m] [nShafts×1]
%     .dofOfEachNodes      % DOFs per node [scalar or nShafts×1]
%   * |Disk|: Disk parameters [struct array]
%     .inShaftNo           % Parent shaft index [mDisks×1]
%     .positionOnShaftDistance % Axial position [m] [mDisks×1]
%   * |Bearing|: Bearing parameters [struct array]
%     .inShaftNo            % Parent shaft index [kBearings×1]
%     .positionOnShaftDistance % Axial position [m] [kBearings×1]
%   * |ComponentSwitch|: Component activation flags [struct]
%     .hasRubImpact         % Rub-impact activation [logical]
%     .hasIntermediateBearing % Intermediate bearing flag [logical]
%     .hasCouplingMisalignment % Coupling misalignment flag [logical]
%     .hasLoosingBearing   % Bearing clearance flag [logical]
%     .hasCustom           % Custom component flag [logical]
%   * see also inputEssentialParameter(), inputBearingHertz(), inputInmediateBearing()
%
% * |gridFineness| - Mesh resolution specification [string]:
%   * |'low'|: Coarse mesh (default, 1 element between key nodes)
%   * |'middle'|: Medium mesh (4 elements between key nodes)
%   * |'high'|: Fine mesh (10 elements between key nodes)
%
% * |manualGrid| - Custom mesh definition [cell array]:
%   {n} = [e1 e2 ... em]  % Element counts per segment (divided by key nodes) for shaft n
%
%% Output Structure
% * |Parameter| - Enhanced system configuration with mesh data:
%   * |Mesh|: Discretization results [struct]
%     .nodeDistance        % Node positions per shaft [1×nShafts cell]
%     .Node                % Node properties [nNodes×1 struct]:
%       .name              % Node ID [integer]
%       .onShaftNo         % Parent shaft index [integer]
%       .onShaftDistance   % Axial position [m]
%       .dof               % DOF count at node [integer]
%       .diskNo            % Associated disk ID [integer]
%       .bearingNo         % Associated bearing ID [integer]
%       .isBearing         % Bearing node flag [logical]
%       ... (additional component fields as applicable)
%     .dofInterval        % DOF ranges [nNodes×2]
%     .dofOnNodeNo         % Node mapping for DOFs [nDOFs×1]
%     .nodeNum             % Total node count [integer]
%     .dofNum              % Total DOF count [integer]
%   * Updated component fields with node mappings:
%     .Disk.positionOnShaftNode
%     .Bearing.positionOnShaftNode
%     .Bearing.positionNode
%     ... (other components)
%
%% Key Algorithms
% 1. Key Node Identification:
%    * Collects critical positions: shaft ends, disks, bearings, etc.
%    * Implements proximity merging (L/5000 threshold)
% 2. Mesh Segmentation:
%    Automatic:
%      * Uniform segmentation based on gridFineness level
%    Manual:
%      * Custom element counts per segment
% 3. Node-Component Mapping:
%    * Associates physical components with nearest node
%    * Handles multi-shaft configurations
% 4. DOF Management:
%    * Creates DOF intervals for each node
%    * Maintains DOF-node relationships
% 5. Special Component Handling:
%    * Generates additional nodes for bearing masses
%    * Flags loose bearing positions
%
%% Example
% % Medium-resolution automatic mesh
% sysConfig = meshModel(baseParams, 'middle');
%
% % Custom mesh for dual-shaft system
% manualGrid = {[3 2 4], [5 1]}; % Shaft1: 3|2|4 elements, Shaft2: 5|1
% sysConfig = meshModel(baseParams, manualGrid);
%
% % Basic analysis mesh
% sysConfig = meshModel(baseParams);
%
%% Implementation Notes
% * Node Merging Threshold: L/5000 (L = shaft length)
% * Component Mapping:
%    - Disks: Point components at nodes
%    - Bearings: May generate additional mass nodes
%    - Intermediate Bearings: Special two-column mapping
% * DOF Allocation:
%    - Sequential DOF numbering across all nodes
%    - Maintains DOF interval tracking
%
%% See Also
% establishModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function Parameter = meshModel(varargin)   
% define the constant: this function will merge key nodes whose distance
% from other nodes to itself < totalLength/THRESHOLD_COEFFICIENT
THRESHOLD_COEFFICIENT = 5000; 

% recognize the input parameter
if isempty(varargin)
    error('Insufficient input parameters')
else
    InitialParameter = varargin{1};
end

switch length(varargin)
    case 1
        isAutoMesh = true;
        gridFineness = 'low';
    case 2
        if ischar(varargin{2})
            gridFineness = varargin{2};
            isAutoMesh = true;
        elseif iscell(varargin{2})
            manualGrid = varargin{2};
            isAutoMesh =false;
        else
            error('Please input char or cell data for the second parameter')
        end
    otherwise 
        error('too much input parameter')
end

%%

% check the initial parameter
Shaft = InitialParameter.Shaft;
Disk = InitialParameter.Disk;
Bearing = InitialParameter.Bearing;

if InitialParameter.ComponentSwitch.hasRubImpact
    RubImpact = InitialParameter.RubImpact;
    hasRub = true;
else
    hasRub = false;
end

if InitialParameter.ComponentSwitch.hasIntermediateBearing
    InterBearing = InitialParameter.IntermediateBearing;
    hasInterBearing = true;
else
    hasInterBearing = false;
end

if InitialParameter.ComponentSwitch.hasCouplingMisalignment
    Coupling = InitialParameter.CouplingMisalignment;
    hasCoupling = true;
else
    hasCoupling = false;
end

if InitialParameter.ComponentSwitch.hasLoosingBearing
    LoosingBearing = InitialParameter.LoosingBearing;
    hasLoosingBearing = true;
else
    hasLoosingBearing = false;
end

if InitialParameter.ComponentSwitch.hasCustom
    Custom = InitialParameter.Custom;
    hasCustom = true;
else
    hasCustom = false;
end

%%

% record key points on the shaft
keyPoints = cell(Shaft.amount,1);

for iShaft = 1:1:Shaft.amount
    keyPoints{iShaft} = [0; Shaft.totalLength(iShaft)];
    
    
    % record disk
    position = find(Disk.inShaftNo == iShaft);
    keyPointsDisk = Disk.positionOnShaftDistance(position);
    
    
    % record bearing 
    position = find(Bearing.inShaftNo == iShaft);
    keyPointsBearing = Bearing.positionOnShaftDistance(position);
    
    % record rub
    if hasRub
        position = find(RubImpact.inShaftNo == iShaft);
        keyPointsRub = RubImpact.positionOnShaftDistance(position);
    else
        keyPointsRub = [];
    end
    
    
    % record intermediate bearing
    if hasInterBearing
        position = find(InterBearing.betweenShaftNo == iShaft);
        keyPointsInterBearing = InterBearing.positionOnShaftDistance(position);
    else
        keyPointsInterBearing = [];
    end

    % record Customize function
    if hasCustom
        position = find(Custom.inShaftNo == iShaft);
        keyPointsCustom = Custom.positionOnShaftDistance(position);
    else
        keyPointsCustom = [];
    end
    
    
    % assembling
    keyPoints{iShaft} = sort([  keyPoints{iShaft};...
                                keyPointsDisk;...
                                keyPointsRub;...
                                keyPointsInterBearing;...
                                keyPointsBearing; ...
                                keyPointsCustom]);
          
                          
    % merge the same value (or too close) in keyPoints
    ii = 1;
    while ii<length(keyPoints{iShaft})
        distance = abs( keyPoints{iShaft}(ii) - keyPoints{iShaft}(ii+1) );
        isTooClose = distance < Shaft.totalLength(iShaft)/THRESHOLD_COEFFICIENT;
        if isTooClose
            keyPoints{iShaft}(ii+1) = [];
        else
            ii = ii + 1;
        end % end if
    end % end while

end % end for iShaft = 1:1:Shaft.amount

%%

% check the meshing parameter
if isAutoMesh
    
    switch gridFineness
        case 'low' 
            FINENESS = 1;
        case 'middle'
            FINENESS = 4;
        case 'high'
            FINENESS = 10;
        otherwise
            error('Please input: low, middle or high at the second parameter')
    end % end switch
    
else
    
    for iShaft = 1:1:Shaft.amount
        isMatch = length(manualGrid{iShaft}) == ( length(keyPoints{iShaft}) - 1 );
        if ~isMatch
            error(['the dimension of manual grid is not matched with',...
                   'segments. Please use auto mesh and check the number of segments'])
        end % end if
    end % end for iShaft
    
end % end if

%%

% mesh
rowSegmentNum = zeros(Shaft.amount,1);
nodeDistance = cell(Shaft.amount,1);
for iShaft = 1:1:Shaft.amount
    % calculate the number of segments for each shaft
    rowSegmentNum(iShaft) = length(keyPoints{iShaft}) - 1; 
    
    if isAutoMesh
        % calculate the number of elements for each segment
        standardLength = Shaft.totalLength(iShaft)/ FINENESS;
        elementNum = ceil( diff(keyPoints{iShaft}) ./ standardLength );
    else
        elementNum = manualGrid{iShaft};
    end
    
    
    % get the distance from left end of the shaft to each node 
    nodeDistance{iShaft} = [];
    for iSegment = 1:1:rowSegmentNum(iShaft)
        nodesInSegment = linspace( keyPoints{iShaft}(iSegment),...
                                   keyPoints{iShaft}(iSegment+1),...
                                   elementNum(iSegment)+1 );
        nodeDistance{iShaft} = [nodeDistance{iShaft}, nodesInSegment];
    end
    
    
    % merge the same value
    nodeDistance{iShaft} = unique(nodeDistance{iShaft});
    
end % end for iShaft = 1:1:Shaft.amount

%%

% generate a new struct saving the information of each node: Node
nodeNum = sum( cellfun(@length,nodeDistance) );
Node = struct(  'name',             cell(nodeNum,1),...
                'onShaftNo',        [],...
                'onShaftDistance',  [],...
                'diskNo',           [],...
                'bearingNo',        [],...
                'isBearing',        [],...
                'dof',              [] ); %initial

for iNode = 1:1:nodeNum
    if hasRub
        [Node(iNode).rubImpactNo] = [];
    end

    if hasInterBearing
        Node(iNode).interBearingNo = [];
    end

    if hasCoupling
        Node(iNode).couplingNo = [];
    end
    
    if hasLoosingBearing
        Node(iNode).isLoosingBearing = [];
    end

    if hasCustom
        [Node(iNode).CustomNo] = [];
    end
end

 
% check every node            
iShaft = 1;
previousShaftNodeNum = 0;
for iNode = 1:1:nodeNum
    Node(iNode).name = iNode;
    
    % Node.onShaftNo and Node.onShaftDistance
    Node(iNode).onShaftNo = iShaft;
    distanceHere = nodeDistance{iShaft}(iNode - previousShaftNodeNum);
    Node(iNode).onShaftDistance = distanceHere;
    Node(iNode).dof = Shaft.dofOfEachNodes(iShaft);
    
    % judge if the shaft is end
    [previousShaftNodeNum, iShaft] = judgeShaftEnd(previousShaftNodeNum,...
                                     nodeDistance, iShaft, iNode); 
end % end for iNode

%%

% Node.diskNo and Disk.positionOnShaftNode
[Disk,Node] = matchElement(Disk, 'disk', Node, nodeDistance,nodeNum,...
              THRESHOLD_COEFFICIENT, Shaft);
           
%%

% Node.bearingNo and Bearing.positionOnShaftNode
[Bearing,Node] = matchElement(Bearing, 'bearing', Node, nodeDistance,nodeNum,...
                 THRESHOLD_COEFFICIENT, Shaft);

%%

% Node.rubImpactNo and RubImpact.positionOnShaftNode
if hasRub
    [RubImpact,Node] = matchElement(RubImpact, 'rubImpact', Node,...
                       nodeDistance,nodeNum,THRESHOLD_COEFFICIENT, Shaft);
end

%%

% Node.couplingNo and Coupling.positionOnShaftNode
if hasCoupling
    [Coupling,Node] = matchElement(Coupling, 'couplingMisalignment', Node,...
                      nodeDistance,nodeNum,THRESHOLD_COEFFICIENT, Shaft);
end

%%

% Node.interBearingNo and InterBearing.positionOnShaftNode
if hasInterBearing
    [InterBearing,Node] = matchElement(InterBearing, 'interBearing', Node,...
                          nodeDistance,nodeNum,THRESHOLD_COEFFICIENT, Shaft);
end

%%

% Node.customNo and Custom.positionOnShaftNode
if hasCustom
    [Custom,Node] = matchElement(Custom, 'custom', Node,...
                       nodeDistance,nodeNum,THRESHOLD_COEFFICIENT, Shaft);
end

%%

% Node.isLoosingBearing
if hasLoosingBearing
    loosingBearingIndex = LoosingBearing.inBearingNo;
    loosingNode = Bearing.positionOnShaftNode(loosingBearingIndex);
    [Node(loosingNode).isLoosingBearing] = deal(true);
    for iNode = 1:1:nodeNum
        if isempty(Node(iNode).isLoosingBearing)
            Node(iNode).isLoosingBearing = false;
        end
    end
end


%%

% Node.isBearing and Bearing.positionNode
% generater bearing nodes
isInter = false;
[nodeNum, Bearing, Node] = addNode(nodeNum, Bearing, Node, hasLoosingBearing, isInter);

%%

% Node.isInterBearing and InterBearing.positionNode
% generater bearing nodes
if hasInterBearing
    isInter = true;
    [nodeNum, InterBearing, Node] = addNode(nodeNum, InterBearing, Node, hasLoosingBearing, isInter);
end

%%

% Mesh.dofInterval
dofOfEachNode = [Node.dof];
dofInterval = zeros(nodeNum,2);
for iNode = 1:1:nodeNum
    endDof = sum( dofOfEachNode(1:iNode) );
    startDof = endDof - dofOfEachNode(iNode) +1;
    dofInterval(iNode,:) = [startDof, endDof];
end

%%

% Mesh.dofOnNodeNo
dofOnNodeNo = zeros(sum([Node.dof]),1);
dofNo = 1;
for iNode = 1:1:nodeNum
    for iDof = 1:1:Node(iNode).dof
        dofOnNodeNo(dofNo) = Node(iNode).name;
        dofNo = dofNo + 1;
    end
end

%% 

% output
Parameter = InitialParameter;


% output struct: Mesh
Mesh.Node = Node;
Mesh.keyPointsDistance = keyPoints;
Mesh.nodeDistance = nodeDistance;
Mesh.nodeNum = nodeNum;
Mesh.dofNum = sum([Node.dof]);
Mesh.dofOnNodeNo = dofOnNodeNo;
Mesh.dofInterval = dofInterval;
Parameter.Mesh = Mesh;


%output the node position of every important elements
Parameter.Disk = Disk;
Parameter.Bearing = Bearing;

if hasRub
    Parameter.RubImpact = RubImpact;
end

if hasCoupling
    Parameter.CouplingMisalignment = Coupling;
end

if hasInterBearing
    Parameter.IntermediateBearing = InterBearing;
end

if hasCustom
    Parameter.Custom = Custom;
end


%% subfunction 1: matchElement

% write the node information into important elements (e.g. Disk, Bearing)
% and the struct Node
function [Element,Node] = matchElement(Element, elementName, Node,...
                          nodeDistance, nodeNum, THRESHOLD_COEFFICIENT, Shaft)
            
if strcmp(elementName,'interBearing')
    columnNum = 2;
else
    columnNum = 1;
end

Element.positionOnShaftNode = zeros(Element.amount,columnNum);    
shaftNo = 1;
previousShaftNode = 0;
iElement = 1;
for nodeNo = 1:1:nodeNum
    % calculate the distance at nodeNo
    distanceH = nodeDistance{shaftNo}(nodeNo - previousShaftNode);
    
    
    % judge if this node has important element (e.g. disk, bearing)
    if strcmp(elementName,'interBearing')
        isShaftHere = shaftNo == Element.betweenShaftNo;
    else
        isShaftHere = shaftNo == Element.inShaftNo;
    end
    isDistanceHere = abs(distanceH - Element.positionOnShaftDistance)...
                     < Shaft.totalLength(shaftNo)/THRESHOLD_COEFFICIENT;
    isElementHere = isShaftHere & isDistanceHere; % logical matrix
    isElementHereNum = length(isElementHere(isElementHere == true));

    
    % write the information into Element and Node
    if isElementHereNum == 1 % there is a single disk on this position
        
        % Element.positionOnShaftNode 
        indexElement = find(isElementHere == true);
        Element.positionOnShaftNode(indexElement) = nodeNo;
        
        % Node.xxxxNo
        switch elementName
            case 'disk'
                Node(nodeNo).diskNo = iElement;
            case 'bearing'
                Node(nodeNo).bearingNo = iElement;
            case 'rubImpact'
                Node(nodeNo).rubImpactNo = iElement;
            case 'couplingMisalignment'
                Node(nodeNo).couplingNo = iElement;
            case 'custom'
                Node(nodeNo).customNo = iElement;
            case 'interBearing'
                % only write the interBearing No. of 1st column of interBearing
                if sum(isElementHere(:,1)) == 1 
                    Node(nodeNo).interBearingNo = iElement; 
                else
                    iElement = iElement - 1;
                end % end if
            otherwise
                error('wrong elementName in matchElement()')
        end % end switch
        
        iElement = iElement +1;
        
    elseif isElementHereNum > 1 % there are too many disks on this position
       
        error(['the distance between disks is too close, please modify the',...
               'THRESHOLD_COEFFICIENT in meshModel() or redefine the position of disks']);

    end % end if isElementHereNum == 1
    
    
    % judge if the shaft is end
    [previousShaftNode, shaftNo] = judgeShaftEnd(previousShaftNode,...
                                   nodeDistance, shaftNo, nodeNo);

end % end for 


% write the interBearing No. of 2nd column of interBearing
if strcmp(elementName,'interBearing')
        index1Column = Element.positionOnShaftNode(:,1);
        index2Column = Element.positionOnShaftNode(:,2);
        for iInterBearing = 1:1:length(index1Column)
            Node(index2Column(iInterBearing)).interBearingNo = Node(index1Column(iInterBearing)).interBearingNo;
        end
end % end if

end % end subfunction matchElement()

%% subfunction 2: judgeShaftEnd

% judge if this node is the end of the shaft. true -> iShift +1 and
% previousShaftNodeNum + the number of nodes last shaft having
function [previousShaftNodeNum, iShaft]...
        = judgeShaftEnd(previousShaftNodeNum, nodeDistance, iShaft, iNode)

isShaftEnd = iNode == (previousShaftNodeNum +...
                   length(nodeDistance{iShaft}) );
if isShaftEnd
    previousShaftNodeNum = previousShaftNodeNum +...
                           length(nodeDistance{iShaft});
    iShaft = iShaft +1;
end % end if

end % end sub function judgeShaftEnd()

%% subfunction 3: addNode

% add Node.isInterBearing and InterBearing.positionNode, Node.isBearing and
% Bearing.positionNode for Bearing and InterBearing
function [nodeNum, Element, Node] = addNode(nodeNum, Element, Node, hasLoosingBearing, isInter)
indexHasMass = find(Element.mass' ~= 0); % index in 1-d
[indexHasMassRow, indexHasMassCol] = find(Element.mass' ~= 0); % index in 2-d
bearingNodeNum = length(indexHasMass);
bearingMassColumnNum = size(Element.mass,2);
Element.positionNode = zeros(Element.amount,bearingMassColumnNum); % initial
for iBearingNode = 1:1:bearingNodeNum
    newNodeNo = nodeNum + iBearingNode;
    onNodeNo = Element.positionOnShaftNode(indexHasMassCol(iBearingNode),:);
    % copy information in Node
    Node(newNodeNo).onShaftNo = [Node(onNodeNo).onShaftNo];
    Node(newNodeNo).onShaftDistance = [Node(onNodeNo).onShaftDistance];
    
    if isInter
        Node(newNodeNo).interBearingNo = unique([Node(onNodeNo).interBearingNo]);
    else
        Node(newNodeNo).bearingNo = Node(onNodeNo).bearingNo;
    end
    
    if hasLoosingBearing
        Node(newNodeNo).isLoosingBearing = Node(onNodeNo).isLoosingBearing;
    end
    % write new information in Node
    Node(newNodeNo).name = newNodeNo; 
    Node(newNodeNo).dof = Element.dofOfEachNodes(indexHasMassCol(iBearingNode),indexHasMassRow(iBearingNode));
    Node(newNodeNo).isBearing = true;
    % Element.positionNode
    Element.positionNode(indexHasMassCol(iBearingNode),indexHasMassRow(iBearingNode)) = newNodeNo;
end


for iNodee = 1:1:nodeNum
    if isempty(Node(iNodee).isBearing)
        Node(iNodee).isBearing = false;
    end
end
nodeNum = nodeNum + bearingNodeNum;
    
end
end % end main function