%% femInterBearing - Generate FEM matrices for inter-shaft bearings
%
% This function assembles global mass, stiffness, and damping matrices 
% along with gravity force vectors for intermediate bearings in rotor 
% systems, supporting both massless and mass-bearing configurations.
%
%% Syntax
%  [M, K, C, Fg] = femInterBearing(InterBearing, nodeDof)
%
%% Description
% |femInterBearing| constructs finite element matrices for intermediate 
% bearings connecting two shaft segments. The function:
% * Handles both massless and mass-containing bearings
% * Supports complex bearing topologies with multiple mass nodes
% * Generates partitioned matrices for global assembly
% * Computes gravity forces for mass-bearing components
%
%% Input Arguments
% * |InterBearing| - Intermediate bearing properties structure:
%   * |amount|              % Number of bearings (scalar)
%   * |dofOfEachNodes|      % DOF per bearing node [n×1 vector]
%   * |stiffness|           % Horizontal stiffness [N/m] [n×m matrix]
%   * |stiffnessVertical|   % Vertical stiffness [N/m] [n×m matrix]
%   * |damping|             % Horizontal damping [N·s/m] [n×m matrix]
%   * |dampingVertical|     % Vertical damping [N·s/m] [n×m matrix]
%   * |mass|                % Concentrated masses [kg] [n×m matrix]
%   * |positionOnShaftNode| % Connected shaft node indices [n×2 matrix]
%   * |positionNode|        % Mass node indices [n×2 matrix]
%   * n: Number of intermediate bearings
%   * m: Maximum number of mass blocks per bearing
%
% * |nodeDof| - DOF counts per system node [p×1 vector], p = number of nodes
%
%% Output Arguments
% * |M|  % Global mass matrix [sparse q×q]
% * |K|  % Global stiffness matrix [sparse q×q]
% * |C|  % Global damping matrix [sparse q×q]
% * |Fg| % Gravity force vector [q×1]
%   * q: Total DOF of rotor system = sum(nodeDof)
%
%% Matrix Assembly Algorithm
% 1. Bearing Classification:
%   * Massless bearings (sum(mass)==0)
%   * Mass bearings (sum(mass)≠0)
% 2. Massless Bearing Processing:
%   * Validates single stiffness/damping input
%   * Generates elements via |bearingElementInter|
%   * Assembles to shaft connection points
% 3. Mass Bearing Processing:
%   * Generates matrices via |bearingElementInterMass|
%   * Handles multi-node connectivity
%   * Adds gravity forces to mass nodes
% 4. Position Mapping:
%   * Uses |findIndex| for DOF position calculation
% 5. Global Assembly:
%   * Utilizes |addElementIn| for matrix expansion
%   * Employs |repeatAdd| for partitioned matrix assembly
%
%% Special Features
% * Automatic Classification:
%   * Separates massless and mass-bearing elements
%   * Handles each type with specialized functions
% * Complex Topology Support:
%   * Handles bearings with multiple mass nodes
%   * Manages connections between shaft and mass nodes
% * Gravity Force Calculation:
%   * Computes gravity forces only for mass-bearing components
%
%% Example
% % Configure intermediate bearing with two bearings
% Interbearing.amount = 2;
% Interbearing.dofOfEachNodes = [2; 2];
% Interbearing.stiffness = [1e6, 1e6; 1e7, 1e7];
% Interbearing.stiffnessVertical = [1e6, 1e6; 1e7, 1e7];
% Interbearing.damping = [300, 200; 400, 500];
% Interbearing.dampingVertical = [300, 200; 400, 500];
% Interbearing.mass = [0.5; 1]; % Concentrated masses
% Interbearing.positionOnShaftNode = [1, 3; 2, 6]; % Shaft connections
% Interbearing.positionNode = [7; 8]; % Mass node positions
% nodeDof = [4,4,4,4,4,4,2,2]; % DOF configuration
% % Generate global matrices
% [M, K, C, Fg] = femInterBearing(Interbearing, nodeDof);
%
%% Implementation Notes
% * Matrix Assembly Functions:
%   * |repeatAdd|: Assembles 2×2 partitioned matrices
%   * |repeatAdd2|: Handles larger partitioned matrices
% * Position Handling:
%   * |positionOnShaftNode| specifies shaft connection points
%   * |positionNode| specifies mass node locations
% * Zero-Mass Validation:
%   * Requires single stiffness/damping for massless bearings
%
%% See Also
% bearingElementInter, bearingElementInterMass, findIndex, addElementIn
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function  [M, K, C, Fg] = femInterBearing( InterBearing, nodeDof )

% generate global matrices and vectors
dofNum = sum(nodeDof);
M = zeros(dofNum, dofNum);
K = zeros(dofNum, dofNum);
C = zeros(dofNum, dofNum);
Fg = zeros(dofNum, 1);

%%

% distinguish the bearing elements without mass from all elements
bearingMassSum = sum(InterBearing.mass,2);
normalBearingIndex = find(bearingMassSum == 0);
massBearingIndex   = find(bearingMassSum ~= 0);
Temporary          = rmfield(InterBearing,'amount');

%%

% bearing elements without mass
if ~isempty(normalBearingIndex)
    NormalBearing    = getStructPiece(Temporary,normalBearingIndex,[]);
    % check the number of input k and c 
    countk = sum(NormalBearing.stiffness ~= 0, 2);
    countc = sum(NormalBearing.damping ~= 0, 2);
    count = [countk; countc];
    if sum(count) ~= length(count)
        error('Too much stiffness or damping for a no mass bearing, please input one stiffness and damping for a no mass bearing.')  
    end
    
    
    % gnerate elements
    normalBearingNum = size(NormalBearing.stiffness,1);
    Ke = cell(normalBearingNum,1); 
    Ce = cell(normalBearingNum,1);
    
    for iBearing = 1:1:normalBearingNum
        % get the information of ith intermediate braring
        ABearing = getStructPiece(NormalBearing,iBearing,[],false); % a normal bearing
        ABearing.dofOnShaftNode = nodeDof(ABearing.positionOnShaftNode(iBearing,:));
        % generate elements 
        [Ke{iBearing}, Ce{iBearing}] = bearingElementInter(ABearing); 
    end
    
    
    % find index of elements in the global matrix
    bearingIndex = findIndex(NormalBearing.positionOnShaftNode,nodeDof); 

    
    % put the intermediate bearing elements into global matrix
    for iBearing = 1:1:normalBearingNum
        K = repeatAdd(K, Ke, iBearing, bearingIndex);
        C = repeatAdd(C, Ce, iBearing, bearingIndex);
    end
end


%%

% intermediate bearing with mass
if ~isempty(massBearingIndex)
    % initial
    MassBearing    = getStructPiece(Temporary,massBearingIndex,[]);
    massBearingNum = size(MassBearing.stiffness,1);
    MeM = cell(massBearingNum,1);
    KeM = cell(massBearingNum,1); 
    CeM = cell(massBearingNum,1);
    FgeM = cell(massBearingNum,1);
    
    
    % generate mass bearing elements
    for iMBearing = 1:1:massBearingNum
        % get the information of ith mass braring
        AMBearing = getStructPiece(MassBearing,iMBearing,[]); % a normal bearing
        AMBearing.dofOnShaftNode = nodeDof(AMBearing.positionOnShaftNode);
        % generate elements (MeN: Me for mass bearing)
        [MeM{iMBearing}, KeM{iMBearing}, CeM{iMBearing}, FgeM{iMBearing}]...
                                           = bearingElementInterMass(AMBearing); 
    end

    % assembly mass matrix and gravity
    positionM = MassBearing.positionNode(:,1); % find index of element in global matrix
    mBearingIndexM = findIndex(positionM, nodeDof);
    for iMBearing = 1:1:massBearingNum
        M = addElementIn(M, MeM{iMBearing}{2,2}, mBearingIndexM(iMBearing, :));
        Fg = addElementIn(Fg, FgeM{iMBearing}, [mBearingIndexM(iMBearing,1),1]);
    end
    
    % assembly stiffness and damping matrix
    for iMBearing = 1:1:massBearingNum
        % find index
        nodeShaft1 = MassBearing.positionOnShaftNode(iMBearing,1);
        nodeShaft2 = MassBearing.positionOnShaftNode(iMBearing,2);
        nodeMass1 = MassBearing.positionNode(iMBearing,1);
        mnIndex = MassBearing.positionNode(iMBearing,:)~=0;
        nodeMassn = MassBearing.positionNode(iMBearing,mnIndex);
        nodeMassn = nodeMassn(end);
        positionInner = [nodeShaft1, nodeMass1];
        positionOuter = [nodeShaft2, nodeMassn];
        index1 = findIndex(positionInner, nodeDof);
        index2 = findIndex(positionOuter, nodeDof);
        mBearingIndex = [index1(1,1:2);...
                         index1(1,3:4);...
                         index1(2,1:2);...
                         index2(1,1:2);...
                         index2(1,3:4);...
                         index2(2,1:2);...
                         index1(2,3:4)]; % these order of index are associate with the order in bearingElementInterMass()
        % assembly
        K = repeatAdd2(K, KeM, iMBearing, mBearingIndex);
        C = repeatAdd2(C, CeM, iMBearing, mBearingIndex);
    end
end

%%
% sub function 1
function B = repeatAdd(A, Ae, iObject, aIndex)
    A = addElementIn(A, Ae{iObject}{1,1}, aIndex(2*iObject-1,[1,2]));
    A = addElementIn(A, Ae{iObject}{1,2}, aIndex(2*iObject-1,[3,4]));
    A = addElementIn(A, Ae{iObject}{2,1}, aIndex(2*iObject,[1,2]));
    A = addElementIn(A, Ae{iObject}{2,2}, aIndex(2*iObject,[3,4]));
    B = A;
end


% sub function 2
function B = repeatAdd2(A, Ae, iObject, aIndex)
    for iElement=1:1:length(Ae{iObject})
        A = addElementIn(A, Ae{iObject}{iElement}, aIndex(iElement,:));
    end
    B = A;
end % end sub function

end