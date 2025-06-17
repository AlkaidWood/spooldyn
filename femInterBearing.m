%FEMINTERBEARING Generate FEM matrices for intermediate bearings with/without mass
%
% Syntax:
%   [M, K, C, Fg] = femInterBearing(InterBearing, nodeDof)
%
% Input Arguments:
%   InterBearing - Intermediate bearing parameters structure with fields:
%       .positionOnShaftNode: [n×2 double] Node positions connected to shaft (start and end)
%       .positionNode: [n×2 double]       Node positions for mass bearings (if applicable)
%       ... see also inputIntermediateBearing()
%   nodeDof - [m×1 double]               Number of DOFs per node in global system
%
% Output Arguments:
%   M - [n×n sparse]                     Global mass matrix
%   K - [n×n sparse]                     Global stiffness matrix
%   C - [n×n sparse]                     Global damping matrix
%   Fg - [n×1 double]                    Gravity force vector
%
% Description:
%   Constructs global FEM matrices for intermediate bearings considering:
%   - Bearings without mass (connects shaft nodes)
%   - Bearings with concentrated mass (additional node connections)
%   Automatically handles matrix assembly for different bearing types
%
% Notes:
%   - Throws error if non-mass bearings receive multiple stiffness/damping values
%   - Uses local coordinate transformation for bearing elements
%
% Example:
%   % generate global matrix of Inter-shaft Bearing part
%   % structure looks like: 
%   % Node-1 -- k=1e6 c=300 -- mass=0.5 Node-7 -- k=1e6 c=200 -- Node-3
%   % Node-2 -- k=1e6 c=300 -- mass=1 Node-8 -- k=1e6 c=200 -- Node-6
%   Interbearing.amount = 2; % two inter-shaft bearing
%   Interbearing.dofOfEachNodes = [2; 2]; % bearings have mass and  node
%   Interbearing.isHertzian = [false; false]; % ignore Herzian contact
%   Interbearing.isHertzianTop = [false; false];
%   Interbearing.stiffness = [1e6, 1e6; 1e7, 1e7];
%   Interbearing.stiffnessVertical = [1e6, 1e6; 1e7, 1e7];
%   Interbearing.damping = [300, 200; 400, 500];
%   Interbearing.dampingVertical = [300, 200; 400, 500];
%   Interbearing.mass = [0.5; 1];
%   Interbearing.positionOnShaftNode = [1, 3; 2, 6];
%   Interbearing.positionNode = [7; 8];
%   nodeDof = [4,4,4,4,4,4,2,2];
%   [M, K, C, Fg] = femInterBearing(Interbearing, nodeDof);
%
% See also BEARINGELEMENTINTER, BEARINGELEMENTINTERMASS
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.

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