%% FEMBEARING - Generate FEM matrices for bearing components
% Assembles global mass, stiffness, and damping matrices with gravity vector 
% for bearing systems. Supports normal and loose bearing configurations.
%
%% Syntax
%   [M, K, C, Fg] = femBearing(Bearing, nodeDof)
%   [M, K, C, Fg, KLoose, CLoose] = femBearing(Bearing, nodeDof, LoosingBearing)
%
%% Description
% |FEMBEARING| handles both standard and mass-containing bearings, with 
% special handling for bearings with clearance/looseness effects. Features:
% * Dual operation modes: normal vs. loose bearing configurations
% * Distributed stiffness/damping modeling
% * Gravity force integration
%
%% Input Arguments
% *Bearing* - Bearing properties structure:
%   .amount             % Number of bearings (scalar)
%   .dofOfEachNodes     % [N×n] DOF per node
%   .stiffness          % [N×n] Bearing horizontal stiffness coefficients [N/m]
%   .stiffnessVertical  % [N×n] Bearing vertical stiffness coefficients [N/m]
%   .damping            % [N×n] Bearing horizontal Damping coefficients [Ns/m]
%   .dampingVertical    % [N×n] Bearing vertical Damping coefficients [Ns/m]
%   .mass               % [N×n] Bearing masses [kg]
%   .positionOnShaftNode% [N×1] Mounting on shaft node indices
%   .positionNode       % [N×1] mass node indices of bearings
% N is number of bearings
% n is the maximum number of bearing mass blocks +1 in your rotor system
%
% *nodeDof*             % [M×1] DOF count per system node, M is the number
%                         of nodes
%
% *LoosingBearing*     % (Optional) Loose bearing parameters:
%   .inBearingNo       % Bearing indices with clearance
%   .loosingStiffness  % Modified vertical stiffness values [N/m]
%   .loosingDamping    % Modified vertical damping values [Ns/m]
%
%% Output Arguments
% *M*          % Global mass matrix (n×n sparse)
% *K*          % Global stiffness matrix (n×n sparse)
% *C*          % Global damping matrix (n×n sparse)
% *Fg*         % Gravity force vector (n×1)
% *KLoose*     % Loose bearing stiffness matrix (n×n sparse)
% *CLoose*     % Loose bearing damping matrix (n×n sparse)
% n is the number of dofs of entire rotor model
%
%% Algorithm
% 1. Bearing classification:
%    - Normal bearings (mass = 0): Direct stiffness/damping addition
%    - Mass bearings: Multi-node element assembly
% 2. Loose bearing handling:
%    - Modifies stiffness/damping for specified bearings
%    - Maintains separate matrix copies for dynamic switching
%
%% Example
% % Standard bearing assembly
% bearingParams = inputBearingHertzBO().Bearing;
% nodeDOF = [4;4;...];
% Bearing.positionOnShaftNode = [3; 2;...];
% Bearing.positionNode = [12; 16;...];
% [M_bear, K_bear] = femBearing(bearingParams, nodeDOF);
%
% % Loose bearing configuration
% looseParams = struct('inBearingNo', 2, 'loosingStiffness', 1e6, ...);
% nodeDOF = [4;4;...];
% Bearing.positionOnShaftNode = [3; 2;...];
% Bearing.positionNode = [12; 16;...];
% [M, K, C, Fg, KL, CL] = femBearing(bearingParams, nodeDOF, looseParams);
%
%% See Also
% bearingElement, bearingElementMass, meshModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function varargout = femBearing(varargin)

% check input
if nargin == 2
    Bearing = varargin{1};
    nodeDof = varargin{2};
    LoosingBearing = [];
elseif nargin == 3
    Bearing = varargin{1};
    nodeDof = varargin{2};    
    LoosingBearing = varargin{3};
end

%%

% generate global matrices and vectors
dofNum = sum(nodeDof);
M = zeros(dofNum, dofNum);
K = zeros(dofNum, dofNum);
C = zeros(dofNum, dofNum);
Fg = zeros(dofNum, 1);

if ~isempty(LoosingBearing)
    KLoose = zeros(dofNum, dofNum);
    CLoose = zeros(dofNum, dofNum);
end

%%

% distinguish the normal bearing elements from all bearing elements
bearingMassSum = sum(Bearing.mass,2);
normalBearingIndex = find(bearingMassSum == 0);
massBearingIndex   = find(bearingMassSum ~= 0);
Temporary          = rmfield(Bearing,'amount');


%% 

% normal bearing (no mass)
if ~isempty(normalBearingIndex)
    NormalBearing    = getStructPiece(Temporary,normalBearingIndex,[]);
    % check the number of input k and c 
    countk = sum(NormalBearing.stiffness ~= 0, 2);
    countc = sum(NormalBearing.damping ~= 0, 2);
    count = [countk; countc];
    if sum(count) ~= length(count)
        error('Too much stiffness or damping for a no mass bearing, please input one stiffness and damping for a no mass bearing.')  
    end
    % initial Ke Ce for normal bearing
    normalBearingNum = size(NormalBearing.stiffness,1);
    KeN = cell(normalBearingNum,1); 
    CeN = cell(normalBearingNum,1);


    % generate normal bearing elements
    for iNBearing = 1:1:normalBearingNum
        % get the information of ith normal braring
        ANBearing = getStructPiece(NormalBearing,iNBearing,[]); % a normal bearing
        ANBearing.dofOnShaftNode = nodeDof(ANBearing.positionOnShaftNode);
        % generate elements (MeN: Me for normal bearing)
        [KeN{iNBearing}, CeN{iNBearing}] = bearingElement(ANBearing); 
    end


    % find the index of element in global matrix
    nBearingIndex = findIndex(NormalBearing.positionOnShaftNode,nodeDof); 


    % put the normal bearing elements into global matrix
    for iNBearing = 1:1:normalBearingNum
        K = addElementIn( K, KeN{iNBearing}, nBearingIndex(iNBearing, :) );
        C = addElementIn( C, CeN{iNBearing}, nBearingIndex(iNBearing, :) );
    end
    
    
    % save a copy data for loosing bearing
    if ~isempty(LoosingBearing)
        KLoose = K;
        CLoose = C;
    end

end % end if ~isempty(normalBearingIndex)
%%

% mass bearing
if ~isempty(massBearingIndex)
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
                                           = bearingElementMass(AMBearing); 
    end
    
    
    % find the index of element in global matrix
    position = [MassBearing.positionOnShaftNode, MassBearing.positionNode(:,1)];
    mBearingIndex = findIndex(position,nodeDof); 
    
    
    % put the mass bearing elements into global matrix
    for iMBearing = 1:1:massBearingNum
        K = repeatAdd(K, KeM, iMBearing, mBearingIndex);
        C = repeatAdd(C, CeM, iMBearing, mBearingIndex);
        M = repeatAdd(M, MeM, iMBearing, mBearingIndex);
    end

    % find the index of element into global vector (for gravity)
    position = MassBearing.positionNode(:,1);
    mBearingIndex = findIndex(position,nodeDof);

    % put the gravity of mass bearing elements into global vector (for gravity)
    for iMBearing = 1:1:massBearingNum
        indexHere = [mBearingIndex(iMBearing,1), 1];
        Fg = addElementIn(Fg, FgeM{iMBearing}, indexHere);
    end
    
end % if ~isempty(massBearingIndex)

%%

% loosing bearing
if ~isempty(LoosingBearing)
    MassBearing    = getStructPiece(Temporary,massBearingIndex,[]);
    massBearingNum = size(MassBearing.stiffness,1);
    KeM = cell(massBearingNum,1); 
    CeM = cell(massBearingNum,1);
    
    % generat mass bearing elements
    for iMBearing = 1:1:massBearingNum
        % get the Bearing No.
        thisNo = massBearingIndex(iMBearing);
        % find the Lossing Bearing here
        thisLoosingIndex = find(LoosingBearing.inBearingNo==thisNo);
        % get the information of ith mass braring
        AMBearing = getStructPiece(MassBearing,iMBearing,[]); % a normal bearing
        AMBearing.dofOnShaftNode = nodeDof(AMBearing.positionOnShaftNode);
        % change stiffness and damping for lossing bearing
        if ~isempty(thisLoosingIndex)
            for iLoosingKC = 1:1:length(thisLoosingIndex)
                kLoose = LoosingBearing.loosingStiffness(thisLoosingIndex);
                cLoose = LoosingBearing.loosingDamping(thisLoosingIndex);
                looseNo = LoosingBearing.loosingPositionNo(thisLoosingIndex);
                AMBearing.stiffnessVertical(looseNo) = kLoose;
                AMBearing.dampingVertical(looseNo) = cLoose;
            end
        end
        % generate elements (MeN: Me for mass bearing)
        [~, KeM{iMBearing}, CeM{iMBearing}]= bearingElementMass(AMBearing);
    end % end for iMBearing
    
    
    % find the index of element in global matrix
    position = [MassBearing.positionOnShaftNode, MassBearing.positionNode(:,1)];
    mBearingIndex = findIndex(position,nodeDof); 
    
    % put the mass bearing elements into global matrix
    for iMBearing = 1:1:massBearingNum
        KLoose = repeatAdd(KLoose, KeM, iMBearing, mBearingIndex);
        CLoose = repeatAdd(CLoose, CeM, iMBearing, mBearingIndex);
    end % end for

end % end if 


%%

% output
if nargin == 2
    varargout{1} = M;
    varargout{2} = K;
    varargout{3} = C;
    varargout{4} = Fg;
elseif nargin == 3
    varargout{1} = M;
    varargout{2} = K;
    varargout{3} = C;
    varargout{4} = Fg;
    varargout{5} = KLoose;
    varargout{6} = CLoose;
end

%%

% sub function
function B = repeatAdd(A, Ae, iObject, aIndex)
    A = addElementIn(A, Ae{iObject}{1,1}, aIndex(2*iObject-1,[1,2]));
    A = addElementIn(A, Ae{iObject}{1,2}, aIndex(2*iObject-1,[3,4]));
    A = addElementIn(A, Ae{iObject}{2,1}, aIndex(2*iObject,[1,2]));
    A = addElementIn(A, Ae{iObject}{2,2}, aIndex(2*iObject,[3,4]));
    B = A;
end

end