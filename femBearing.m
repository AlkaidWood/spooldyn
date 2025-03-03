%% femBearing
% generate the global mass, stiffness, damping matrix, gravity of bearings
%% Syntax
% [M, K, C, Fg] = femBearing(Bearing, nodeDof)
%
% [M, K, C, Fg, KLoose, CLoose] = femBearing(Bearing, nodeDof, LoosingBearing)
%% Description
% Bearing is a struct saving the physical parameters of bearings with 
% fields: amount, dofOfEachNodes, stiffness, damping, mass,
% positionOnShaftNode, positionNode
%
% nodeDof: is a array (the number of nodes  * 1) saving the dof of each 
% node.
%
% M, K, C are mass, stiffness, damping matrix of bearings. (n*n,
% n is the number of all nodes)
%
% Fg is gravity vector of bearings
%
% KLoose, CLoose are mass, stiffness, damping matrix of loosing bearings


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