%% femInterBearing
% generate the globe mass, stiffness, damping matrix, gravity of bearings
%% Syntax
% [M, K, C, Fg] = femInterBearing( InterBearing, nodeDof )
%% Description
% InterBearing is a struct saving the physical parameters of bearings with 
% fields: amount, stiffness, damping, positionOnShaftNode
%
% nodeDof: is a array (the number of nodes  * 1) saving the dof of each 
% node.
%
% M, K, C are mass, stiffness, damping matrix of intermediate bearings. 
% (n*n, n is the number of all nodes)
%
% Fg is gravity vector (n*1)

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