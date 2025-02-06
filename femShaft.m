%% femShaft
% generate the globe mass, stiffness, gyroscopic matrix, gravity of shafts
%% Syntax
% [M, K, G, N, Fg] = femShaft(Shaft, nodeDistance)
%% Description
% Shaft is a struct saving the physical parameters of shafts with fields:
% amount, dofOfEachNodes, outerRadius, innerRadius, density,
% elasticModulus, poissonRatio.
%
% nodeDistance: is a cell (Physics.amount * 1) saving the distance
% information. nodeDistance{i} saves all distance from left end of the i-th 
% Shaft to each node (m)
%
% M, K, G, N are mass, stiffness, gyroscopic, N matrix of shafts. (n*n,
% n is the number of all nodes on shafts)
%
% Fg is the gravity vector (n*1)


function [M, K, G, N, Fg] = femShaft(Shaft, nodeDistance)

% check input parameters
Temporary = rmfield(Shaft,'rayleighDamping');
checkInputData(Temporary);
isMatch = Shaft.amount == length(nodeDistance);
if ~isMatch
    error('the dimension of two input parameters are not matched');
end % end if

%%

% generate element matrix and vector
Me = cell(Shaft.amount,1); 
Ke = cell(Shaft.amount,1);
Ge = cell(Shaft.amount,1);
Ne = cell(Shaft.amount,1);
Fge = cell(Shaft.amount,1);

Temporary = rmfield(Shaft,{'amount','rayleighDamping'}); % for extract part of data of Shaft

for iShaft = 1:1:Shaft.amount
    nodeNum = length(nodeDistance{iShaft});
    elementNum = nodeNum - 1;
    
    % The mattix of i-th element is saved in Me{iShaft}{iElement}
    Me{iShaft} = cell(elementNum,1);
    Ke{iShaft} = cell(elementNum,1);
    Ge{iShaft} = cell(elementNum,1);
    Ne{iShaft} = cell(elementNum,1);
    Fge{iShaft} = cell(elementNum,1);
    
    % get the physical information of the iShaft 
    AShaft = getStructPiece(Temporary,iShaft,[]);
    % generate elements
    for iElement = 1:1:elementNum
        % the length of the iElement
        AShaft.length = nodeDistance{iShaft}(iElement+1)...
                        - nodeDistance{iShaft}(iElement);
        [Me{iShaft}{iElement},...
         Ke{iShaft}{iElement},...
         Ge{iShaft}{iElement},...
         Ne{iShaft}{iElement},...
         Fge{iShaft}{iElement}] = shaftElement(AShaft);
    end % end for iElement
end % end for iShaft

%%

% assembling in each shaft
MiShaft = cell(Shaft.amount,1); % to save the mass matrix of each shaft
KiShaft = cell(Shaft.amount,1);
GiShaft = cell(Shaft.amount,1);
NiShaft = cell(Shaft.amount,1);
FgiShaft = cell(Shaft.amount,1); % to save the gravity vector of each shaft

for iShaft = 1:1:Shaft.amount
    nodeNum = length(nodeDistance{iShaft});
    elementNum = nodeNum - 1;
    
    % the row and column number of intersection
    intersectRow = Shaft.dofOfEachNodes(iShaft) * ones(1,elementNum-1);
    intersectColumn = intersectRow;
    MiShaft{iShaft} = assembleLinear(Me{iShaft}, intersectRow, intersectColumn);
    KiShaft{iShaft} = assembleLinear(Ke{iShaft}, intersectRow, intersectColumn);
    GiShaft{iShaft} = assembleLinear(Ge{iShaft}, intersectRow, intersectColumn);
    NiShaft{iShaft} = assembleLinear(Ne{iShaft}, intersectRow, intersectColumn);
    FgiShaft{iShaft} = assembleLinear(Fge{iShaft}, intersectRow, ones(1,elementNum-1));
end


%%

% assembling shafts
intersectRow = zeros(1,Shaft.amount-1);
intersectColumn = zeros(1,Shaft.amount-1);
M = assembleLinear(MiShaft, intersectRow, intersectColumn);
K = assembleLinear(KiShaft, intersectRow, intersectColumn);
G = assembleLinear(GiShaft, intersectRow, intersectColumn);
N = assembleLinear(NiShaft, intersectRow, intersectColumn);
Fg = assembleLinear(FgiShaft, intersectRow, 1*ones(1,Shaft.amount-1));

end % end function