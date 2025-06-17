%% FEMSHAFT - Generate FEM matrices for shaft components
% Assembles global mass, stiffness, gyroscopic, and N-matrix matrices with 
% gravity vector for shaft structures using Timoshenko beam elements.
%
%% Syntax
%   [M, K, G, N, Fg] = femShaft(Shaft, nodeDistance)
%
%% Description
% |FEMSHAFT| performs finite element matrix assembly for multi-shaft rotor
% systems. Implements Timoshenko beam theory for:
% * Distributed mass/stiffness effects
% * Gyroscopic moments
% * Geometric nonlinearities
% * Gravity loading
%
%% Input Arguments
% *Shaft* - Shaft properties structure:
%   .amount            % Number of shafts (scalar)
%   .dofOfEachNodes    % [N×1] DOF per node (vector)
%   .outerRadius       % [N×1] Outer radii [m]
%   .innerRadius       % [N×1] Inner radii [m]
%   .density           % [N×1] Material densities [kg/m³]
%   .elasticModulus    % [N×1] Elastic moduli [Pa]
%   .poissonRatio      % [N×1] Poisson's ratios
%
% *nodeDistance*       % [N×1 cell] Node positions per shaft:
%   {i} = [d1 d2... dn] % Axial positions for shaft i [m]
%
%% Output Arguments
% *M*      % Global mass matrix (n×n sparse)
% *K*      % Global stiffness matrix (n×n sparse)
% *G*      % Global gyroscopic matrix (n×n sparse)
% *N*      % Nonlinear geometric stiffness matrix (n×n sparse)
% *Fg*     % Gravity force vector (n×1)
%
%% Algorithm
% 1. Element-level matrix generation:
%    - Uses Timoshenko beam formulation
%    - Accounts for annular cross-sections
% 2. Per-shaft matrix assembly:
%    - Combines element matrices with overlap at shared nodes
% 3. Global system assembly:
%    - Stacks shaft matrices diagonally
%
%% Example
% % Generate shaft FEM matrices
% shaftParams = inputEssentialParameterBO().Shaft;
% nodeDist = {linspace(0,1,10), linspace(0,0.5,5)};
% [M, K, G, ~, Fg] = femShaft(shaftParams, nodeDist);
%
%% See Also
% shaftElement, assembleLinear, meshModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


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