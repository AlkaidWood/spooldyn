%% femShaft - Generate FEM matrices for shaft components using Timoshenko beam theory
%
% This function assembles global mass, stiffness, gyroscopic, and nonlinear 
% matrices along with gravity vectors for multi-shaft rotor systems using 
% Timoshenko beam elements.
%
%% Syntax
%  [M, K, G, N, Fg] = femShaft(Shaft, nodeDistance)
%
%% Description
% |femShaft| constructs finite element matrices for shaft components in 
% rotor dynamics models. The function:
% * Implements Timoshenko beam theory for annular shafts
% * Handles multi-shaft configurations
% * Accounts for distributed mass and stiffness
% * Includes gyroscopic and geometric nonlinear effects
% * Computes gravity loading
%
%% Input Arguments
% * |Shaft| - Shaft properties structure:
%   * |amount|            % Number of shafts (scalar)
%   * |dofOfEachNodes|    % DOF per node [N×1 vector]
%   * |outerRadius|       % Outer radii [m] [N×1 vector]
%   * |innerRadius|       % Inner radii [m] [N×1 vector]
%   * |density|           % Material densities [kg/m³] [N×1 vector]
%   * |elasticModulus|    % Elastic moduli [Pa] [N×1 vector]
%   * |poissonRatio|      % Poisson's ratios [N×1 vector]
%   * N: Number of shaft segments
%
% * |nodeDistance|        % Node positions [cell array]:
%   * {i} = [d₁ d₂ ... dₘ] % Axial positions for shaft i [m]
%   * m: Number of nodes per shaft
%
%% Output Arguments
% * |M|  % Global mass matrix [sparse n×n]
% * |K|  % Global stiffness matrix [sparse n×n]
% * |G|  % Global gyroscopic matrix [sparse n×n]
% * |N|  % Global Transient matrix [sparse n×n]
% * |Fg| % Gravity force vector [n×1]
%   * n: Total DOF of rotor system
%
%% Formulation
% Based on Timoshenko beam theory for annular shafts:
% 1. Element Matrices:
%   * Computed per shaft segment via |shaftElement|
%   * Accounts for:
%     - Translational and rotational inertia
%     - Shear deformation effects
%     - Rotary inertia
% 2. Geometric Nonlinearity:
%   * Captures stress-stiffening effects via |N| matrix
%
%% Assembly Process
% 1. Element-Level Generation:
%   * For each shaft segment, computes:
%     - Mass matrix (Me)
%     - Stiffness matrix (Ke)
%     - Gyroscopic matrix (Ge)
%     - Transient matrix (Ne)
%     - Gravity vector (Fge)
% 2. Shaft-Level Assembly:
%   * Combines elements within each shaft using |assembleLinear|
%   * Maintains node connectivity
% 3. Global Assembly:
%   * Combines multi-shaft matrices diagonally
%   * Handles shaft-to-shaft connections
%
%% Implementation Details
% 1. Input Validation:
%   * Verifies shaft count matches node distance cells
%   * Checks physical parameter consistency
% 2. Element Processing:
%   * Computes element length from node positions
%   * Passes segment properties to |shaftElement|
% 3. Matrix Conditioning:
%   * Uses sparse matrix storage for efficiency
%
%% Example
% % Configure shaft parameters
% shaftCfg = struct('amount', 2, ...
%                  'dofOfEachNodes', [4; 4], ...
%                  'outerRadius', [0.05; 0.04], ...
%                  'innerRadius', [0; 0], ...
%                  'density', [7850; 7850], ...
%                  'elasticModulus', [210e9; 210e9], ...
%                  'rayleighDamping', [0, 1e-6],...
%                  'poissonRatio', [0.3; 0.3]);
% % Define node positions
% nodePos = {linspace(0, 1, 10), linspace(0, 0.8, 8)}; 
% % Generate FEM matrices
% [M, K, G] = femShaft(shaftCfg, nodePos);
%
%% See Also
% shaftElement, assembleLinear, femDisk, femBearing
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