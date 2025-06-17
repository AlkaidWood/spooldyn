%% FEMDISK - Generate FEM matrices for disk components
% Assembles global mass, gyroscopic, and N-matrix matrices with gravity 
% and eccentricity vectors for disk elements in rotor systems.
%
%% Syntax
%   [M, G, N, Q, Fg, E] = femDisk(Disk, nodeDof)
%
%% Description
% |FEMDISK| performs finite element matrix assembly for concentrated disk 
% components. Handles:
% * Translational/rotational inertia
% * Gyroscopic effects
% * Mass unbalance forces
% * Gravity loading
%
%% Input Arguments
% *Disk* - Disk properties structure:
%   .amount              % Number of disks (scalar)
%   .dofOfEachNodes     % [N×1] DOF per mounted node
%   .radius             % [N×1] Outer radii [m]
%   .thickness          % [N×1] Axial thicknesses [m]
%   .density            % [N×1] Material densities [kg/m³]
%   .positionOnShaftNode% [N×1] Mounting node indices
%
% *nodeDof*             % [M×1] DOF count per system node
%
%% Output Arguments
% *M*      % Global mass matrix (n×n sparse)
% *G*      % Global gyroscopic matrix (n×n sparse)
% *N*      % Nonlinear matrix (n×n sparse)
% *Q*      % Unbalance force vector (n×1 zeros)
% *Fg*     % Gravity force vector (n×1)
% *E*      % [N×1] Mass eccentricities [m]
%
%% Algorithm
% 1. Element-level matrix generation for each disk:
%    - Uses concentrated mass formulation
%    - Calculates polar/diametral inertia
% 2. Global matrix assembly:
%    - Maps disk elements to system DOFs
%    - Aggregates using sparse matrix addition
%
%% Example
% % Generate disk matrices
% diskParams = inputEssentialParameterBO().Disk;
% nodeDOF = [4;4;...]; % System DOF configuration
% [M_disk, G_disk] = femDisk(diskParams, nodeDOF);
%
%% See Also
% diskElement, addElementIn, femShaft
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%



function [M, G, N, Q, Fg, E] = femDisk(Disk,nodeDof)


% generate elements
Me = cell(Disk.amount,1); 
Ge = cell(Disk.amount,1);
Ne = cell(Disk.amount,1);
Fge = cell(Disk.amount,1);
E = zeros(Disk.amount,1);
Temporary = rmfield(Disk,'amount'); % for extract part of data of Shaft

for iDisk = 1:1:Disk.amount
    % get the information of ith Disk
    ADisk = getStructPiece(Temporary,iDisk,[]);
    % generate elements
    [Me{iDisk}, Ge{iDisk}, Ne{iDisk}, Fge{iDisk}, E(iDisk)] = diskElement(ADisk); 
end

%%

% generate global matrices and vector
dofNum = sum(nodeDof);
M = zeros(dofNum, dofNum);
G = zeros(dofNum, dofNum);
N = zeros(dofNum, dofNum);
Fg = zeros(dofNum, 1);

%%

% calculate the position of disk element in global matrix
diskOnDofPosition = findIndex(Disk.positionOnShaftNode,nodeDof);

%%

% put disk elements into global matrices
for iDisk = 1:1:Disk.amount
   M = addElementIn( M, Me{iDisk}, diskOnDofPosition(iDisk, :) );
   G = addElementIn( G, Ge{iDisk}, diskOnDofPosition(iDisk, :) );
   N = addElementIn( N, Ne{iDisk}, diskOnDofPosition(iDisk, :) );
   Fg = addElementIn(Fg, Fge{iDisk}, [diskOnDofPosition(iDisk,1),1]);
end

Q = zeros(dofNum,1);

end