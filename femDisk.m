%% femDisk - Generate FEM matrices for disk components in rotor systems
%
% This function assembles global mass, gyroscopic, and transient matrices 
% along with gravity and eccentricity vectors for disk elements in rotor 
% dynamics models.
%
%% Syntax
%  [M, G, N, Q, Fg, E] = femDisk(Disk, nodeDof)
%
%% Description
% |femDisk| constructs finite element matrices for disk components in 
% rotor systems. The function:
% * Computes disk mass and inertia properties
% * Generates mass and gyroscopic matrices
% * Creates gravity and eccentricity force vectors
% * Supports multiple disk configurations
%
%% Input Arguments
% * |Disk| - Disk properties structure:
%   * |amount|              % Number of disks (scalar)
%   * |dofOfEachNodes|      % DOF per mounted node [N×1 vector]
%   * |radius|              % Outer radii [m] [N×1 vector]
%   * |thickness|           % Axial thicknesses [m] [N×1 vector]
%   * |density|             % Material densities [kg/m³] [N×1 vector]
%   * |positionOnShaftNode| % Mounting node indices [N×1 vector]
%   * N: Number of disks
%
% * |nodeDof| - DOF counts per system node [M×1 vector], M = number of nodes
%
%% Output Arguments
% * |M|  % Global mass matrix [sparse n_total×n_total]
% * |G|  % Global gyroscopic matrix [sparse n_total×n_total]
% * |N|  % Nonlinear matrix [sparse n_total×n_total]
% * |Q|  % Unbalance force vector [n_total×1] (currently zeros)
% * |Fg| % Gravity force vector [n_total×1]
% * |E|  % Mass eccentricities [m] [N×1 vector]
% * |EPhase| % Phase of mass eccentricities [rad] [N×1 vector]
%   * n_total: Total DOF of rotor system = sum(nodeDof)
%
%% Matrix Assembly Process
% 1. Element Generation:
%    * For each disk, calls |diskElement| to compute:
%      - Mass matrix (Me)
%      - Gyroscopic matrix (Ge)
%      - Transient matrix (Ne)
%      - Gravity force vector (Fge)
%      - Eccentricity value (E)
% 2. Global Matrix Initialization:
%    * Creates zero matrices of size sum(nodeDof)
% 3. Position Mapping:
%    * Determines DOF positions using |findIndex|
% 4. Assembly:
%    * Adds each disk's matrices to global positions via |addElementIn|
%
%% Physical Modeling
% * Mass Calculation:
%   $ m = \pi \rho t (r_o^2 - r_i^2) $
% * Inertia Terms:
%   $ I_d = \frac{1}{12}m(3(r_o^2 + r_i^2) + t^2) $ (Diametral)
%   $ I_p = \frac{1}{2}m(r_o^2 + r_i^2) $ (Polar)
% * Matrix Structures:
%   * Mass matrix combines translational and rotational terms
%   * Gyroscopic matrix accounts for polar inertia effects
%
%% Implementation Notes
% * Eccentricity Handling:
%   * Returns eccentricity values but doesn't incorporate in force vector
%   * Unbalance force vector (Q) currently returns zeros
% * Position Mapping:
%   * Uses |findIndex| for DOF position calculation
% * Matrix Assembly:
%   * Utilizes |addElementIn| for efficient sparse matrix construction
%
%% Example
% % Configure disk parameters
% diskCfg = struct('amount', 2, ...
%                  'dofOfEachNodes', [4; 4], ...
%                  'outerRadius', [0.15; 0.12], ...
%                  'innerRadius', [0; 0], ...
%                  'thickness', [0.025; 0.02], ...
%                  'density', [7850; 7850], ...
%                  'eccentricity', [1e-3; 1e-3],...
%                  'positionOnShaftNode', [3; 5]);
% % System DOF configuration
% nodeDOF = [4,4,4,4,4,4,4,4,4,4]'; 
% % Generate disk matrices
% [M, G, ~, ~, Fg, Ecc] = femDisk(diskCfg, nodeDOF);
%
%% See Also
% diskElement, addElementIn, findIndex, femShaft, femBearing
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%



function [M, G, N, Q, Fg, E, EPhase] = femDisk(Disk,nodeDof)


% generate elements
Me = cell(Disk.amount,1); 
Ge = cell(Disk.amount,1);
Ne = cell(Disk.amount,1);
Fge = cell(Disk.amount,1);
E = zeros(Disk.amount,1);
EPhase = zeros(Disk.amount,1);
Temporary = rmfield(Disk,'amount'); % for extract part of data of Shaft

for iDisk = 1:1:Disk.amount
    % get the information of ith Disk
    ADisk = getStructPiece(Temporary,iDisk,[]);
    % generate elements
    [Me{iDisk}, Ge{iDisk}, Ne{iDisk}, Fge{iDisk}, E(iDisk), EPhase(iDisk)] = diskElement(ADisk); 
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