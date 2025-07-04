%% diskElement - Generate FEM matrices for disk components in rotor dynamics
%
% This function computes mass, gyroscopic, and transient matrices along with 
% gravity and eccentricity force vectors for disk elements in rotor systems.
%
%% Syntax
%  [Me, Ge, Ne, Fge, Ee] = diskElement(ADisk)
%
%% Description
% |diskElement| calculates finite element matrices for disk components 
% using thin disk approximation. The function:
% * Computes disk mass and inertia properties
% * Constructs mass and gyroscopic matrices
% * Generates gravity and eccentricity force vectors
% * Supports mass unbalance modeling
%
%% Input Arguments
% * |ADisk| - Disk properties structure:
%   * |dofOfEachNodes|    % DOF count at mounting node (scalar)
%   * |outerRadius|       % Outer radius [m]
%   * |innerRadius|       % Inner radius [m]
%   * |density|           % Material density [kg/m³]
%   * |thickness|         % Axial thickness [m]
%   * |eccentricity|      % Mass eccentricity [m]
%
%% Output Arguments
% * |Me|  - Mass matrix (4×4) combining translational and rotational inertia
% * |Ge|  - Gyroscopic matrix (4×4) accounting for polar inertia effects
% * |Ne|  - Transient matrix (4×4) for taking into account transient
%           dynamic behavior
% * |Fge| - Gravity force vector (4×1) [N]
% * |Ee|  - Eccentricity force magnitude [N]
%
%% Physical Formulation
% 1. Mass Calculation:
%    $ m = \pi \rho t (r_o^2 - r_i^2) $
% 2. Inertia Terms:
%    $ I_d = \frac{1}{12}m(3(r_o^2 + r_i^2) + t^2) $ (Diametral inertia)
%    $ I_p = \frac{1}{2}m(r_o^2 + r_i^2) $ (Polar inertia)
% 3. Matrix Construction:
%    * Mass matrix: Combines translational (MT) and rotational (MR) terms
%    * Gyroscopic matrix: Based on polar inertia
%
%% Matrix Structures
% 1. Mass Matrix (Me):
%    $ Me = \begin{bmatrix}
%        m & 0 & 0 & 0 \\
%        0 & m & 0 & 0 \\
%        0 & 0 & I_d & 0 \\
%        0 & 0 & 0 & I_d
%    \end{bmatrix} $
%
% 2. Gyroscopic Matrix (Ge):
%    $ Ge = \begin{bmatrix}
%        0 & 0 & 0 & 0 \\
%        0 & 0 & 0 & 0 \\
%        0 & 0 & 0 & -I_p \\
%        0 & 0 & I_p & 0
%    \end{bmatrix} $
%
% 3. Gravity Vector (Fge):
%    $ Fge = \begin{bmatrix} 0 \\ -mg \\ 0 \\ 0 \end{bmatrix} $
%
% 4. Eccentricity Force (Ee):
%    $ Ee = m \cdot e $
%
%% Implementation Notes
% * Uses thin disk approximation (axial thickness << diameter)
% * Accounts for both diametral and polar inertia effects
% * Gravity acts in negative Y-direction (vertical downward)
% * Eccentricity modeled as static unbalance force
%
%% Example
% % Configure disk parameters
% diskCfg = struct('dofOfEachNodes', 4, ...
%                  'outerRadius', 0.15, ...
%                  'innerRadius', 0.05, ...
%                  'density', 7850, ...
%                  'thickness', 0.025, ...
%                  'eccentricity', 0.0001);
% % Generate disk element matrices
% [Me, Ge, ~, Fg, Ecc] = diskElement(diskCfg);
%
%% See Also
% shaftElement, bearingElement, assembleGlobalMatrix
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function [Me, Ge, Ne, Fge, Ee] = diskElement(ADisk)

% check the input
fieldName = {'dofOfEachNodes', 'outerRadius', 'innerRadius', 'density', 'thickness'};
hasFieldName = isfield(ADisk, fieldName);
if length(hasFieldName) ~= sum(hasFieldName)
    error('Incorrect field names for input struct')
end

%%

% calculate the constants
r1 = ADisk.innerRadius;
r2 = ADisk.outerRadius;
thickness = ADisk.thickness;
rho = ADisk.density;
eDisk = ADisk.eccentricity;
m = (r2^2 - r1^2) * pi* thickness * rho;
Id = 1/12 * m * (3*(r1^2+r2^2)+thickness^2);
Ip = 1/2 * m * (r1^2+r2^2);

%%

% mass matrix
MT = [ m, 0, 0, 0;...
       0, m, 0, 0;...
       0, 0, 0, 0;...
       0, 0, 0, 0 ];

MR = [ 0,  0,  0,  0;...
       0,  0,  0,  0;...
       0,  0, Id,  0;...
       0,  0,  0, Id ]; 
   
Me = MT + MR;

%%

% gyrosocpic matrix
Ge = [  0,   0,   0,   0;...
        0,   0,   0,   0;...
        0,   0,   0, -Ip;...
        0,   0,  Ip,   0 ]; 
  
%%

% Ne matrix
Ne = [  0,   0,   0,   0;...
        0,   0,   0,   0;...
        0,   0,   0,   0;...
        0,   0,  Ip,   0 ]; 

%%

% gravity
FgeTotal = m * 9.8; % N
Fge = [0; -FgeTotal; 0; 0];


%%

% eccentricity
Ee = m * eDisk;

end