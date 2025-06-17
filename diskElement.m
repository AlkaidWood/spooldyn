%% DISKELEMENT - Generate element matrices for disk components
% Computes mass, gyroscopic, and N-matrix matrices with gravity vector and
% eccentricity for individual disk elements in rotor dynamics analysis.
%
%% Syntax
%   [Me, Ge, Ne, Fge, Ee] = diskElement(ADisk)
%
%% Description
% |DISKELEMENT| calculates FEM component matrices for disk elements using:
% * Thin disk approximation
% * Rotational inertia effects
% * Mass unbalance consideration
%
%% Input Arguments
% *ADisk* - Disk properties structure:
%   .dofOfEachNodes    % DOF count at mounting node (scalar)
%   .outerRadius       % Disk outer radius [m]
%   .innerRadius       % Disk inner radius [m]
%   .density           % Material density [kg/m³]
%   .thickness         % Axial thickness [m]
%   .eccentricity      % Mass eccentricity [m]
%
%% Output Arguments
% *Me*     % Element mass matrix (4×4)
% *Ge*     % Element gyroscopic matrix (4×4)
% *Ne*     % Nonlinear matrix (4×4)
% *Fge*    % Gravity force vector (4×1)
% *Ee*     % Eccentricity force component [N×1]
%
%% Formulation
% 1. Mass Calculation:
%    $ m = \pi \rho t (r_o^2 - r_i^2) $
% 2. Inertia Terms:
%    $ I_d = \frac{1}{4}m(r_o^2 + r_i^2) $ (Diametral)
%    $ I_p = \frac{1}{2}m(r_o^2 + r_i^2) $ (Polar)
% 3. Matrix Construction:
%    - Mass matrix combines translational (MT) and rotational (MR) terms
%    - Gyroscopic matrix accounts for polar inertia effects
%
%% Example
% % Create disk parameters
% diskProps = struct('outerRadius', 0.1, 'innerRadius', 0.05, ...
%                    'density', 7850, 'thickness', 0.02, ...
%                    'eccentricity', 1e-4);
% [Me, Ge] = diskElement(diskProps);
%
%% See Also
% femDisk, shaftElement, assembleLinear
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
Id = 1/4 * m * (r1^2+r2^2);
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