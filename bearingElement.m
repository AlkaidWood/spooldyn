%% bearingElement - Generate stiffness/damping matrices for massless bearings
%
% This function constructs local stiffness and damping matrices for bearing 
% elements without concentrated mass in rotor dynamics models.
%
%% Syntax
%  [Ke, Ce] = bearingElement(ANBearing)
%
%% Description
% |bearingElement| calculates stiffness and damping matrices for massless 
% bearing elements. The function:
% * Validates bearing property inputs
% * Constructs orthotropic stiffness/damping matrices
% * Automatically expands matrices to match nodal DOF count
%
%% Input Arguments
% * |ANBearing| - Bearing properties structure with fields:
%   * |dofOnShaftNode|   % DOF count per bearing node (scalar integer)
%   * |stiffness|         % Horizontal stiffness coefficient [N/m]
%   * |stiffnessVertical| % Vertical stiffness coefficient [N/m]
%   * |damping|           % Horizontal damping coefficient [N·s/m]
%   * |dampingVertical|    % Vertical damping coefficient [N·s/m]
%
%% Output Arguments
% * |Ke| - Local stiffness matrix (n×n), where n = dofOnShaftNode
% * |Ce| - Local damping matrix (n×n), where n = dofOnShaftNode
%
%% Matrix Construction Rules
% 1. Orthotropic modeling:
%    * Horizontal and vertical stiffness/damping treated independently
%    * Zero coupling between directions
% 2. Matrix expansion:
%    * Stiffness/damping terms placed in corresponding DOF positions
%    * Remaining DOF filled with zeros
% 3. Zero value handling:
%    * Zero stiffness/damping values are filtered out
%    * Preserves non-zero values only
%
%% Example
% % Create massless bearing parameters
% bearing = struct('dofOnShaftNode', 4, ...   % Correct field name
%                  'stiffness', 1e8, ...
%                  'stiffnessVertical', 1.2e8, ...
%                  'damping', 500, ...
%                  'dampingVertical', 600);
% % Generate stiffness and damping matrices
% [Ke, Ce] = bearingElement(bearing);
%
%% See Also
% bearingElementMass, diskElement, shaftElement, assemblyGlobalMatrix
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function [Ke, Ce] = bearingElement(ANBearing)

%%
% check input stiffness and damping
ANBearing.stiffness(find(ANBearing.stiffness==0)) = [];
ANBearing.damping(find(ANBearing.damping==0)) = [];
if isempty(ANBearing.stiffness)
    ANBearing.stiffness = 0;
else
    % check length
    if length(ANBearing.stiffness) ~= 1
        error('too much input stiffness for bearing without mass')
    end
end

if isempty(ANBearing.damping)
    ANBearing.damping = 0;
else
    % check length
    if length(ANBearing.damping) ~= 1
        error('too much input damping for bearing without mass')
    end
end

%%
% constants
kV = ANBearing.stiffness; % V direction: horizontal
kW = ANBearing.stiffnessVertical; % W direction: vertical
cV = ANBearing.damping;
cW = ANBearing.dampingVertical;
dof = ANBearing.dofOnShaftNode;

%%
% generate stiffness matrix of bearing element
Ke = [ kV, 0;
       0, kW ];
Ke = blkdiag( Ke,zeros(dof - length(Ke)) ); % expand stiffness matrix

%%
% generate damping matrix of bearing element
Ce = [ cV, 0;
       0, cW ];
Ce = blkdiag( Ce,zeros(dof - length(Ce)) ); % expand damping matrix

end