%% BEARINGELEMENT - Generate stiffness/damping matrices for massless bearings
% Constructs local stiffness and damping matrices for bearing elements 
% without concentrated mass in rotor dynamics systems.
%
%% Syntax
%   [Ke, Ce] = bearingElement(ANBearing)
%
%% Description
% |BEARINGELEMENT| calculates the local stiffness and damping matrices 
% for massless bearing elements. Supports:
% * Orthotropic bearing properties (horizontal/vertical directions)
% * Automatic matrix expansion to match nodal DOF
%
%% Input Arguments
% *ANBearing* - Bearing properties structure:
%   .dofOfEachNodes     % DOF count at bearing node (scalar)
%   .stiffness          % Horizontal stiffness [N/m]
%   .stiffnessVertical  % Vertical stiffness [N/m]
%   .damping            % Horizontal damping [Ns/m]
%   .dampingVertical    % Vertical damping [Ns/m]
%
%% Output Arguments
% *Ke*     % Local stiffness matrix (n×n)
% *Ce*     % Local damping matrix (n×n)
%           % n = ANBearing.dofOfEachNodes
%
%% Algorithm
% 1. Input validation:
%    - Ensures single stiffness/damping value per direction
% 2. Matrix construction:
%    - Creates 2×2 directional matrices
%    - Expands to full DOF size with zero padding
%
%% Example
% % Create massless bearing parameters
% bearingProps = struct('dofOfEachNodes', 4, ...
%                       'stiffness', 1e8, ...
%                       'stiffnessVertical', 1.2e8, ...
%                       'damping', 500, ...
%                       'dampingVertical', 600);
% [Ke, Ce] = bearingElement(bearingProps);
%
%% See Also
% femBearing, bearingElementMass, assembleLinear
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