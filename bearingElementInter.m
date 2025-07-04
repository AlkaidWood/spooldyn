%% bearingElementInter - Generate stiffness/damping matrices for non-mass intermediate bearings
%
% This function constructs partitioned stiffness and damping matrices for 
% intermediate bearings without concentrated mass connecting two shaft nodes.
%
%% Syntax
%  [Ke, Ce] = bearingElementInter(ABearing)
%
%% Description
% |bearingElementInter| calculates partitioned stiffness and damping matrices 
% for massless intermediate bearings connecting two shaft nodes. The function:
% * Models orthotropic bearing properties in local coordinates
% * Creates partitioned matrices for node-node connectivity
% * Handles different DOF counts on connected shaft nodes
%
%% Input Arguments
% * |ABearing| - Bearing properties structure:
%   * |dofOnShaftNode|    % DOF counts on connected shaft nodes [1×2 vector]
%   * |stiffness|         % Horizontal stiffness coefficient [N/m]
%   * |damping|           % Horizontal damping coefficient [N·s/m]
%   * |stiffnessVertical| % Vertical stiffness coefficient [N/m] (optional, defaults to horizontal stiffness)
%   * |dampingVertical|   % Vertical damping coefficient [N·s/m] (optional, defaults to horizontal damping)
%
%% Output Arguments
% * |Ke| - Partitioned stiffness matrix (2×2 cell array):
%   * |Ke{1,1}| - Stiffness matrix for start node DOF
%   * |Ke{1,2}| - Coupling stiffness matrix between start and end nodes
%   * |Ke{2,1}| - Coupling stiffness matrix between end and start nodes
%   * |Ke{2,2}| - Stiffness matrix for end node DOF
% * |Ce| - Partitioned damping matrix (2×2 cell array with same structure)
%
%% Matrix Construction Rules
% 1. Core matrix components:
%   * Stiffness core: |Kin = [kV, 0; 0, kW]|
%   * Damping core: |Cin = [cV, 0; 0, cW]|
% 2. Partitioning:
%   * |K11| = |Kin| added to start node DOF positions
%   * |K12| = |-Kin| added to start-to-end coupling positions
%   * |K21| = |-Kin| added to end-to-start coupling positions
%   * |K22| = |Kin| added to end node DOF positions
% 3. Orthotropic modeling:
%   * Horizontal and vertical components remain uncoupled
%
%% Default Behavior
% * If |stiffnessVertical| not provided, uses same value as |stiffness|
% * If |dampingVertical| not provided, uses same value as |damping|
% * Zero values are filtered from stiffness/damping inputs
%
%% Example
% % Configure intermediate bearing between two shafts
% bearingCfg.dofOnShaftNode = [4, 4];   % DOF for both connected shafts
% bearingCfg.stiffness = 1e6;            % Horizontal stiffness
% bearingCfg.stiffnessVertical = 1.2e6;   % Vertical stiffness
% bearingCfg.damping = 500;             % Horizontal damping
% bearingCfg.dampingVertical = 600;     % Vertical damping
% % Generate partitioned matrices
% [Ke, Ce] = bearingElementInter(bearingCfg);
%
%% See Also
% bearingElement, bearingElementInterMass, addElementIn
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function [Ke, Ce] = bearingElementInter(ABearing)
%%
% check input stiffness and damping
ABearing.stiffness(find(ABearing.stiffness==0)) = [];
ABearing.damping(find(ABearing.damping==0)) = [];
if isempty(ABearing.stiffness)
    ABearing.stiffness = 0;
else
    % check length
    if length(ABearing.stiffness) ~= 1
        error('too much input stiffness for bearing without mass')
    end
end

if isempty(ABearing.damping)
    ABearing.damping = 0;
else
    % check length
    if length(ABearing.damping) ~= 1
        error('too much input damping for bearing without mass')
    end
end

%%

% constants
kV = ABearing.stiffness;
cV = ABearing.damping;
kW = ABearing.stiffnessVertical;
cW = ABearing.dampingVertical;
dof1 = ABearing.dofOnShaftNode(1);
dof2 = ABearing.dofOnShaftNode(2);

%%

% stiffness matrix
Kin = [ kV, 0;...
        0, kW ];
 
K11 = zeros(dof1);      K12 = zeros(dof1, dof2);
K21 = K12';             K22 = zeros(dof2);

K11 = addElementIn(K11,Kin,[1,1]);  K12 = addElementIn(K12,-Kin,[1,1]);
K21 = addElementIn(K21,-Kin,[1,1]); K22 = addElementIn(K22,Kin,[1,1]);

Ke = {K11, K12;...
      K21, K22 };
  
%%

% damping matrix
Cin = [ cV, 0;...
        0, cW ];
 
C11 = zeros(dof1);      C12 = zeros(dof1, dof2);
C21 = C12';             C22 = zeros(dof2);

C11 = addElementIn(C11,Cin,[1,1]);  C12 = addElementIn(C12,-Cin,[1,1]);
C21 = addElementIn(C21,-Cin,[1,1]); C22 = addElementIn(C22,Cin,[1,1]);

Ce = {C11, C12;...
      C21, C22 };

end