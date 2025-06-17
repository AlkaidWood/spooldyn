%BEARINGELEMENTINTER Generate stiffness/damping matrices for non-mass intermediate bearings
%
% Syntax:
%   [Ke, Ce] = bearingElementInter(ABearing)
%
% Input Arguments:
%   ABearing - Bearing element configuration structure with fields:
%       .dofOnShaftNode: [1×2 double]     DOFs count on connected shaft nodes
%       .stiffness: double                Direct stiffness in principal directions
%       .stiffnessVertical: double        Vertical stiffness (optional, same as stiffness by default)
%       .damping: double                  Direct damping in principal directions
%       .dampingVertical: double          Vertical damping (optional, same as damping by default)
%
% Output Arguments:
%   Ke - [4×4 cell array]                 Element stiffness matrix partitions
%   Ce - [4×4 cell array]                 Element damping matrix partitions
%
% Description:
%   Creates partitioned stiffness and damping matrices for intermediate bearings
%   without concentrated mass. Matrices are generated in local coordinates and
%   partitioned for connection between two shaft nodes.
%
% Notes:
%   - Throws error if multiple stiffness/damping values are provided
%   - Vertical stiffness/damping defaults to horizontal values if not specified
%   - Matrix partitions follow node connectivity order [start_node, end_node]
%
% Example:
%   generate K11 K12 K21 K22 C11 C12 C21 C22 for inter-shaft bearing without
%   % mass
%   AInterbearing.dofOnShaftNode = [4, 4];
%   AInterbearing.stiffness = 1e6;
%   AInterbearing.stiffnessVertical = 1e6;
%   AInterbearing.damping = 200;
%   AInterbearing.dampingVertical = 300;
%   [Ke, Ce] = bearingElementInter(AInterbearing);
%
% See also BEARINGELEMENTINTERMASS, ADDELEMENTIN
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.

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