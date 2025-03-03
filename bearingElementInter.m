%% bearingElementInter
% generate the stiffness, damping matrix of a intermediate bearing element 
%% Syntax
% [Ke, Ce] = bearingElementInter(ABearing)
%% Description
% ABearing is a struct saving the physical parameters of a intermediate 
% bearing element with fields: dofOnShaftNode, stiffness, damping
%
% Ke, Ce are stiffness, damping cell of a bearing element. 
% (n*n, n is the number of dofs on this intermediate bearing element)
%% Symbols
% kV: horizontal stiffness of intermediate bearing
%
% kW: vertical stiffness of intermediate bearing
%
% cV: horizontal damping of intermediate bearing
%
% cW: vertical damping of intermediate bearing

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