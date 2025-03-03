%% bearingElement
% generate the stiffness, damping matrix of a bearing element without mass
%% Syntax
% [Ke, Ce] = bearingElement(ANBearing)
%% Description
% ANBearing is a struct saving the physical parameters of a bearing element
% with fields: dofOfEachNodes, stiffness, damping
%
% Ke, Ce are stiffness, damping matrix of a bearing element. 
% (n*n, n is the number of dofs on this bearing element)
%% Symbols
% k: stiffness of bearing
%
% c: damping of bearing


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