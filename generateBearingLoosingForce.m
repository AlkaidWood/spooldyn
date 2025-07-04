%% generateBearingLoosingForce - Generate bearing loosening force function
%
% This function creates a MATLAB function file (bearingLoosingForce.m) that 
% implements stiffness and damping switching logic for bearings with clearance 
% faults in rotor dynamics simulations.
%
%% Syntax
%  generateBearingLoosingForce(LoosingBearing, Mesh)
%
%% Description
% |generateBearingLoosingForce| constructs a condition-based switching function 
% for bearing stiffness and damping matrices during clearance faults. The 
% generated function:
% * Monitors specific DOF for clearance threshold violations
% * Switches between normal and loosened bearing matrices
% * Automatically detects bearing locations from mesh data
%
%% Input Arguments
% * |LoosingBearing| - Bearing loosening configuration structure:
%   * |loosingPositionNo| % Position index in bearing chain (scalar)
%   * |interval|          % Clearance threshold [m] (scalar)
%
% * |Mesh| - System discretization structure with fields:
%   * |Node|: [1×N struct] Node properties array
%     .name              % Node identifier
%     .isBearing         % Bearing node flag (logical)
%     .isLoosingBearing  % Loosening bearing flag (logical)
%   * |dofInterval|: [N×2] DOF ranges per node
%   * |nodeNum|          % Total node count (scalar)
%
%% Generated Function (bearingLoosingForce.m)
% Function Signature:
%   [K, C] = bearingLoosingForce(qn, Matrix)
% * Inputs:
%   - |qn|: Current displacement vector
%   - |Matrix|: System matrix structure
% * Outputs:
%   - |K|: Updated stiffness matrix
%   - |C|: Updated damping matrix
%
%% Switching Logic
% The generated function implements:
%   if qn(loosingDof) ∈ [0, interval]
%       K = Matrix.stiffnessLoosing
%       C = Matrix.dampingLoosing
%   else
%       K = Matrix.stiffness
%       C = Matrix.damping
%   end
%
%% Position Handling
% * |loosingPositionNo = 1|:
%   - Monitors shaft-connected node (bearing outer ring)
% * |loosingPositionNo > 1|:
%   - Monitors specified mass node in bearing chain
%
%% Implementation Notes
% 1. Node Identification:
%   * Automatically locates bearing nodes from mesh data
%   * Determines monitoring DOF based on |loosingPositionNo|
% 2. File Management:
%   * Overwrites existing bearingLoosingForce.m
%   * Creates temporary .txt file during generation
% 3. Threshold Handling:
%   * Uses unilateral clearance check (qn ≥ 0 and qn ≤ interval)
%
%% Example
% % Configure bearing loosening parameters
% looseCfg = struct('loosingPositionNo', 2, 'interval', 0.0001);
% % Generate switching function (After modeling)
% generateBearingLoosingForce(looseCfg, Parameter.Mesh);
%
%% Dependencies
% Requires complete mesh data from |meshModel|
%
%% See Also
% generateDynamicEquation, bearingElement, meshModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function generateBearingLoosingForce(LoosingBearing, Mesh)

% check the exist of bearingLoosingForce.m and create .txt
if isfile('bearingLoosingForce.m')
    delete bearingLoosingForce.m
end

blf = fopen('bearingLoosingForce.txt','w');

%%

% write comments, firstly

comments = [...
"%% bearingLoosingForce";...
"% saving the equation of loosing bearing force in this function";...
"%% Syntax";...
"% [K, C] = bearingLoosingForce(qn)";...
"%% Description";...
"% qn: is the displacement at the n-th time";...
"% ";...
"% Matrix: is a struct saving all matrix used in dynamic equation";...
"% ";...
"% K, C: are matrix after loosing";...
" ";...
" "...
];

%%

% function start
functionStart = [...
"function [K, C] = bearingLoosingForce(qn, Matrix)";...
" "...
];

fprintf(blf,'%s\n',comments);
fprintf(blf,'%s\n',functionStart);

%%

% calculate the loosing dof
Node = Mesh.Node;
loosingNodeNo = zeros(1000,1); % initial
loosingNum = 0; % record the mass number of the loosing bearing

% If the LoosingBearing.loosingPositionNo == 1:
% k1,c1 will be change to loosing k,c when the y-displacement of the shaft
% connecting the loosing bearing exceeds the interval
% If the LoosingBearing.loosingPositionNo = j, j > 1:
% kj,cj will be change to loosing k,c when the y-displacement of the
% (j-1)th mass of the loosing bearing exceeds the interval
for iNode = 1:1:Mesh.nodeNum
    if LoosingBearing.loosingPositionNo == 1 % denote that the k1,c1 is loosing
        isElement = Node(iNode).isBearing == false; % this element is shaft
    else % denote the kj,cj is loosing, j>1
        isElement = Node(iNode).isBearing == true; % this element is bearing
    end % end if
    isLoosingBearing = Node(iNode).isLoosingBearing;
    if isElement && isLoosingBearing
        loosingNum = loosingNum + 1;
        loosingNodeNo(loosingNum) = Node(iNode).name;
    end % end if
end % end for
loosingNodeNo = loosingNodeNo(loosingNodeNo~=0); % delete the 0 elements
if LoosingBearing.loosingPositionNo ~= 1
    loosingNodeNo = loosingNodeNo(LoosingBearing.loosingPositionNo-1);
end
loosingDof = Mesh.dofInterval(loosingNodeNo,1) + 1; % loosing direction on y
interval = LoosingBearing.interval;

%%

% function body
functionBody = {...
 ' ';...
['if qn(', num2str(loosingDof), ') >= 0 && qn(', num2str(loosingDof), ') <= ', num2str(interval)];...
 '    K = Matrix.stiffnessLoosing;';...
 '    C = Matrix.dampingLoosing;';...
 'else';...
 '    K = Matrix.stiffness;';...
 '    C = Matrix.damping;';...
 'end';...
 ' ';...
};

functionBody = cell2string(functionBody);

fprintf(blf,'%s\n', functionBody);

%%

% function end
functionEnd = [...
"end";...
" "...
];
fprintf(blf,'%s\n',functionEnd);

%%

% close .txt and transfer .txt -> .m
fclose(blf);
system('rename bearingLoosingForce.txt bearingLoosingForce.m');
end