%% BEARINGELEMENTMASS - Generate FEM matrices for bearings with mass
% Constructs mass, stiffness, damping matrices and gravity vector for 
% bearing elements with concentrated masses in rotor dynamics systems.
%
%% Syntax
%   [Me, Ke, Ce, Fge] = bearingElementMass(AMBearing)
%
%% Description
% |BEARINGELEMENTMASS| calculates local matrices for mass-bearing elements 
% using multi-node modeling. Handles:
% * Distributed stiffness/damping between mass nodes
% * Gravity force integration
% * Orthotropic bearing properties (horizontal/vertical directions)
%
%% Input Arguments
% *AMBearing* - Bearing properties structure:
%   .dofOfEachNodes    % [1×K] DOF per node
%   .stiffness         % [1×M] Horizontal stiffness values [N/m]
%   .stiffnessVertical % [1×M] Vertical stiffness values [N/m]
%   .damping           % [1×M] Horizontal damping values [Ns/m]
%   .dampingVertical   % [1×M] Vertical damping values [Ns/m]
%   .mass              % [1×K] Concentrated masses [kg] (K ≤ M-1)
%   .dofOnShaftNode    % DOF count at shaft connection node
%
%% Output Arguments
% *Me*     % Mass matrix cell array (2×2 cell of matrices)
% *Ke*     % Stiffness matrix cell array (2×2 cell of matrices)
% *Ce*     % Damping matrix cell array (2×2 cell of matrices)
% *Fge*    % Gravity force vector [N] (n×1)
%
%% Algorithm
% 1. Matrix assembly stages:
%    a) Shaft connection node stiffness/damping
%    b) Inter-mass node stiffness/damping links
%    c) Mass matrix diagonal assembly
% 2. Cell matrix organization:
%    - Partitioned into shaft/bearing submatrices
%    - Enables direct global matrix insertion
%
%% Example
% % Create mass-bearing parameters
% bearingProps = struct('dofOfEachNodes', [4 2], ...
%                      'stiffness', [1e8, 5e7], ...
%                      'stiffnessVertical', [1.2e8, 6e7], ...
%                      'damping', [500, 300], ...
%                      'dampingVertical', [600, 350], ...
%                      'mass', [3.5, 2.1], ...
%                      'dofOnShaftNode', 4);
% [Me, Ke, Ce, Fg] = bearingElementMass(bearingProps);
%
%% See Also
% femBearing, bearingElement, assembleLinear
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function [Me, Ke, Ce, Fge] = bearingElementMass(AMBearing)

%% initial
% constants
m = AMBearing.mass;
kV = AMBearing.stiffness; % V direction: horizontal
kW = AMBearing.stiffnessVertical; % W direction: vertical
cV = AMBearing.damping;
cW = AMBearing.dampingVertical;
dofShaft = AMBearing.dofOnShaftNode;
dofBearing = AMBearing.dofOfEachNodes;
m = m(m~=0); % get the non-zero mass
massNum = length(m);
if length(AMBearing.mass)~=massNum
    kV = kV(1:massNum+1);
    kW = kW(1:massNum+1);
    cV = cV(1:massNum+1);
    cW = cW(1:massNum+1);
    dofBearing = dofBearing(1:massNum);% get first n+1 sitffness and damping, n is the number of the non-zero mass of bearings
end
dofElement = [dofShaft, dofBearing];
dofNum = sum(dofElement);
dofBearingNum = sum(dofBearing);

% initial
K = zeros(dofNum, dofNum);
C = zeros(dofNum, dofNum);
Fge = zeros(dofBearingNum, 1);


%% add part of the shaft (stiffness and damping)
% stiffness matrix
Kin = [ kV(1), 0;...
        0, kW(1) ];
K = addElementIn(K, Kin, [1,1]);
K = addElementIn(K, -Kin, [1,dofShaft+1]);

% damping matrix
Cin = [ cV(1), 0;...
        0, cW(1) ];
C = addElementIn(C, Cin, [1,1]);
C = addElementIn(C, -Cin, [1,dofShaft+1]);


%% add part of the mass bearing (stiffness and damping)

for im = 1:1:massNum
    isLast = im==massNum; % boolean, last element
    isFirst = im==1;
    if isLast&&isFirst 
        [Kn, Cn] = mn(kV(1),kV(2),cV(1),cV(2),kW(1),kW(2),cW(1),cW(2),dofShaft,dofBearing);
        K = addElementIn(K, Kn, [1,1]);
        C = addElementIn(C, Cn, [1,1]);
    elseif isFirst&&~isLast
        [Kj, Cj] = mj(kV(1),kV(2),cV(1),cV(2),kW(1),kW(2),cW(1),cW(2),dofShaft,dofBearing(1),dofBearing(2));
        K = addElementIn(K, Kj, [1,1]);
        C = addElementIn(C, Cj, [1,1]);
    elseif isLast&&~isFirst
        [Kn, Cn] = mn(kV(end-1),kV(end),cV(end-1),cV(end),kW(end-1),kW(end),cW(end-1),cW(end),dofBearing(end-1),dofBearing(end));
        dofHere = sum(dofElement(1:end-2))+1;
        K = addElementIn(K, Kn, [dofHere,dofHere]);
        C = addElementIn(C, Cn, [dofHere,dofHere]);
    else
        [Kj, Cj] = mj(kV(im),kV(im+1),cV(im),cV(im+1),kW(im),kW(im+1),cW(im),cW(im+1),dofBearing(im-1),dofBearing(im),dofBearing(im+1));
        dofHere = sum(dofElement(1:im-1))+1;
        K = addElementIn(K, Kj, [dofHere,dofHere]);
        C = addElementIn(C, Cj, [dofHere,dofHere]);
    end % end if
end % end for im


%% divide into 4 sub-matrix (stiffness and damping)

Ke = mat2cell(K, [dofShaft, dofBearingNum], [dofShaft, dofBearingNum]);
Ce = mat2cell(C, [dofShaft, dofBearingNum], [dofShaft, dofBearingNum]);


%% mass

Min = zeros(sum(dofBearingNum));
for im = 1:1:massNum
    Mi = [ m(im), 0;...
            0,     m(im)];
    dofHere = sum(dofBearing(1:im-1))+1;
    Min = addElementIn(Min, Mi, [dofHere, dofHere]);
end

M11 = zeros(dofShaft);      M12 = zeros(dofShaft, dofBearingNum);
M21 = M12';                 M22 = zeros(dofBearingNum);

M22 = addElementIn(M22, Min, [1,1]);

% output
Me = {M11, M12;...
      M21, M22 };

%% gravity

for im=1:1:massNum
    FgeHere = -9.8*m(im);
    indexHere = 2 + (im-1)*dofBearing;
    Fge(indexHere) = FgeHere;
end
  
%% sub-function
    
    % sub-function 1
    function [Kn, Cn] = mn(kn,kn1,cn,cn1,knW,knW1,cnW,cnW1,dof1,dof2)
        dofNum1 = dof1+dof2;
        % initial
        Kn = zeros(dofNum1);
        Cn = zeros(dofNum1);
        % construct 
        Kn1 = [-kn, 0;...
               0,   -knW];
        Kn2 = [kn+kn1, 0;...
               0,      knW+knW1];
        Cn1 = [-cn, 0;...
               0,   -cnW];
        Cn2 = [cn+cn1, 0;...
               0,      cnW+cnW1];
        % assembly
        Kn = addElementIn(Kn, Kn1, [dof1+1, 1]);
        Kn = addElementIn(Kn, Kn2, [dof1+1, dof1+1]);
        Cn = addElementIn(Cn, Cn1, [dof1+1, 1]);
        Cn = addElementIn(Cn, Cn2, [dof1+1, dof1+1]);
    end


    % sub-function 2
    function [Kj, Cj] = mj(kj,kj1,cj,cj1,kjW,kjW1,cjW,cjW1,dof1,dof2,dof3)
        dofNum1 = dof1 + dof2 + dof3;
        % initial
        Kj = zeros(dofNum1);
        Cj = zeros(dofNum1);
        % construct
        Kj1 = [-kj, 0;...
               0,   -kjW];
        Kj2 = [kj+kj1, 0;...
               0,      kjW+kjW1];
        Kj3 = [-kj1, 0;...
               0,   -kjW1];  
        Cj1 = [-cj, 0;...
               0,   -cjW];
        Cj2 = [cj+cj1, 0;...
               0,      cjW+cjW1];
        Cj3 = [-cj1, 0;...
               0,   -cjW1];
        % assembly
        Kj = addElementIn(Kj, Kj1, [dof1+1, 1]);
        Kj = addElementIn(Kj, Kj2, [dof1+1, dof1+1]);
        Kj = addElementIn(Kj, Kj3, [dof1+1, dof1+dof2+1]);
        Cj = addElementIn(Cj, Cj1, [dof1+1, 1]);
        Cj = addElementIn(Cj, Cj2, [dof1+1, dof1+1]);
        Cj = addElementIn(Cj, Cj3, [dof1+1, dof1+dof2+1]);
    end
end  