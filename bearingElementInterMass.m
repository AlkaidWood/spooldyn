%BEARINGELEMENTINTERMASS Generate FEM matrices for mass-bearing elements with multi-DOF
%
% Syntax:
%   [Me, Ke, Ce, Fge] = bearingElementInterMass(AMBearing)
%
% Input Arguments:
%   AMBearing - Mass-bearing configuration structure with fields:
%       .dofOnShaftNode: [1×2 double]     DOFs count on connected shaft nodes
%       .dofOfEachNodes: [1×n double]     DOFs per bearing mass node
%       .mass: [1×n double]               Concentrated masses (m_x, m_y components)
%       .stiffness: [1×(n+1) double]      Inter-mass stiffness components (horizontal)
%       .stiffnessVertical: [1×(n+1) double] Vertical stiffness components
%       .damping: [1×(n+1) double]        Inter-mass damping components (horizontal)
%       .dampingVertical: [1×(n+1) double] Vertical damping components
%
% Output Arguments:
%   Me - [2×2 cell array]                 Partitioned mass matrix components
%   Ke - [1×7 cell array]                 Stiffness matrix partitions:
%       1: Inner shaft, 2,3: Inner-mass coupling
%       4: Outer shaft, 5,6: Outer-mass coupling 
%       7: Mass-chain components
%   Ce - [1×7 cell array]                 Damping matrix partitions (same structure as Ke)
%   Fge - [n×1 double]                    Gravity force vector for mass nodes
%
% Description:
%   Constructs partitioned FEM matrices for intermediate bearings with:
%   - Multiple concentrated masses between shaft segments
%   - Configurable stiffness/damping between mass nodes
%   - Gravity forces for mass components
%   Uses chain topology for mass-bearing connections
%
% Notes:
%   - Automatically handlel mass configurations with different matrix partitions
%   - Vertical stiffness/damping must match horizontal parameter dimensions
%   - Mass components must be specified in connection order
%
% Example:
% bearingConfig = struct('dofOnShaftNode', [4,4], 'mass', [3.5 2.1],...
%   'stiffness', [1e6 8e5 7e5], 'damping', [1e3 8e2 7e2],...
%   'dampingVertical', [1e3 8e2 7e2],...
%   'stiffnessVertical', [1e6 8e5 7e5], 'dofOfEachNodes', [2,2]);
% [Me, Ke, Ce, Fge] = bearingElementInterMass(bearingConfig);
%
% See also BEARINGELEMENTINTER, ADDELEMENTIN
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.

function [Me, Ke, Ce, Fge] = bearingElementInterMass(AMBearing)

%% initial
% constants
m = AMBearing.mass;
kV = AMBearing.stiffness;
cV = AMBearing.damping;
kW = AMBearing.stiffnessVertical;
cW = AMBearing.dampingVertical;
dofShaft = AMBearing.dofOnShaftNode;
dofBearing = AMBearing.dofOfEachNodes;
m = m(m~=0); % get the non-zero mass
massNum = length(m);
if length(AMBearing.mass)~=massNum
    kV = kV(1:massNum+1);
    cV = cV(1:massNum+1);
    kW = kW(1:massNum+1);
    cW = cW(1:massNum+1);
    dofBearing = dofBearing(1:massNum);% get first n+1 sitffness and damping, n is the number of the non-zero mass of bearings
end
dofElement = [dofShaft, dofBearing];
dofNum = sum(dofElement);
dofShaftNum = sum(dofShaft);
dofBearingNum = sum(dofBearing);

% initial
K = zeros(dofNum, dofNum);
C = zeros(dofNum, dofNum);


%% add part of the inner shaft (stiffness and damping)
% stiffness matrix
Kin = [ kV(1), 0;...
        0, kW(1) ];
K = addElementIn(K, Kin, [1,1]);
K = addElementIn(K, -Kin, [1,dofShaftNum+1]);

% damping matrix
Cin = [ cV(1), 0;...
        0, cW(1) ];
C = addElementIn(C, Cin, [1,1]);
C = addElementIn(C, -Cin, [1,dofShaftNum+1]);


%% add part of the outer shaft (stiffness and damping)
% stiffness matrix
Kin = [kV(end), 0;...
       0,      kW(end)];
K = addElementIn(K, Kin, [dofShaft(1)+1, dofShaft(1)+1]);
K = addElementIn(K, -Kin, [dofShaft(1)+1, sum(dofElement(1:end-1))+1]);

% damping matrix
Cin = [cV(end), 0;...
       0,      cW(end)];
C = addElementIn(C, Cin, [dofShaft(1)+1, dofShaft(1)+1]);
C = addElementIn(C, -Cin, [dofShaft(1)+1, sum(dofElement(1:end-1))+1]);

%% add part of the mass bearing (stiffness and damping)

for im = 1:1:massNum
    isLast = im==massNum; % boolean, last element
    isFirst = im==1;
    if isLast&&isFirst
        [K1, C1] = mFirstLast(kV(1),kV(2),cV(1),cV(2),kW(1),kW(2),cW(1),cW(2),dofShaft(1),dofShaft(2),dofBearing);
        K = addElementIn(K, K1, [1,1]);
        C = addElementIn(C, C1, [1,1]);
    elseif isFirst&&~isLast
        [Kj, Cj] = mj(kV(1),kV(2),cV(1),cV(2),kW(1),kW(2),cW(1),cW(2),dofShaft(1),dofBearing(1),dofBearing(2));
        Kje = mat2cell(Kj, [dofShaft(1), dofBearing(1)+dofBearing(2)], [dofShaft(1), dofBearing(1)+dofBearing(2)]);
        Cje = mat2cell(Cj, [dofShaft(1), dofBearing(1)+dofBearing(2)], [dofShaft(1), dofBearing(1)+dofBearing(2)]);
        K = addElementIn(K, Kje{2,1}, [dofShaftNum+1,1]); % Kje{1,1}, kje{2,1} are 0 matrix
        K = addElementIn(K, Kje{2,2}, [dofShaftNum+1,dofShaftNum+1]);
        C = addElementIn(C, Cje{2,1}, [dofShaftNum+1,1]);
        C = addElementIn(C, Cje{2,2}, [dofShaftNum+1,dofShaftNum+1]);
    elseif isLast&&~isFirst
        [Kn, Cn] = mn(kV(end-1),kV(end),cV(end-1),cV(end),kW(end-1),kW(end),cW(end-1),cW(end),dofShaft(2),dofBearing(end-1),dofBearing(end));
        Kne = mat2cell(Kn, [dofShaft(2), dofBearing(end-1)+dofBearing(end)], [dofShaft(2), dofBearing(end-1)+dofBearing(end)]);
        Cne = mat2cell(Cn, [dofShaft(2), dofBearing(end-1)+dofBearing(end)], [dofShaft(2), dofBearing(end-1)+dofBearing(end)]);
        dofHere = sum(dofElement(1:end-2))+1;
        K = addElementIn(K, Kne{2,1}, [dofHere,dofShaft(1)+1]); % Kne{1,1}, kne{2,1} are 0 matrix
        K = addElementIn(K, Kne{2,2}, [dofHere,dofHere]);
        C = addElementIn(C, Cne{2,1}, [dofHere,dofShaft(1)+1]); % Kne{1,1}, kne{2,1} are 0 matrix
        C = addElementIn(C, Cne{2,2}, [dofHere,dofHere]);
    else
        [Kj, Cj] = mj(kV(im),kV(im+1),cV(im),cV(im+1),kW(im),kW(im+1),cW(im),cW(im+1),dofBearing(im-1),dofBearing(im),dofBearing(im+1));
        dofHere = sum(dofElement(1:im))+1;
        K = addElementIn(K, Kj, [dofHere,dofHere]);
        C = addElementIn(C, Cj, [dofHere,dofHere]);
    end % end if
end % end for im


%% divide into 7 sub-matrix (stiffness and damping)

% there are 7 non-zeros sub-matrix in this intermediate bearing element
% 1->Inner shaft; 2->Inner shaft with m1 (1,2); 3->Inner shaft with m1
% (2,1); 4->Outer shaft; 5->Outer shaft with mn (2,n); 6->Outer shaft with
% mn (n,2); 7->m1 m2 ... mn
switch massNum
    case 1
        KeTemp = mat2cell(K, [dofShaft, dofBearing], [dofShaft, dofBearing]);
        CeTemp = mat2cell(C, [dofShaft, dofBearing], [dofShaft, dofBearing]);
        Ke = {KeTemp{1,1}, KeTemp{1,3}, KeTemp{3,1}, KeTemp{2,2}, KeTemp{2,3}, KeTemp{3,2}, KeTemp{3,3}};
        Ce = {CeTemp{1,1}, CeTemp{1,3}, CeTemp{3,1}, CeTemp{2,2}, CeTemp{2,3}, CeTemp{3,2}, CeTemp{3,3}};
    case 2
        KeTemp = mat2cell(K, [dofShaft, dofBearing], [dofShaft, dofBearing]);
        KeTemp2 = mat2cell(K, [dofShaftNum,dofBearingNum], [dofShaftNum,dofBearingNum]);
        CeTemp = mat2cell(C, [dofShaft, dofBearing], [dofShaft, dofBearing]);
        CeTemp2 = mat2cell(C, [dofShaftNum,dofBearingNum], [dofShaftNum,dofBearingNum]);
        Ke = {KeTemp{1,1}, KeTemp{1,3}, KeTemp{3,1}, KeTemp{2,2}, KeTemp{2,4}, KeTemp{4,2}, KeTemp2{2,2}};
        Ce = {CeTemp{1,1}, CeTemp{1,3}, CeTemp{3,1}, CeTemp{2,2}, CeTemp{2,4}, CeTemp{4,2}, CeTemp2{2,2}};
    otherwise % dofBearingNum >= 3
        divideIndex = [dofShaft, dofBearing(1), sum(dofBearing(2:end-1)), dofBearing(end)];
        divideIndex2 = [dofShaftNum,dofBearingNum];
        KeTemp = mat2cell(K, divideIndex, divideIndex);
        KeTemp2 = mat2cell(K, divideIndex2, divideIndex2);
        CeTemp = mat2cell(C, divideIndex, divideIndex);
        CeTemp2 = mat2cell(C, divideIndex2, divideIndex2);
        Ke = {KeTemp{1,1}, KeTemp{1,3}, KeTemp{3,1}, KeTemp{2,2}, KeTemp{2,5}, KeTemp{5,2}, KeTemp2{2,2}};
        Ce = {CeTemp{1,1}, CeTemp{1,3}, CeTemp{3,1}, CeTemp{2,2}, CeTemp{2,5}, CeTemp{5,2}, CeTemp2{2,2}};
end


%% mass

Min = zeros(sum(dofBearingNum));
for im = 1:1:massNum
    Mi = [ m(im), 0;...
            0,     m(im)];
    dofHere = sum(dofBearing(1:im-1))+1;
    Min = addElementIn(Min, Mi, [dofHere, dofHere]);
end

M11 = zeros(dofShaftNum);      M12 = zeros(dofShaftNum, dofBearingNum);
M21 = M12';                    M22 = zeros(dofBearingNum);

M22 = addElementIn(M22, Min, [1,1]);

% output
Me = {M11, M12;...
      M21, M22 };


%% gravity

% initial
Fge = zeros(dofBearingNum, 1);

% calculate and output
for im = 1:1:massNum
    FgeHere = -9.8*m(im);
    indexHere = 2 + (im-1)*dofBearing;
    Fge(indexHere) = FgeHere;
end

  
%% sub-function
    
    % sub-function 1
    function [Kn, Cn] = mn(kn,kn1,cn,cn1,knW,knW1,cnW,cnW1,dof1,dof2,dof3)
        dofNum1 = dof1+dof2+dof3;
        % initial
        Kn = zeros(dofNum1);
        Cn = zeros(dofNum1);
        % construct 
        Kn1 = [-kn, 0;...
               0,   -knW];
        Kn2 = [kn+kn1, 0;...
               0,      knW+knW1];
        Kn3 = [-kn1, 0;...
               0,   -knW1];
        Cn1 = [-cn, 0;...
               0,   -cnW];
        Cn2 = [cn+cn1, 0;...
               0,      cnW+cnW1];
        Cn3 = [-cn1, 0;...
               0,   -cnW1];
        % assembly
        Kn = addElementIn(Kn, Kn3, [dof1+dof2+1, 1]);
        Kn = addElementIn(Kn, Kn1, [dof1+dof2+1, dof1+1]);
        Kn = addElementIn(Kn, Kn2, [dof1+dof2+1, dof1+dof2+1]);
        Cn = addElementIn(Cn, Cn3, [dof1+dof2+1, 1]);
        Cn = addElementIn(Cn, Cn1, [dof1+dof2+1, dof1+1]);
        Cn = addElementIn(Cn, Cn2, [dof1+dof2+1, dof1+dof2+1]);
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
    
    % sub-function 3
    function [K1, C1] = mFirstLast(k1, k2, c1, c2, kW1, kW2, cW1, cW2, dof1, dof2, dof3)
        dofNum1 = dof1 + dof2 + dof3;
        % initial
        K1 = zeros(dofNum1);
        C1 = zeros(dofNum1);
        % construct
        K11 = [-k1, 0;...
               0,   -kW1];
        K12 = [-k2, 0;...
               0,   -kW2];
        K13 = [k1+k2, 0;...
               0,     kW1+kW2];
        C11 = [-c1, 0;...
               0,   -cW1];
        C12 = [-c2, 0;...
               0,   -cW2];
        C13 = [c1+c2, 0;...
               0,     cW1+cW2];
        % assembly
        K1 = addElementIn(K1, K11, [dof1+dof2+1, 1]);
        K1 = addElementIn(K1, K12, [dof1+dof2+1, dof1+1]);
        K1 = addElementIn(K1, K13, [dof1+dof2+1, dof1+dof2+1]);
        C1 = addElementIn(C1, C11, [dof1+dof2+1, 1]);
        C1 = addElementIn(C1, C12, [dof1+dof2+1, dof1+1]);
        C1 = addElementIn(C1, C13, [dof1+dof2+1, dof1+dof2+1]);
    end
end  