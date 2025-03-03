%% bearingElementMass
% generate the mass, stiffness, damping matrix, gravity vector of a bearing
% element with mass
%% Syntax
% [Me, Ke, Ce, Fge] = bearingElementMass(AMBearing)
%% Description
% AMBearing is a struct saving the physical parameters of a bearing element
% with fields: dofOfEachNodes, stiffness, damping, mass, dofOnShaftNode
%
% Me, Ke, Ce are mass, stiffness, damping cell of a bearing element. 
% (n*n, n is the number of dofs on this bearing element)
% 
% Fge is gravity vector in column
%% Symbols
% m: mass of bearing
% 
% k: stiffness of bearing
%
% c: damping of bearing


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