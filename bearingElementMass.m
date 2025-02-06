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
k = AMBearing.stiffness;
c = AMBearing.damping;
dofShaft = AMBearing.dofOnShaftNode;
dofBearing = AMBearing.dofOfEachNodes;
m = m(m~=0); % get the non-zero mass
massNum = length(m);
if length(AMBearing.mass)~=massNum
    k = k(1:massNum+1);
    c = c(1:massNum+1); 
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
Kin = [ k(1), 0;...
        0, k(1) ];
K = addElementIn(K, Kin, [1,1]);
K = addElementIn(K, -Kin, [1,dofShaft+1]);

% damping matrix
Cin = [ c(1), 0;...
        0, c(1) ];
C = addElementIn(C, Cin, [1,1]);
C = addElementIn(C, -Cin, [1,dofShaft+1]);


%% add part of the mass bearing (stiffness and damping)

for im = 1:1:massNum
    isLast = im==massNum; % boolean, last element
    isFirst = im==1;
    if isLast&&isFirst 
        [Kn, Cn] = mn(k(1),k(2),c(1),c(2),dofShaft,dofBearing);
        K = addElementIn(K, Kn, [1,1]);
        C = addElementIn(C, Cn, [1,1]);
    elseif isFirst&&~isLast
        [Kj, Cj] = mj(k(1),k(2),c(1),c(2),dofShaft,dofBearing(1),dofBearing(2));
        K = addElementIn(K, Kj, [1,1]);
        C = addElementIn(C, Cj, [1,1]);
    elseif isLast&&~isFirst
        [Kn, Cn] = mn(k(end-1),k(end),c(end-1),c(end),dofBearing(end-1),dofBearing(end));
        dofHere = sum(dofElement(1:end-2))+1;
        K = addElementIn(K, Kn, [dofHere,dofHere]);
        C = addElementIn(C, Cn, [dofHere,dofHere]);
    else
        [Kj, Cj] = mj(k(im),k(im+1),c(im),c(im+1),dofBearing(im-1),dofBearing(im),dofBearing(im+1));
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
    function [Kn, Cn] = mn(kn,kn1,cn,cn1,dof1,dof2)
        dofNum1 = dof1+dof2;
        % initial
        Kn = zeros(dofNum1);
        Cn = zeros(dofNum1);
        % construct 
        Kn1 = [-kn, 0;...
               0,   -kn];
        Kn2 = [kn+kn1, 0;...
               0,      kn+kn1];
        Cn1 = [-cn, 0;...
               0,   -cn];
        Cn2 = [cn+cn1, 0;...
               0,      cn+cn1];
        % assembly
        Kn = addElementIn(Kn, Kn1, [dof1+1, 1]);
        Kn = addElementIn(Kn, Kn2, [dof1+1, dof1+1]);
        Cn = addElementIn(Cn, Cn1, [dof1+1, 1]);
        Cn = addElementIn(Cn, Cn2, [dof1+1, dof1+1]);
    end


    % sub-function 2
    function [Kj, Cj] = mj(kj,kj1,cj,cj1,dof1,dof2,dof3)
        dofNum1 = dof1 + dof2 + dof3;
        % initial
        Kj = zeros(dofNum1);
        Cj = zeros(dofNum1);
        % construct
        Kj1 = [-kj, 0;...
               0,   -kj];
        Kj2 = [kj+kj1, 0;...
               0,      kj+kj1];
        Kj3 = [-kj1, 0;...
               0,   -kj1];  
        Cj1 = [-cj, 0;...
               0,   -cj];
        Cj2 = [cj+cj1, 0;...
               0,      cj+cj1];
        Cj3 = [-cj1, 0;...
               0,   -cj1];
        % assembly
        Kj = addElementIn(Kj, Kj1, [dof1+1, 1]);
        Kj = addElementIn(Kj, Kj2, [dof1+1, dof1+1]);
        Kj = addElementIn(Kj, Kj3, [dof1+1, dof1+dof2+1]);
        Cj = addElementIn(Cj, Cj1, [dof1+1, 1]);
        Cj = addElementIn(Cj, Cj2, [dof1+1, dof1+1]);
        Cj = addElementIn(Cj, Cj3, [dof1+1, dof1+dof2+1]);
    end
end  