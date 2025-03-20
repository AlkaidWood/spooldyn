%% diskElement
% generate the mass, stiffness, gyroscopic matrix, gravity vector and eccentricity of a disk element
%% Syntax
% [Me, Ge, Ne, Fge, Ee] = diskElement(ADisk)
%% Description
% ADisk is a struct saving the physical parameters of a disk element with
% fields: dofOfEachNodes, radius, density, thickness, eccentricity
%
% Me, Ge, Ne are mass, gyroscopic and N matrix of a disk element. 
% (n*n, n is the number of all dofs on this shaft element)
%
% Fge is gravity vector of a disk element (n*1)
%
% Ee is eccentricity of disk
%% Symbols
% m: mass of disk
%
% r: radius of disk
%
% rho: density 
%
% dof: degree of freedom of this disk element
%
% Id: rotational inertial about diameter
% 
% Ip: polar rotational inertial
%
% eDisk: eccentricity of the disk


function [Me, Ge, Ne, Fge, Ee] = diskElement(ADisk)

% check the input
fieldName = {'dofOfEachNodes', 'outerRadius', 'innerRadius', 'density', 'thickness'};
hasFieldName = isfield(ADisk, fieldName);
if length(hasFieldName) ~= sum(hasFieldName)
    error('Incorrect field names for input struct')
end

%%

% calculate the constants
r1 = ADisk.innerRadius;
r2 = ADisk.outerRadius;
thickness = ADisk.thickness;
rho = ADisk.density;
eDisk = ADisk.eccentricity;
m = (r2^2 - r1^2) * pi* thickness * rho;
Id = 1/4 * m * (r1^2+r2^2);
Ip = 1/2 * m * (r1^2+r2^2);

%%

% mass matrix
MT = [ m, 0, 0, 0;...
       0, m, 0, 0;...
       0, 0, 0, 0;...
       0, 0, 0, 0 ];

MR = [ 0,  0,  0,  0;...
       0,  0,  0,  0;...
       0,  0, Id,  0;...
       0,  0,  0, Id ]; 
   
Me = MT + MR;

%%

% gyrosocpic matrix
Ge = [  0,   0,   0,   0;...
        0,   0,   0,   0;...
        0,   0,   0, -Ip;...
        0,   0,  Ip,   0 ]; 
  
%%

% Ne matrix
Ne = [  0,   0,   0,   0;...
        0,   0,   0,   0;...
        0,   0,   0,   0;...
        0,   0,  Ip,   0 ]; 

%%

% gravity
FgeTotal = m * 9.8; % N
Fge = [0; -FgeTotal; 0; 0];


%%

% eccentricity
Ee = m * eDisk;

end