%% shaftElement
% generate the mass, stiffness, gyroscopic matrix, gravity vector of a shaft element
%% Syntax
% [Me, Ke, Ge, Ne, Fge] = shaftElement(AShaft)
%% Description
% AShaft is a struct saving the physical parameters of a shaft element with
% fields: dofOfEachNodes, outerRadius, innerRadius, density, elasticModulus, 
% poissonRatio, length.
%
% Me, Ke, Ge, Ne are mass, stiffness, gyroscopic matrix of a shaft element. 
% (n*n, n is the number of all dofs on this shaft element)
%
% Fge is gravity vector of a shaft element (n*1)
%% Symbols
% A: sectional area
%
% As: effective shear area 
% 
% $$A_s = \frac{A}{\frac{7+6\mu}{6(1+\mu)} \left[ 1+ \frac{20+12\mu}{7+6\mu} \left( \frac{Dd}{D^2+d^2}  \right)^2 \right]}$$
% 
% l: length of the shaft element
% 
% mu: poisson's ratio
% 
% E: elastic modulus
% 
% G: shear modulus 
%
% $$G = \frac{E}{2(1+\mu)}$$
%
% d: inner radius
%
% D: outter radius
%
% phis: a constant 
% 
% $$\varphi_s = \frac{12EI}{G A_s l^2} = \frac{24 I (1+\mu)}{A_s l^2}$$
% 
% I: second moment of aera
%
% m: mass of the element


function [Me, Ke, Ge, Ne, Fge] = shaftElement(AShaft)

% check the input
fieldName = {'dofOfEachNodes', 'outerRadius', 'innerRadius', 'density',... 
             'elasticModulus', 'poissonRatio', 'length'};
hasFieldName = isfield(AShaft, fieldName);
if length(hasFieldName) ~= sum(hasFieldName)
    error('Incorrect field names for input struct')
end

%%

% calculate the constants
r       = AShaft.innerRadius;
R       = AShaft.outerRadius;
l       = AShaft.length;
E       = AShaft.elasticModulus;
mu      = AShaft.poissonRatio;
rho     = AShaft.density;
A       = pi*R^2 - pi*r^2;
rhoL    = rho * A;
As1     = (7+6*mu) / ( 6*(1+mu) ); 
As2     = (20+12*mu) /(7+6*mu); 
As3     = ( (R*r)/(R^2+r^2) )^2;
As      = A / ( As1*(1+As2*As3) );
I       = pi/4 *( R^4 - r^4 );
phis    = 24*I*(1+mu) / (As*l^2);

%%

% mass matrix (translation)
coefficient = rhoL * l / (1+phis)^2;
MT1 = 13/35 + (7/10)*phis + (1/3)*phis^2;
MT2 = l^2 * ( 1/105 +(1/60)*phis + (1/120)*phis^2 );
MT3 = 9/70 + (3/10)*phis + (1/6)*phis^2;
MT4 = l * ( 11/210 + (11/120)*phis + (1/24)*phis^2 );
MT5 = l * ( 13/420 + (3/40)*phis + (1/24)*phis^2 );
MT6 = (-1)*l^2 *  ( 1/140 + (1/60)*phis + (1/120)*phis^2 );

MT = [ MT1,    0,    0,    0,    0,    0,    0,    0;...
         0,  MT1,    0,    0,    0,    0,    0,    0;...
         0, -MT4,  MT2,    0,    0,    0,    0,    0;...
       MT4,    0,    0,  MT2,    0,    0,    0,    0;...
       MT3,    0,    0,  MT5,  MT1,    0,    0,    0;...
         0,  MT3, -MT5,    0,    0,  MT1,    0,    0;...
         0,  MT5,  MT6,    0,    0,  MT4,  MT2,    0;...
      -MT5,    0,    0,  MT6, -MT4,    0,    0,  MT2];

MT = coefficient * triangular2symmetric(MT);


% mass matrix (rotation)
coefficient = rhoL * I / ( l * (1+phis)^2 * A );
MR1 = 6/5;
MR2 = l^2 * ( 2/15 + (1/6)*phis + (1/3)*phis^2);
MR3 = l^2 * ( -1/30 - (1/6)*phis + (1/6)*phis^2 );
MR4 = l * (1/10 - (1/2)*phis);

MR = [ MR1,    0,    0,    0,    0,    0,    0,    0;...
         0,  MR1,    0,    0,    0,    0,    0,    0;...
         0, -MR4,  MR2,    0,    0,    0,    0,    0;...
       MR4,    0,    0,  MR2,    0,    0,    0,    0;...
      -MR1,    0,    0, -MR4,  MR1,    0,    0,    0;...
         0, -MR1,  MR4,    0,    0,  MR1,    0,    0;...
         0, -MR4,  MR3,    0,    0,  MR4,  MR2,    0;...
       MR4,    0,    0,  MR3, -MR4,    0,    0,  MR2];
   
MR = coefficient * triangular2symmetric(MR);


% mass matrix
Me = MT + MR;

%%

% Ne
coefficient = rhoL * I / ( 15 * l * (1+phis)^2 * A );
N1 = 36;
N2 = 3*l - 15*l*phis;
N3 = l^2 + 5*l^2*phis - 5*l^2*phis^2;
N4 = 4*l^2 + 5*l^2*phis + 10*l^2*phis^2;

Ne = [  0, -N1,  N2,   0,   0,  N1,  N2,   0;...
        0,   0,   0,   0,   0,   0,   0,   0;...
        0,   0,   0,   0,   0,   0,   0,   0;...
        0, -N2,  N4,   0,   0,  N2, -N3,   0;...
        0   N1, -N2,   0,   0, -N1, -N2,   0;...
        0,   0,   0,   0,   0,   0,   0,   0;...
        0,   0,   0,   0,   0,   0,   0,   0;...
        0, -N2, -N3,   0,   0,  N2,  N4,   0 ];
    
Ne = coefficient * Ne;

%%

% gyroscopic matrix
Ge = Ne - Ne';

%%

% stiffness matrix

coefficient = E*I / ( l^3*(1+phis) );
K1 = 12;
K2 = l^2 * ( 4 + phis );
K3 = l^2 * ( 2 - phis );
K4 = 6*l;

Ke = [ K1,   0,   0,   0,   0,   0,   0,   0;...
        0,  K1,   0,   0,   0,   0,   0,   0;...
        0, -K4,  K2,   0,   0,   0,   0,   0;...
       K4,   0,   0,  K2,   0,   0,   0,   0;...
      -K1,   0,   0, -K4,  K1,   0,   0,   0;...
        0, -K1,  K4,   0,   0,  K1,   0,   0;...
        0, -K4,  K3,   0,   0,  K4,  K2,   0;...
       K4,   0,   0,  K3, -K4,   0,   0,  K2 ];

Ke = coefficient * triangular2symmetric(Ke);

%%

% gravity
m = A * l *rho; % Kg, mass of the element
FgeTotal = m * 9.8; % N
Fge = [0; -FgeTotal/2; 0; 0; 0; -FgeTotal/2; 0; 0];

end % end function