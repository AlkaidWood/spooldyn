%% shaftElement - Generate finite element matrices for Timoshenko beam shaft elements
%
% This function computes mass, stiffness, gyroscopic, and transient matrices 
% along with gravity forces for Timoshenko beam elements used in rotor dynamics 
% modeling of hollow circular shafts.
%
%% Syntax
%   [Me, Ke, Ge, Ne, Fge] = shaftElement(AShaft)
%
%% Description
% |shaftElement| calculates finite element matrices for shaft segments using 
% Timoshenko beam theory, accounting for:
% * Shear deformation effects
% * Rotary inertia
% * Gyroscopic moments
% * transient vibration
% * Gravity loading
%
%% Input Arguments
% * |AShaft| - Shaft properties structure with fields:
%   * |dofOfEachNodes|: DOF per node [scalar]
%   * |outerRadius|: Outer radius [m]
%   * |innerRadius|: Inner radius [m]
%   * |density|: Material density [kg/m³]
%   * |elasticModulus|: Elastic modulus [Pa]
%   * |poissonRatio|: Poisson's ratio
%   * |length|: Element length [m]
%
%% Output Arguments
% * |Me| - Mass matrix [8×8]
% * |Ke| - Stiffness matrix [8×8]
% * |Ge| - Gyroscopic matrix [8×8]
% * |Ne| - Transient matrix [8×8]
% * |Fge| - Gravity force vector [8×1]
%
%% Physical Parameters
% * |A|: Cross-sectional area [m²]
%   $A = \pi(R^2 - r^2)$
% * |As|: Effective shear area [m²]
%   $A_s = \frac{A}{\frac{7+6\mu}{6(1+\mu)} \left[ 1+ \frac{20+12\mu}{7+6\mu} \left( \frac{Rr}{R^2+r^2}  \right)^2 \right]}$
% * |I|: Second moment of area [m⁴]
%   $I = \frac{\pi}{4}(R^4 - r^4)$
% * |G|: Shear modulus [Pa]
%   $G = \frac{E}{2(1+\mu)}$
% * |φs|: Shear deformation coefficient
%   $\varphi_s = \frac{24I(1+\mu)}{A_s l^2}$
% * |ρL|: Mass per unit length [kg/m]
%   $\rho_L = \rho A$
%
%% Matrix Formulation
% 1. Mass Matrix (Me):
%    * Translational component (MT):
%        $M_T = \frac{\rho_L l}{(1+\phi_s)^2} \times$
%        $\begin{bmatrix}
%          MT1 & 0 & 0 & 0 & MT3 & 0 & 0 & -MT5 \\
%          0 & MT1 & -MT4 & 0 & 0 & MT3 & MT5 & 0 \\
%          0 & -MT4 & MT2 & 0 & 0 & -MT5 & MT6 & 0 \\
%          MT4 & 0 & 0 & MT2 & MT5 & 0 & 0 & MT6 \\
%          MT3 & 0 & 0 & MT5 & MT1 & 0 & 0 & -MT4 \\
%          0 & MT3 & -MT5 & 0 & 0 & MT1 & MT4 & 0 \\
%          0 & MT5 & MT6 & 0 & 0 & MT4 & MT2 & 0 \\
%         -MT5 & 0 & 0 & MT6 & -MT4 & 0 & 0 & MT2
%        \end{bmatrix}$
%    * Rotational component (MR):
%        $M_R = \frac{\rho_L I}{l(1+\phi_s)^2 A} \times$
%        $\begin{bmatrix}
%          MR1 & 0 & 0 & MR4 & -MR1 & 0 & 0 & MR4 \\
%          0 & MR1 & -MR4 & 0 & 0 & -MR1 & MR4 & 0 \\
%          0 & -MR4 & MR2 & 0 & 0 & MR4 & MR3 & 0 \\
%          MR4 & 0 & 0 & MR2 & -MR4 & 0 & 0 & MR3 \\
%         -MR1 & 0 & 0 & -MR4 & MR1 & 0 & 0 & -MR4 \\
%          0 & -MR1 & MR4 & 0 & 0 & MR1 & -MR4 & 0 \\
%          0 & MR4 & MR3 & 0 & 0 & -MR4 & MR2 & 0 \\
%          MR4 & 0 & 0 & MR3 & -MR4 & 0 & 0 & MR2
%        \end{bmatrix}$
%    * Total: $M_e = M_T + M_R$
%
% 2. Transient Matrix (Ne):
%    $N_e = \frac{\rho_L I}{15l(1+\phi_s)^2 A} \times$
%    $\begin{bmatrix}
%      0 & -N1 & N2 & 0 & 0 & N1 & N2 & 0 \\
%      0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
%      0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
%      0 & -N2 & N4 & 0 & 0 & N2 & -N3 & 0 \\
%      0 & N1 & -N2 & 0 & 0 & -N1 & -N2 & 0 \\
%      0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
%      0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
%      0 & -N2 & -N3 & 0 & 0 & N2 & N4 & 0
%    \end{bmatrix}$
%
% 3. Gyroscopic Matrix (Ge):
%    $G_e = N_e - N_e^T$
%
% 4. Stiffness Matrix (Ke):
%    $K_e = \frac{EI}{l^3(1+\phi_s)} \times$
%    $\begin{bmatrix}
%      K1 & 0 & 0 & K4 & -K1 & 0 & 0 & K4 \\
%      0 & K1 & -K4 & 0 & 0 & -K1 & K4 & 0 \\
%      0 & -K4 & K2 & 0 & 0 & K4 & K3 & 0 \\
%      K4 & 0 & 0 & K2 & -K4 & 0 & 0 & K3 \\
%     -K1 & 0 & 0 & -K4 & K1 & 0 & 0 & -K4 \\
%      0 & -K1 & K4 & 0 & 0 & K1 & -K4 & 0 \\
%      0 & K4 & K3 & 0 & 0 & -K4 & K2 & 0 \\
%      K4 & 0 & 0 & K3 & -K4 & 0 & 0 & K2
%    \end{bmatrix}$
%
% 5. Gravity Vector (Fge):
%    $F_{ge} = \begin{bmatrix} 0 \\ -mg/2 \\ 0 \\ 0 \\ 0 \\ -mg/2 \\ 0 \\ 0 \end{bmatrix}$
%    where $m = \rho A l$
%
%% Implementation Notes
% 1. Element DOF Ordering:
%    [x1, y1, θx1, θy1, x2, y2, θx2, θy2]^T
% 2. Matrix Symmetry:
%    Uses |triangular2symmetric| for symmetric storage
% 3. Shear Deformation:
%    Incorporated via φs coefficient
% 4. Hollow Shaft Handling:
%    Supports annular cross-sections
%
%% Example
% % Define shaft properties
% shaftProps = struct(...
%     'dofOfEachNodes', 4, ...
%     'outerRadius', 0.05, ...
%     'innerRadius', 0.03, ...
%     'density', 7850, ...
%     'elasticModulus', 210e9, ...
%     'poissonRatio', 0.3, ...
%     'length', 0.5);
% 
% % Generate element matrices
% [Me, Ke, Ge, Ne, Fge] = shaftElement(shaftProps);
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


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