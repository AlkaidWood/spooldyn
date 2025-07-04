%% generateMisalignmentForce - Generate coupling misalignment force function
%
% This function creates a MATLAB function file (misalignmentForce.m) that 
% calculates coupling misalignment forces in rotor systems based on 
% specified geometric offsets and rotational dynamics.
%
%% Syntax
%  generateMisalignmentForce(CouplingMisalignment, Mesh)
%
%% Description
% |generateMisalignmentForce| constructs a function to compute coupling 
% misalignment forces resulting from:
% * Parallel offsets (ΔY)
% * Angular misalignments (Δα)
% * Inter-shaft distances (ΔL)
% The generated function implements:
% * Equivalent misalignment calculation
% * Speed-dependent force formulation
% * Phase-dependent force components
%
%% Input Arguments
% * |CouplingMisalignment| - Misalignment configuration structure:
%   * |parallelOffset|     % Parallel offset ΔY [m]
%   * |distance|           % Inter-shaft distance ΔL [m]
%   * |angleOffset|        % Angular misalignment Δα [rad]
%   * |mass|               % Coupling mass m [kg]
%   * |positionOnShaftNode|% Mounting node indices [vector]
%   * |inShaftNo|          % Connected shaft indices [vector]
%   * |amount|             % Number of couplings [scalar]
%
% * |Mesh| - Discretization structure with:
%   * |dofInterval|        % DOF ranges per node [n×2 matrix]
%   * |dofNum|             % Total DOF count [scalar]
%
%% Physical Formulation
% 1. Equivalent Misalignment:
%   $$\Delta E = \frac{\Delta Y}{2} + \frac{\Delta L}{2} \tan\left(\frac{\Delta\alpha}{2}\right)$$
% 2. Force Calculation:
%   $f = -2m \Delta E \cdot \omega^2$
% 3. Directional Components:
%   * X-component: $f_x = f \cdot \sin(2\omega t)$
%   * Y-component: $f_y = f \cdot \cos(2\omega t)$
%
%% Generated Function (misalignmentForce.m)
% Function Signature:
%   fMisalignment = misalignmentForce(omega, domega)
% * Inputs:
%   - |omega|: Rotational phase vector [rad]
%   - |domega|: Rotational speed vector [rad/s]
% * Output:
%   - |fMisalignment|: Misalignment force vector [n×1]
%
%% Implementation Details
% 1. Precomputation:
%   * Calculates equivalent misalignment ΔE during generation
%   * Determines force coefficient: $-2m\Delta E$
% 2. Runtime Calculation:
%   * Computes force magnitude proportional to $\omega^2$
%   * Applies phase-dependent sin/cos modulation
%   * Distributes forces to appropriate DOFs
% 3. File Handling:
%   * Overwrites existing misalignmentForce.m
%   * Creates temporary .txt file during generation
%
%% Example
% % Configure misalignment parameters
% misalignCfg = struct('parallelOffset', 0.001, ...
%                     'distance', 0.05, ...
%                     'angleOffset', 0.01, ...
%                     'mass', 1.5, ...
%                     'positionOnShaftNode', [3, 5], ...
%                     'inShaftNo', [1, 2], ...
%                     'amount', 2);
% % Generate force function (After Modeling)
% generateMisalignmentForce(misalignCfg, meshData);
% % Usage in simulation:
% f_misalign = misalignmentForce(phase_vector, speed_vector);
%
%% Dependencies
% Requires proper DOF mapping from |meshModel|
%
%% See Also
% generateDynamicEquation, generateHertzianForce, meshModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function generateMisalignmentForce(CouplingMisalignment, Mesh)

% check the exist of rubImpactForce and create .txt
if isfile('misalignmentForce.m')
    delete misalignmentForce.m
end

cmf = fopen('misalignmentForce.txt','w');

%%

% write comments, firstly

comments = [...
"%% misalignmentForce";...
"% saving the equation of coupling misalignment force in this function";...
"%% Syntax";...
"% fMisalignment = misalignmentForce(qn, omega, domega)";...
"%% Description";...
"% omega, domega: are vector denoting the phase and speed of the shaft ";...
"% ";...
"% fMisalignment: is misalignment force (vector)";...
" ";...
" "...
];

%%

% function start
functionStart = [...
"function fMisalignment = misalignmentForce(omega, domega)";...
" "...
];

fprintf(cmf,'%s\n',comments);
fprintf(cmf,'%s\n',functionStart);

%%

% calculate the constants in the misalignment force

deltaY     = CouplingMisalignment.parallelOffset;
deltaL     = CouplingMisalignment.distance;
deltaAlpha = CouplingMisalignment.angleOffset;
m          = CouplingMisalignment.mass;
deltaE     = deltaY ./ 2 + deltaL ./ 2 .* tan(deltaAlpha ./ 2); 
coeff      = -2 * m .* deltaE;
misDof = Mesh.dofInterval(CouplingMisalignment.positionOnShaftNode,:)';
misDofX = misDof(1,:);
misDofY = misDof(1,:) +1;

%%
functionBody = {...
 ' ';...
['fMisalignment = zeros(', num2str(Mesh.dofNum), ',1);'];...
['for iMis = 1:1:', num2str(CouplingMisalignment.amount)];...
['    inShaftNo = [', num2str(CouplingMisalignment.inShaftNo'), '];'];...
['    constants = [', num2str(coeff), '] .* domega(inShaftNo).^2 ;'];...
['    fMisalignment([', num2str(misDofX), ']) = constants .* sin(2*omega(inShaftNo));'];...
['    fMisalignment([', num2str(misDofY), ']) = constants .* cos(2*omega(inShaftNo));'];...
 'end';...
 ' ';...
};

functionBody = cell2string(functionBody);

fprintf(cmf,'%s\n', functionBody);

%%

% function end
functionEnd = [...
"end";...
" "...
];
fprintf(cmf,'%s\n',functionEnd);

%%

% close .txt and transfer .txt -> .m
fclose(cmf);
system('rename misalignmentForce.txt misalignmentForce.m');
end