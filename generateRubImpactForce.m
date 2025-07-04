%% generateRubImpactForce - Generate rub-impact force calculation function
%
% This function creates a MATLAB function file (rubImpactForce.m) that 
% computes rub-impact forces between rotor and stator components in 
% rotor dynamics simulations.
%
%% Syntax
%  generateRubImpactForce(RubImpact, Mesh)
%
%% Description
% |generateRubImpactForce| constructs a function to calculate nonlinear 
% rub-impact forces based on rotor-stator clearance violations. The 
% generated function:
% * Implements radial clearance checking
% * Computes tangential friction forces
% * Applies nonlinear stiffness effects
% * Distributes forces to appropriate DOFs
%
%% Input Arguments
% * |RubImpact| - Rub-impact configuration structure:
%   * |positionOnShaftNode| % Shaft node indices for rub points [n×1]
%   * |stiffness|           % Contact stiffness coefficients [N/m] [n×1]
%   * |coefficientOfFriction| % Friction coefficients [n×1]
%   * |interval|            % Radial clearances [m] [n×1]
%   * |amount|              % Number of rub points [scalar]
%
% * |Mesh| - Discretization structure with:
%   * |dofInterval|         % DOF ranges per node [n×2 matrix]
%   * |dofNum|              % Total DOF count [scalar]
%
%% Generated Function (rubImpactForce.m)
% Function Signature:
%   fRub = rubImpactForce(qn)
% * Input:
%   - |qn|: Displacement vector [n×1]
% * Output:
%   - |fRub|: Rub-impact force vector [n×1]
%
%% Force Calculation Algorithm
% For each rub point i:
% 1. Radial Displacement Calculation:
%    $e = \sqrt{x_i^2 + y_i^2}$
% 2. Clearance Check:
%    if $e \ge \delta_i$ (radial clearance)
% 3. Force Components:
%    $f_x = k_i \left(1 - \frac{\delta_i}{e}\right) (x_i - \mu_i y_i)$
%    $f_y = k_i \left(1 - \frac{\delta_i}{e}\right) (\mu_i x_i + y_i)$
% 4. Force Application:
%    * Applies to X and Y DOFs of rub point
%
%% Physical Interpretation
% * |stiffness| (k): Contact stiffness during rub events
% * |coefficientOfFriction| (μ): Friction coefficient
% * |interval| (δ): Radial clearance between rotor and stator
%
%% Implementation Details
% 1. Parameter Mapping:
%   * Automatically maps rub points to DOFs
%   * Configures force calculation parameters
% 2. File Generation:
%   * Overwrites existing rubImpactForce.m
%   * Creates temporary .txt file during generation
% 3. Force Calculation:
%   * Uses radial displacement magnitude
%   * Implements conditional force application
%   * Handles multiple rub points
%
%% Example
% % Configure rub-impact parameters
% rubCfg = struct('positionOnShaftNode', [3, 5], ...
%                 'stiffness', [1e6, 1.5e6], ...
%                 'coefficientOfFriction', [0.2, 0.25], ...
%                 'interval', [0.001, 0.0008], ...
%                 'amount', 2);
% % Generate force function
% generateRubImpactForce(rubCfg, meshData);
% % Usage in simulation:
% rubForce = rubImpactForce(q_current);
%
%% Application Notes
% * The function only activates when radial displacement exceeds clearance
% * Combines normal contact force with tangential friction
% * Nonlinear stiffness increases as clearance decreases
%
%% See Also
% generateDynamicEquation, generateHertzianForce, bearingElement
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function generateRubImpactForce(RubImpact, Mesh)

% check the exist of rubImpactForce and create .txt
if isfile('rubImpactForce.m')
    delete rubImpactForce.m
end

rif = fopen('rubImpactForce.txt','w');

%%

% write comments, firstly

comments = [...
"%% rubImpactForce";...
"% saving the equation of rub-impact force in this function";...
"%% Syntax";...
"% fRub = rubImpactForce(qn)";...
"%% Description";...
"% qn: is the displacement at the n-th time";...
"% ";...
"% fRub: is rub-impact force (vector)";...
" ";...
" "...
];

%%

% function start
functionStart = [...
"function fRub = rubImpactForce(qn)";...
" "...
];

fprintf(rif,'%s\n',comments);
fprintf(rif,'%s\n',functionStart);

%%

% calculate the number of dof for rub-impact force
rubDof = Mesh.dofInterval(RubImpact.positionOnShaftNode,:)';
rubDof = rubDof(1,:);


functionBody = {...
['kRub   = [', num2str(RubImpact.stiffness), '];'];...
['mu     = [', num2str(RubImpact.coefficientOfFriction), '];'];...
['delta  = [', num2str(RubImpact.interval), '];'];...
['rubDof = [', num2str(rubDof), '];'];...
 ' ';...
 ' ';...
['fRub = zeros(', num2str(Mesh.dofNum), ',1);'];...
['for iRub = 1:1:', num2str(RubImpact.amount)];...
 '    e = sqrt(qn(rubDof(iRub))^2 + qn(rubDof(iRub)+1)^2);';...
 '    if e >= delta(iRub)';...
 '       fRub(rubDof(iRub))   = kRub(iRub)*(1-delta/e) * (qn(rubDof(iRub)) - mu*qn(rubDof(iRub)+1));';...
 '       fRub(rubDof(iRub)+1) = kRub(iRub)*(1-delta/e) * (mu*qn(rubDof(iRub)) + qn(rubDof(iRub)+1));';...
 '    end';...
 'end';...
 ' ';...
};

functionBody = cell2string(functionBody);

fprintf(rif,'%s\n', functionBody);

%%

% function end
functionEnd = [...
"end";...
" "...
];
fprintf(rif,'%s\n',functionEnd);

%%

% close .txt and transfer .txt -> .m
fclose(rif);
system('rename rubImpactForce.txt rubImpactForce.m');
end