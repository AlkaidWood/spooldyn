%% CUSTOMFORCE - Apply user-defined external forces to rotor system
% Implements custom force injection for dynamic rotor system simulations.
%
%% Syntax
%   F = customForce(qn, dqn, tn, omega, domega, ddomega, Parameter)
%
%% Description
% |CUSTOMFORCE| enables user-defined external force input during dynamic 
% simulations. Modify this function to implement:
% * Time-dependent forces
% * Position/velocity-dependent forces
% * Rotational-speed-dependent excitations
%
%% Input Arguments
% *qn*       - [n×1 double] System displacement vector at time tn
%              n = total degrees of freedom (DOF)
% *dqn*      - [n×1 double] Velocity vector (dq/dt) at time tn
% *tn*       - [double] Current simulation time (seconds)
% *omega*    - [m×1 double] Shaft rotational phases (radians)
% *domega*   - [m×1 double] Shaft angular velocities (rad/s)
% *ddomega*  - [m×1 double] Shaft angular accelerations (rad/s²)
% *Parameter* - [struct] System configuration. You can get Parameter after
%               using Parameter = establishModel(InitialParameter)
%
%% Output
% *F* - [n×1 double] Custom force vector to apply at current time step
%
%% Implementation Guide
% 1. Access system parameters through |Parameter| structure
% 2. Access state parameters through qn, dqn, tn, omega, domega, ddomega
% 3. Return force vector matching system DOF ordering
%
%% Example
% % Add sinusoidal force at DOF 10
% F = zeros(Parameter.Mesh.dofNum, 1);
% F(10) = 1000*sin(2*pi*50*tn); % 50Hz excitation
%
% % Apply constant radial force to bearing node
% bearingNode = Parameter.Bearing.positionNode(1);
% dofRange = Parameter.Mesh.dofInterval(bearingNode,:);
% F(dofRange(1):dofRange(2)) = [0; 1000; 0; 0]; % [Fx=0, Fy=1kN]
%
%% See Also
% establishModel, calculateResponse, dynamicEquation
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%
%%

function F = customForce(qn, dqn, tn, omega, domega, ddomega, Parameter)

dof_num = Parameter.Mesh.dofNum;

F = zeros(dof_num, 1);

end