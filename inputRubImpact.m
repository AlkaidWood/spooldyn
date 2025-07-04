%% inputRubImpact - Configure rotor-stator rub impact parameters
%
% This function configures parameters for rotor-stator rub impact modeling, 
% enabling simulation of contact phenomena between rotating and stationary 
% components in rotor systems.
%
%% Syntax
%  OutputParameter = inputRubImpact(InputParameter)
%
%% Description
% |inputRubImpact| adds rotor-stator rub impact configuration to rotor 
% system parameters to simulate:
% * Radial clearance between rotor and stator
% * Contact stiffness during rub events
% * Frictional effects during contact
%
% * Inputs:
%   * |InputParameter| - Preconfigured rotor system parameters structure
%
% * Outputs:
%   * |OutputParameter| - Updated parameter structure with rub impact configuration
%
%% Rub Impact Parameters (RubImpact structure)
% * amount                - Number of rub impact locations (scalar)
% * inShaftNo             - Shaft index containing rub point (column vector)
% * positionOnShaftDistance - Rub position from shaft end [m] (column vector)
% * interval              - Radial clearance between rotor and stator [m]
% * stiffness             - Contact stiffness during rub [N/m]
% * coefficientOfFriction - Friction coefficient during contact
%
%% Model Characteristics
% * Simulates intermittent or continuous rotor-stator contact
% * Models both normal contact forces and tangential friction forces
% * Position specified relative to shaft starting point
% * Automatically enables |hasRubImpact| in ComponentSwitch
%
%% Example
%   % Initialize system parameters
%   sysParams = inputEssentialParameter();
%   % Add rub impact at shaft position 0.733m
%   sysParams = inputRubImpact(sysParams);
%
%% See Also
%  inputEssentialParameter, checkInputData, solveGlobalSystem
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


%%
function OutputParameter = inputRubImpact(InputParameter)

RubImpact.amount                = 1;
RubImpact.inShaftNo             = 1;
RubImpact.positionOnShaftDistance = 0.733; % m
RubImpact.interval              = 6.55e-3; % the gap between rotator and stator (m)
RubImpact.stiffness             = 1e6;     % N*m
RubImpact.coefficientOfFriction = 0.2;

checkInputData(RubImpact);

%%

OutputParameter = InputParameter;
OutputParameter.RubImpact = RubImpact;
OutputParameter.ComponentSwitch.hasRubImpact = true;
end