%% inputCustomFunction - Configure custom force functions for rotor system
%
% This function adds custom force definitions to a rotor system model, 
% enabling user-defined forces applied at specified positions along shafts.
%
%% Syntax
%  OutputParameter = inputCustomFunction(InputParameter)
%
%% Description
% |inputCustomFunction| configures custom force functions applied at 
% specific node positions on shafts. It enables:
% * Definition of force application points
% * Implementation of user-defined force functions
% * Dynamic force calculation based on system state
%
% * Inputs:
%   * |InputParameter| - Preconfigured rotor system parameters structure
%
% * Outputs:
%   * |OutputParameter| - Updated parameter structure with custom force configuration
%
%% Custom Force Parameters (Custom structure)
% * amount            - Number of custom force points (scalar)
% * inShaftNo         - Shaft index for each force point (column vector)
% * positionOnShaftDistance - Force positions from shaft ends [m] (column vector)
% * force             - Handle to custom force calculation function
%
%% Force Function Interface
% The custom force function must follow the signature:
%   [F] = @(qn, dqn, tn, omega, domega, ddomega, Parameter)
% * Input Arguments:
%   * |qn|      - System displacement vector at time tn
%   * |dqn|     - System velocity vector at time tn
%   * |tn|      - Current simulation time
%   * |omega|   - Rotational phases of shafts [rad]
%   * |domega|  - Rotational speeds of shafts [rad/s]
%   * |ddomega| - Rotational accelerations of shafts [rad/s²]
%   * |Parameter| - Full system parameter structure
% * Output:
%   * |F| - Custom force vector (n×1) added to global force vector
%
%% Model Integration
% * Creates additional nodes at specified force positions
% * Automatically enables |hasCustom| flag in ComponentSwitch
% * Forces are added to the global force vector during simulation
%
%% Example
%   % Initialize system parameters
%   sysParams = inputEssentialParameter();
%   % Add custom forces
%   sysParams = inputCustomFunction(sysParams);
%   % Implement force function in customForce.m
%
%% See Also
%  customForce, inputEssentialParameter
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


%%
function OutputParameter = inputCustomFunction(InputParameter)

% input 1
% to create nodes at the force position
% -------------------------------------------------------------------------
Custom.amount                  = 2;
Custom.inShaftNo               = [1; 1];
Custom.positionOnShaftDistance = [20; 497.2]*1e-3;
% -------------------------------------------------------------------------

checkInputData(Custom);

% input 2
% to create your force
% Please edit your function by using qn, dqn, tn, omega, domega, ddomega as
% input; and output force column vector (n*1), n is dofs number of entire
% system.
% *qn: displacement of system at time tn
% *dqn: The first derivative of qn
% *tn: the n-th time step
% *omega: rotational phase of shafts
% *domega: rotational speed of shafts
% *ddomega: rotational acceleration of shafts
% -------------------------------------------------------------------------
Custom.force = @(qn, dqn, tn, omega, domega, ddomega, Parameter) customForce(qn, dqn, tn, omega, domega, ddomega, Parameter);
% -------------------------------------------------------------------------

%%

% output
OutputParameter = InputParameter;
OutputParameter.Custom = Custom;
OutputParameter.ComponentSwitch.hasCustom = true;
end