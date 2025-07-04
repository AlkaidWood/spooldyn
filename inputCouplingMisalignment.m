%% inputCouplingMisalignment - Configure coupling misalignment parameters
%
% This function configures coupling misalignment parameters for rotor 
% systems modeling, enabling simulation of parallel and angular misalignment 
% effects in driveline couplings.
%
%% Syntax
%  OutputParameter = inputCouplingMisalignment(InputParameter)
%
%% Description
% |inputCouplingMisalignment| adds coupling misalignment configuration to 
% rotor system parameters to simulate:
% * Parallel offset misalignment
% * Angular misalignment
% * Combined misalignment effects
%
% * Inputs:
%   * |InputParameter| - Preconfigured rotor system parameters structure
%
% * Outputs:
%   * |OutputParameter| - Updated parameter structure with coupling misalignment
%
%% Coupling Misalignment Parameters (CouplingMisalignment structure)
% * amount               - Number of couplings (scalar)
% * inShaftNo            - Shaft index containing the coupling (column vector)
% * positionOnShaftDistance - Coupling position from shaft end [m] (column)
% * parallelOffset       - Parallel offset distance [m] (column vector)
% * angleOffset          - Angular misalignment [rad] (column vector)
% * distance             - Distance between coupling centers [m] (column)
% * mass                 - Coupling mass [kg] (column vector)
%
%% Model Integration
% * Automatically enables |hasCouplingMisalignment| in ComponentSwitch
% * Generates misalignment forces/moments during simulation
% * Supports multiple misaligned couplings in a single system
% * Position specified relative to shaft starting point
%
%% Example
%   % Initialize system parameters
%   sysParams = inputEssentialParameter();
%   % Add coupling misalignment at shaft position 0.733m
%   sysParams = inputCouplingMisalignment(sysParams);
%
%% See Also
% inputEssentialParameter
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


%%
function OutputParameter = inputCouplingMisalignment(InputParameter)

CouplingMisalignment.amount = 1;
CouplingMisalignment.inShaftNo = 1;
CouplingMisalignment.positionOnShaftDistance = 0.733; % m
CouplingMisalignment.parallelOffset = 10e-3; % m
CouplingMisalignment.angleOffset = 0; % rad
CouplingMisalignment.distance = 50e-3; % m
CouplingMisalignment.mass = 10; % kg


checkInputData(CouplingMisalignment);

%%

OutputParameter = InputParameter;
OutputParameter.CouplingMisalignment = CouplingMisalignment;
OutputParameter.ComponentSwitch.hasCouplingMisalignment = true;
end