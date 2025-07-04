%% inputLoosingBearing - Configure bearing clearance (loosing) parameters
%
% This function configures parameters for bearing clearance modeling, 
% enabling simulation of bearing loosing faults in rotor systems.
%
%% Syntax
%  OutputParameter = inputLoosingBearing(InputParameter)
%
%% Description
% |inputLoosingBearing| adds bearing clearance parameters to simulate
% "loosing" faults where bearings develop internal clearances due to wear
% or damage. Key features:
% * Simulates increased clearance in specific bearing elements
% * Models altered stiffness/damping characteristics
% * Supports multi-mass bearing models
%
% * Inputs:
%   * |InputParameter| - Preconfigured rotor system parameters structure
%
% * Outputs:
%   * |OutputParameter| - Updated parameter structure with bearing clearance
%
%% Loosing Bearing Parameters (LoosingBearing structure)
% * amount              - Number of bearings with clearance (scalar, currently supports 1)
% * inBearingNo         - Bearing index (row index in Parameter.Bearing after establishModel)
% * interval            - Clearance distance [m] (scalar)
% * loosingStiffness    - Loosened state stiffness [N/m] (scalar)
% * loosingDamping      - Loosened state damping [N·s/m] (scalar)
% * loosingPositionNo   - Position index in mass-spring chain (scalar)
%
%% Model Specifications
% * Applies to bearings modeled as:
%   shaft - k1c1 - m1 - k2c2 - m2 - ... - basement
% * Clearance affects specified spring-damper position in the chain
% * Requires bearings with mass > 0 (validated internally)
% * Currently supports single bearing clearance simulation
%
%% Example
%   % Initialize system parameters
%   sysParams = inputEssentialParameter();
%   % Configure bearing parameters
%   sysParams = inputBearingHertz(sysParams);
%   % Add bearing clearance at specified position
%   sysParams = inputLoosingBearing(sysParams);
%
%% See Also
%  inputBearingHertz, inputEssentialParameter, establishModel
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


%%
function OutputParameter = inputLoosingBearing(InputParameter)

LoosingBearing.amount = 1; % up to now, this program only support to simulate 1 loosing fault; amount = 1
% inbearingNo corresponding to the row index of the Parameter.Bearing.xxx after establishModel()
LoosingBearing.inBearingNo = [2]; 
LoosingBearing.interval = [1e-4]; % m
LoosingBearing.loosingStiffness = [1.5e7]; % N*m
LoosingBearing.loosingDamping = [100]; % N*s/m

% the model of bearing element:
% shaft--k1c1--m1--k2c2--m2--k3c3--m3--k4c4- ...-mn--k(n+1)c(n+1)--basement;
LoosingBearing.loosingPositionNo = [2]; % indicates the k,c no. which is loosing (e.g., loosing k1,c1 -> 1 or loosing k2,c2 -> 2)


checkInputData(LoosingBearing);

if InputParameter.Bearing.mass(LoosingBearing.inBearingNo) == 0
    error('the mass of Bearing which is loose must be lager than 0');
end

%%

OutputParameter = InputParameter;
OutputParameter.LoosingBearing = LoosingBearing;
OutputParameter.ComponentSwitch.hasLoosingBearing = true;

end