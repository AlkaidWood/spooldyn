%% get_tk_from_simulation2 - Calculate rotation period timestamps from system parameters
%
% This function computes the time points (tk) corresponding to each full revolution 
% of multi-shaft rotating systems based on operational status parameters and time data. 
% It generates rotational phase profiles and identifies revolution timestamps.
%
%% Syntax
%  tk = get_tk_from_simulation2(Status, time, shaftNum)
%  tk = get_tk_from_simulation2(Status, time, shaftNum, NameValues)
%
%% Description
% |get_tk_from_simulation2| calculates revolution completion times for:
% * Constant speed operation
% * Accelerating/decelerating systems
% * Custom speed profiles
% * Multi-shaft systems with speed ratios
% It:
% * Validates system status parameters
% * Generates rotational phase profiles
% * Computes revolution timestamps
% * Filters low-speed regions
%
%% Input Arguments
% * |Status| - System operational status structure with fields:
%   * |ratio|: [n×1] Shaft speed ratios (n = shaftNum-1)
%   * |vmax|: Maximum rotational speed [rad/s] [scalar]
%   * |acceleration|: Rotational acceleration [rad/s²] [scalar]
%   * |duration|: Duration of constant speed phase [s] [scalar]
%   * |isDeceleration|: Deceleration flag [logical] [scalar]
%   * |vmin|: Minimum rotational speed [rad/s] [scalar]
%   * |isUseCustomize|: Custom profile flag [logical] [scalar]
%   * |customize|: Custom speed function handle (if applicable)
%
% * |time| - Time vector [1×m]:
%   * Must be monotonically increasing
%   * Covers the full operational range
%
% * |shaftNum| - Number of shafts [integer]:
%   * Default: 1
%
%% Name-Value Pair Arguments
% * |'is_delete_low_speed_points'| - Low-speed filter flag [logical]:
%   * |false|: Keep all points (default)
%   * |true|: Remove start/end low-speed regions
% * |'low_speed_threshold'| - Rotation speed threshold [rad/s]:
%   * Default: 2π rad/s (1 rev/s)
%
%% Output Arguments
% * |tk| - Revolution timestamps [cell array]:
%   * Size: shaftNum × 1
%   * Elements: Timestamps for each revolution (1 × rev_count vectors)
%
%% Algorithm
% 1. Parameter Validation:
%   * Checks required status fields
%   * Verifies time vector properties
% 2. Phase Calculation:
%   Standard operation:
%     ωₛ(t) = rotationalStatus(t, vmax×ratioₛ, duration, accel×ratioₛ, 
%                             decel_flag, vmin×ratioₛ)
%   Custom operation:
%     [~, ~, ωₛ(t)] = Status.customize(t)
% 3. Timestamp Calculation:
%   tkₛ = get_tk_from_simulation(ωₛ, time, filtering_params)
%
%% Physical Interpretation
% * |tk|: Revolution completion times for each shaft
% * Speed ratios: ω_shaftₖ = ω_ref × ratioₖ
% * Phase accumulation: ω(t) = ∫α(t)dt
%
%% Implementation Notes
% * Validation:
%   * Enforces 10 required status fields
%   * Checks time vector monotonicity
% * Phase Generation:
%   * Uses |rotationalStatus| for standard profiles
%   * Supports custom functions via handle
% * Ratio Handling:
%   * First shaft always has ratio = 1
%   * Additional ratios applied to derived shafts
%
%% Example
% % Configure status parameters for dual-shaft system
% Status = struct('vmax', 20*pi, 'acceleration', pi, 'duration', 5, ...
%                'isDeceleration', true, 'vmin', 5*pi, 'ratio', 1.5, ...
%                'isUseCustomize', false, 'customize', []);
% time = 0:0.01:20; % Time vector
% 
% % Calculate revolution timestamps
% tk = get_tk_from_simulation2(Status, time, 2);
%
% % With custom profile and filtering
% Status.isUseCustomize = true;
% Status.customize = @(t) calculateStatus(t);
% tk_filt = get_tk_from_simulation2(Status, time, 2, ...
%     'is_delete_low_speed_points', true, ...
%     'low_speed_threshold', 5*pi); % 2.5 rev/s threshold
%
%% Dependencies
% * |rotationalStatus|: Standard speed profile generation
% * |get_tk_from_simulation|: Core timestamp calculation
%
%% See Also
% get_tk_from_simulation, rotationalStatus, diff
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%
function tk = get_tk_from_simulation2(Status, time, shaftNum, NameValue)

arguments
Status
time
shaftNum = 1
NameValue.is_delete_low_speed_points = false
NameValue.low_speed_threshold = 2*pi
end



%% check input

% check input Status
referenceFields = {'ratio', 'vmax', 'acceleration', 'duration', 'isDeceleration', 'vmin', 'isUseCustomize', 'customize'};
fields_name = fieldnames(Status);
missingFields = cell(1, length(referenceFields));
missingCount = 0;
for i = 1:length(referenceFields)
    field = referenceFields{i};
    if ~ismember(field, fields_name)
        missingCount = missingCount + 1;
        missingFields{missingCount} = field;
    end
end
missingFields = missingFields(1:missingCount);
% report error
if ~isempty(missingFields)
    error('Input Status struct miss fields: %s\n', strjoin(missingFields, ', '));
end

% check input time
if ~isvector(time)
    error('The input time series must be a vector.');
end

if any(diff(time) <= 0)
    error('The input time series must be monotonically increasing.');
end


%% calculate rotational angular in time domain

% load some constants
vmax            = Status.vmax;
duration        = Status.duration;
acceleration    = Status.acceleration;
isDeceleration  = Status.isDeceleration;
vmin            = Status.vmin;
ratio           = Status.ratio;
ratio           = [1; ratio]; % the first shaft is basic

% initial 
omega = zeros(shaftNum,length(time));

if ~Status.isUseCustomize % using default status
    % calculate the status matrix
    for iShaft = 1:1:shaftNum
        iVmax           = vmax * ratio(iShaft);
        iVmin           = vmin * ratio(iShaft);
        iAcceleration   = acceleration*ratio(iShaft);
        status_here = rotationalStatus(time, iVmax, duration, iAcceleration,...
                                          isDeceleration, iVmin);
        omega(iShaft,:) = status_here(1,:);
    end % end for iShaft
else 
    % using customized status
    % calculate status
    for iTime = 1:1:length(time)
        [~, ~, omega_here] = Status.customize(time(iTime));
        for iShaft = 1:1:shaftNum
            omega(iShaft, iTime) = omega_here(iShaft);
        end % end for iShaft
    end % end for iTime
    
end % end if


%% get tk (output)

time_repeat = repmat(time, shaftNum, 1);
tk = get_tk_from_simulation(omega, time_repeat, ...
    is_delete_low_speed_points = NameValue.is_delete_low_speed_points, ...
    low_speed_threshold = NameValue.low_speed_threshold);

end