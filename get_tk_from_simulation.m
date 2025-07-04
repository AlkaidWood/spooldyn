%% get_tk_from_simulation - Calculate rotation period timestamps from phase data
%
% This function computes the time points (tk) corresponding to each full revolution 
% of a rotating system based on rotational phase (omega) and time data. 
% It identifies the completion of each rotational cycle and optionally filters 
% low-speed regions at the start and end of the time series.
%
%% Syntax
%  tk = get_tk_from_simulation(omega, time)
%  tk = get_tk_from_simulation(omega, time, NameValues)
%
%% Description
% |get_tk_from_simulation| determines the precise moments when a rotating 
% system completes full revolutions (2π radians). The function:
% * Converts continuous phase data into discrete revolution events
% * Handles multiple signal channels simultaneously
% * Supports optional low-speed region filtering
% * Uses interpolation for accurate timestamp calculation
%
%% Input Arguments
% * |omega| - Rotational phase data [matrix or vector]:
%   * Values represent angular position in radians
%   * Matrix dimensions: n_signals × n_samples
%   * Vector for single signal: 1 × n_samples
% * |time| - Corresponding time values [matrix or vector]:
%   * Must match dimensions of |omega|
%   * Time units: seconds or consistent units
%
%% Name-Value Pair Arguments
% * |'is_delete_low_speed_points'| - Low-speed filter flag [logical]:
%   * |false|: Keep all revolution points (default)
%   * |true|: Remove start/end low-speed regions
% * |'low_speed_threshold'| - Rotation speed threshold [rad/s]:
%   * Points below threshold are considered low-speed
%   * Default: 2π rad/s (1 revolution/second)
%
%% Output Arguments
% * |tk| - Revolution timestamps [cell array]:
%   * Size: n_signals × 1
%   * Elements: Timestamps for each revolution (1 × rev_count vectors)
%   * Units match input |time|
%
%% Algorithm
% 1. Input Validation:
%   * Checks dimension consistency
%   * Handles signal orientation
% 2. Revolution Identification:
%   * Calculates revolution count from phase range
%   * Defines phase targets at 2π intervals
% 3. Timestamp Interpolation:
%   * Uses spline interpolation for accurate timing
%   * Formula: tk = interp1(ω(t), t, 2kπ + ω₀, 'spline')
% 4. Low-Speed Filtering (Optional):
%   * Detects consecutive low-speed points at start/end
%   * Removes revolutions where rotational period exceeds threshold
%
%% Physical Interpretation
% * |tk|: Times when rotating system completes integer revolutions
% * Phase continuity: ω(tk) = 2kπ + ω₀ (mod 2π)
% * Low-speed regions correspond to acceleration/deceleration phases
%
%% Implementation Notes
% * Orientation Handling:
%   * Automatically transposes row-dominant inputs
% * Interpolation:
%   * Uses spline interpolation for smooth phase transitions
%   * May produce non-physical results with sparse data
% * Low-Speed Filtering:
%   * Threshold based on rotational period: 2π/Δt > threshold
%   * Two-pass detection (start and end separately)
%
%% Example
% % Generate phase data for constant rotation
% time = 0:0.01:10;                 % Time vector (1001 points)
% omega = 2*pi*10*time;             % Phase (10 rev/s)
% 
% % Calculate revolution timestamps
% tk = get_tk_from_simulation(omega, time);
% % Returns: { [0.1, 0.2, 0.3, ..., 10.0] } (100 timestamps)
%
% % With low-speed filtering
% time = 0:0.01:10;                 % Time vector (1001 points)
% omega = 0.5 * (2*pi*10)*time.^2;  % acceleration (10 rev/s^2)
% tk_filtered = get_tk_from_simulation(omega, time, ...
%     'is_delete_low_speed_points', true, ...
%     'low_speed_threshold', 20*pi); % 10 rev/s threshold
% % Returns same as above since no low-speed regions
%
%% Data Requirements
% * Inputs must be finite and real-valued
% * Phase should be monotonically increasing
% * Time vector should be uniformly sampled for best accuracy
% * Dimensional consistency between omega and time
%
%% See Also
% interp1, diff, find, time2angular
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%
function tk = get_tk_from_simulation(omega, time, NameValue)

arguments
omega
time
NameValue.is_delete_low_speed_points = false
NameValue.low_speed_threshold = 2*pi
end

% check input
if all(size(omega) ~= size(time))
    error("The dimension of omega signal and time signal should be same.")
end % end if

% reshape the input data 
[row_num, column_num] = size(omega);
if row_num>column_num
    omega = omega';
    time = time';
end % end if

% initial output
tk = cell(size(omega,1),1);

% generate tk
for iSignal = 1:1:size(omega,1)

    % create angular domain
    rev_num = floor(omega(iSignal,end)/(2*pi)); % number of revolution
    thetaK = 0:2*pi:rev_num*2*pi; % end point of each period
    thetaK = thetaK + omega(iSignal,1); % add the initial info
    
    % interpolation of time corresponding to thetaW
    if rev_num>0
        tk_here = interp1(omega(iSignal,:),time(iSignal,:), thetaK,'spline');
    else
        tk_here = 0;
    end
    
    % delete low speed point (optional)
    if NameValue.is_delete_low_speed_points
        % Pre - allocate arrays to store the indices to be deleted at the start and end
        % Assume the maximum number of elements to be deleted is the length of tk
        indexDelStart = false(length(tk_here), 1); 
        indexDelEnd = false(length(tk_here), 1);
        
        % Check the low - speed region from the beginning of the sequence
        for i = 1:length(tk_here)-1
            if 2*pi/(tk_here(i + 1)-tk_here(i)) < NameValue.low_speed_threshold
                indexDelStart(i) = true;
                indexDelStart(i + 1) = true;
            else
                % Once a non - low - speed region is encountered, stop checking
                break;
            end
        end
        
        % Check the low - speed region from the end of the sequence
        for i = length(tk_here):-1:2
            if 2*pi/(tk_here(i)-tk_here(i - 1)) < NameValue.low_speed_threshold
                indexDelEnd(i - 1) = true;
                indexDelEnd(i) = true;
            else
                % Once a non - low - speed region is encountered, stop checking
                break;
            end
        end
        
        % Combine the deletion indices from the start and end
        indexDel = indexDelStart | indexDelEnd;
        
        % Delete the corresponding time points
        tk_here(indexDel) = [];
    end

    % reshape the output as same as input
    if row_num>column_num
        tk_here = tk_here';
    end % end if

    % save
    tk{iSignal} = tk_here;
end % end for

end