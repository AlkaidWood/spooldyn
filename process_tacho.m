%% process_tacho - Process raw tacho signal to extract pulse times and rotational speed
%
% This function processes a raw tacho signal to identify pulse start times and 
% compute rotational speed. It supports multiple processing options including 
% noise filtering, low-speed point removal, signal smoothing, and visualization.
%
%% Syntax
%   tk = process_tacho(tacho, sampling_frequency)
%   tk = process_tacho(tacho, sampling_frequency, time)
%   tk = process_tacho(_, NameValue)
%   [tk, omega_rpm] = process_tacho(_)
%   [tk, omega_rpm, omega_rad] = process_tacho(_)
%
%% Description
% |process_tacho| performs comprehensive processing of tachometer signals:
% * Identifies pulse start times using adaptive thresholding
% * Computes rotational speed (RPM and rad/s)
% * Implements advanced signal conditioning options
% * Supports multiple output formats
%
%% Inputs
% * |tacho| - Raw tacho signal [vector]
% * |sampling_frequency| - Sampling rate [Hz]
%
%% Optional Inputs
% * |time| - Time vector corresponding to tacho signal [vector]:
%   * Must match length of tacho
%   * Default: auto-generated time series
% * |Name-Value Pairs| - Processing options (case-insensitive):
%   * |'is_delete_low_speed_points'| - Delete low-speed points [logical]:
%     * |false| (default) | true |
%   * |'low_speed_threshold'| - Speed threshold for deletion [rad/s]:
%     * Default: 2*π rad/s (1 revolution per second)
%   * |'is_smooth_speed'| - Apply Savitzky-Golay smoothing to speed [logical]:
%     * |false| (default) | true |
%   * |'is_smooth_tacho'| - Recalculate pulses from smoothed speed [logical]:
%     * |false| (default) | true |
%   * |'is_plot_speed'| - Generate speed visualization [logical]:
%     * |false| (default) | true |
%
%% Outputs
% * |tk| - Pulse start times [vector]:
%   * Time instants of pulse initiation
%   * Units: seconds
% * |omega_rpm| - Rotational speed [vector]:
%   * Units: revolutions per minute (RPM)
%   * Same length as tk
% * |omega_rad| - Rotational speed [vector]:
%   * Units: radians per second (rad/s)
%   * Same length as tk
%
%% Algorithm Details
% 1. Pulse Detection:
%    * Adaptive threshold: (max(tacho) + min(tacho)) / 2
%    * Linear interpolation for precise timing
% 2. Speed Calculation:
%    * ω_rad = 2π / Δt_pulses
%    * ω_rpm = ω_rad × 60/(2π)
% 3. Signal Processing:
%    * Low-speed removal: Removes consecutive endpoints < threshold
%    * Smoothing: Dual-pass Savitzky-Golay filter (window=20)
%    * Signal regeneration: Cumulative integration from smoothed speed
%
%% Examples
% % 1. Basic pulse detection with simulated signal
%   Fs = 20000;                          % Sampling frequency (Hz)
%   t = 0:1/Fs:5;                       % Time vector for 10 seconds
%   f_rot = 10;                           % Rotation frequency 
%   tacho = 100*sin(2*pi*f_rot*t);
%   tk = process_tacho(tacho, Fs, t, 'is_plot_speed', true);    % Get pulse times
%
% % 2. Full processing with visualization
%   [tk, rpm] = process_tacho(tacho, Fs, t, ...
%       'is_delete_low_speed_points', true, ...
%       'low_speed_threshold', 3*pi, ...
%       'is_smooth_speed', true, ...
%       'is_plot_speed', true);
%
% % 3. Signal regeneration from smoothed speed
%   [tk, rpm, rads] = process_tacho(tacho, Fs, t, ...
%       'is_smooth_tacho', true, 'is_plot_speed', true);
%
%% Application Notes
% 1. Low-Speed Handling:
%    * Removes unreliable endpoints in run-up/coast-down
%    * Preserves valid transient regions
% 2. Smoothing Recommendations:
%    * |is_smooth_speed|: For cleaner speed visualization
%    * |is_smooth_tacho|: For high-noise signal recovery
% 3. Visualization:
%    * Compares raw and processed speed when smoothing applied
%    * Standardized 500×400 pixel figure size
%
% See Also
% smoothdata, cumtrapz, gradient, smooth
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function [varargout] = process_tacho(tacho, sampling_frequency, time, NameValue)

arguments
    tacho 
    sampling_frequency
    time = 0
    NameValue.is_delete_low_speed_points = false
    NameValue.low_speed_threshold = 2*pi
    NameValue.is_smooth_speed = false
    NameValue.is_smooth_tacho = false
    NameValue.is_plot_speed = false
end

% threshold
thr = (max(tacho)+min(tacho))/2;

pointNum = length(tacho);

% find two points [i] and [i+1] across the threshold
index1 = zeros(1,pointNum); % initial
index2 = zeros(1,pointNum);
nn = 1;
for i=1:1:pointNum-1
    if tacho(i)<=thr && tacho(i+1)>thr
        index1(nn) = i;
        index2(nn) = i+1;
        nn = nn+1;
    end % end if
end % end for
index1 = index1(index1~=0);
index2 = index2(index2~=0);

% generate time
if time==0
    timeEnd = pointNum/sampling_frequency;
    t = linspace(0,timeEnd, pointNum)'; % generate time series
else
    % check the length of time signal
    if length(time)~=length(tacho)
        error('Please input the time signal which has the same length with tacho signal.')
    end
    t = time;
end

% interpolation
tk = zeros(1,length(index1)); % this point represents the start point of new revolution
for i=1:1:length(index1)
    i1 = index1(i);
    i2 = index2(i);
    tk(i) = t(i1) + (t(i2)-t(i1)) / (tacho(i2)-tacho(i1)) * (thr-tacho(i1));
end % end for

% delete low speed point (optional)
if NameValue.is_delete_low_speed_points
    % Pre - allocate arrays to store the indices to be deleted at the start and end
    % Assume the maximum number of elements to be deleted is the length of tk
    indexDelStart = false(length(tk), 1); 
    indexDelEnd = false(length(tk), 1);
    
    % Check the low - speed region from the beginning of the sequence
    for i = 1:length(tk)-1
        if 2*pi/(tk(i + 1)-tk(i)) < NameValue.low_speed_threshold
            indexDelStart(i) = true;
            indexDelStart(i + 1) = true;
        else
            % Once a non - low - speed region is encountered, stop checking
            break;
        end
    end
    
    % Check the low - speed region from the end of the sequence
    for i = length(tk):-1:2
        if 2*pi/(tk(i)-tk(i - 1)) < NameValue.low_speed_threshold
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
    tk(indexDel) = [];
end

% calculate speed curve (optional)
deltat = gradient(tk);
omega_rad = 2*pi ./ deltat;
omega_rpm = omega_rad/(2*pi)*60;
omega_rpm_origin = omega_rpm; % for ploting speed curve


% smooth the speed curve (optional)
if NameValue.is_smooth_speed || NameValue.is_smooth_tacho
    % smooth
    omega_rad_smooth = smoothdata(omega_rad, 'sgolay', 20);
    omega_rad_smooth2 = smoothdata(omega_rad_smooth, 'sgolay', 20);
    omega_rpm_smooth = omega_rad_smooth2/(2*pi)*60;
    % fresh speed curve
    omega_rad = omega_rad_smooth2;
    omega_rpm = omega_rpm_smooth;
end

% smooth tacho signal (optional)
if NameValue.is_smooth_tacho
    deltat_smooth = 2*pi ./ omega_rad_smooth2;
    tk_smooth = cumtrapz(deltat_smooth,2) + tk(1);
    % fresh tk
    tk = tk_smooth;
end

% plot speed curve (optional)
if NameValue.is_plot_speed
    h = figure('Name','Speed Curve from Tacho Signal');
    if NameValue.is_smooth_speed || NameValue.is_smooth_tacho
        plot(tk, omega_rpm_origin); hold on
        plot(tk, omega_rpm, 'r');
        legend('origin data', 'after smooth')
    else
        plot(tk, omega_rpm);
    end % end if smooth
    xlabel('Time (s)')
    ylabel('RPM')
    set(h,'Position',[200, 200, 500, 400])
end % end if plot speed

% output
if nargout == 1
    varargout{1} = tk;
elseif nargout == 2
    varargout{1} = tk;
    varargout{2} = omega_rpm;
elseif nargout == 3
    varargout{1} = tk;
    varargout{2} = omega_rpm;
    varargout{3} = omega_rad;
end % end if

end % end function