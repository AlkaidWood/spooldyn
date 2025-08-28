%% get_fft_components - Extract rotation-related frequency components from vibration signals
%
% This function extracts specific frequency components related to rotational 
% speed from vibration signals in the time domain, such as 1X (synchronous), 
% 2X (twice rotational speed), etc. The analysis is performed in the angular 
% domain for accurate speed-dependent frequency tracking.
%
%% Syntax
%  [X, x_axis] = get_fft_components(time_signal, time, tk, components_index)
%  [X, x_axis] = get_fft_components(..., components_index, Name, Value)
%
%% Input Arguments
% * |time_signal| - Time-domain vibration signals:
%   * Matrix or vector (n_signals × n_samples or n_samples × n_signals)
%   * Signal orientation automatically detected
% * |time| - Time vector corresponding to the signals [1×n_samples]
% * |tk| - Tacho pulse timestamps (start times of rotation periods) [vector]
% * |components_index| - (Optional) Frequency component indices [vector]:
%   * Default: 1 (1X component)
%   * Example: [1 3 6 10] for 1X,3X,6X,10X
%
%% Name-Value Pair Arguments
% * |'output_type'| - Output format specification [string]:
%   * |'time'|: Time-domain output (default)
%   * |'rpm'|: Rotational speed (RPM) domain
%   * |'rev'|: Revolution count domain
% * |'is_plot_result'| - Result visualization flag [logical]:
%   * |false|: No plots (default)
%   * |true|: Generate component plots
% * |'base_frequency_denominator'| - Frequency scaling factor [scalar]:
%   * Default: 1 (standard harmonics)
%   * Example: 2 with [1 3 5] → 1/2X, 3/2X, 5/2X
%
%% Output Arguments
% * |X| - Frequency component data [cell array]:
%   * Size: {n_signals × 1}
%   * Elements: (n_components × n_points) matrices
%   * Contains complex FFT amplitudes
% * |x_axis| - Corresponding x-axis values [vector]:
%   * Time [s] for |'time'| output
%   * Rotational speed [RPM] for |'rpm'| output
%   * Revolution count for |'rev'| output
%
%% Algorithm
% 1. Angular Domain Transformation:
%    * Resamples signals to constant angular increments
%    * Uses tacho pulses for angular position mapping
% 2. Revolution Segmentation:
%    * Divides angular signals into complete revolutions
%    * Accounts for |base_frequency_denominator| groupings
% 3. Order Domain Analysis:
%    * Computes FFT on angular-domain segments
%    * Extracts specified harmonic components
% 4. Domain Mapping:
%    * Interpolates angular results back to:
%        - Time domain (piecewise cubic interpolation)
%        - RPM domain (average speed calculation)
%        - Revolution count domain
%
%% Physical Interpretation
% * |X|: Complex amplitudes of vibration components
% * |x_axis|: Reference coordinate system for component evolution
%
%% Example
% % Analyze multi-sensor vibration data
% time = linspace(0, 10, 10000); % Time vector
% tk = 0:0.1:10;                 % Tacho pulses (10Hz rotation)
% vib_data = randn(3, 10000);    % 3 vibration signals
% components = [1, 2, 3.5];      % 1X, 2X, 3.5X components
% 
% % Extract components in RPM domain with plots
% [X_components, rpm] = get_fft_components(vib_data, time, tk, components, ...
%     'output_type', 'rpm', ...
%     'is_plot_result', true, ...
%     'base_frequency_denominator', 1);
%
% % Extract sub-harmonic components
% [X_sub, revs] = get_fft_components(vib_data, time, tk, [1, 2], ...
%     'output_type', 'rev', ...
%     'base_frequency_denominator', 2); % Extracts 0.5X and 1X
%
%% Advantages
% * Speed-dependent frequency tracking
% * Sub-harmonic component analysis
% * Multiple visualization domains
% * Automatic signal orientation detection
%
%% Notes
% * Requires uniform time sampling
% * Tacho pulses must cover entire signal duration
% * Maintains phase information in complex outputs
% * Non-integer harmonics supported
%
%% See Also
% time2angular, fft, interp1
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function [X, x_axis] = get_fft_components(time_signal, time, tk, components_index, NameValue)

arguments
    time_signal
    time
    tk
    components_index = 1
    NameValue.output_type = 'time'
    NameValue.is_plot_result = false
    NameValue.base_frequency_denominator = 1
end
output_type = NameValue.output_type;
is_plot_result = NameValue.is_plot_result;
base_fre = NameValue.base_frequency_denominator;


% reshape time signal (each signal in row)
row_num = size(time_signal,1);
column_num = size(time_signal,2);
if row_num > column_num
    time_signal = time_signal';
end
signal_num = size(time_signal, 1); % the number of signals


% convert signal to angular domain
[angular_signal, thetaW, thetaK] = time2angular(time_signal, time, tk);
rev_num = round(thetaK(end) / (2*pi));
% get the divided number for each revolution
div_num = round(2*pi/mean(diff(thetaW)));


% get fft componets
% initial
outputs_num = length(components_index);
X = cell(signal_num, 1); % save the fft components
rev_num_new = length(1:base_fre:rev_num-(base_fre-1));
for iSignal = 1:1:signal_num
    X{iSignal} = zeros(outputs_num,rev_num_new);
end

% get
index = 1;
for iRev = 1 : base_fre : rev_num-(base_fre-1)
    i1 = (iRev-1)*div_num+1; % start index of the iRev-th revolution
    i2 = i1 + div_num*base_fre - 1; % end index of the iRev-th revolution
    signal_sub = angular_signal(:, i1:i2); % sub-signal (one revolution)
    % FFT in order domain
    signal_frequency = 2*fft(signal_sub,[],2)/(div_num*base_fre); % fft
    for iSignal = 1:1:signal_num
        X{iSignal}(:,index) = signal_frequency(iSignal, components_index+1)';
    end
    index = index + 1;
end


% output
switch output_type
    case 'time'
        %calculate time series (for time domain)
        new_tk = tk(1:base_fre:rev_num+1);
        new_tk = (new_tk(1:end-1) + new_tk(2:end))./2;
        % Find the index of the value in 'time' that is closest to and greater than or equal to tk(1)
        index1 = find(time >= new_tk(1), 1, 'first');
        % Find the index of the value in 'time' that is closest to and less than or equal to tk(end)
        index2 = find(time <= new_tk(end), 1, 'last');
        % Keep only the data between the two closest values and store it in 'time_new'
        if ~isempty(index1) && ~isempty(index2)
            time_new = time(index1:index2);
        else
            time_new = [];
        end
        % output
        x_axis = time_new;
        % initial output in time domain
        X_time = cell(signal_num, 1); % save the fft components
        for iSignal = 1:1:signal_num
            X_time{iSignal} = zeros(outputs_num,length(time_new));
        end
        % map the X from angular domain to time domain
        for iSignal = 1:1:signal_num
            for iComponent = 1:1:outputs_num
                X_time{iSignal}(iComponent,:) = interp1(new_tk, X{iSignal}(iComponent,:), time_new, 'pchip');
            end
        end
        % output
        X = X_time;
    case 'rpm'
        % calculate speed (for angular domain)
        deltat = gradient(tk);
        omega_rad = 2*pi ./ deltat;
        reshaped_omega_rad = reshape(omega_rad(1:end-rem(rev_num, base_fre)-1), base_fre, []);
        group_means = mean(reshaped_omega_rad,1);
        omega_rad = group_means;
        omega_rpm = omega_rad/(2*pi)*60;
        x_axis = omega_rpm;
    case 'rev'
        % output result respect to revolution circle
        rev = 1 : base_fre : rev_num-(base_fre-1);
        x_axis = rev;
    otherwise
        error('output_type should be one of time, rpm, rev.')
end

% plot result (optional)
if is_plot_result
    % make labels for curves
    labels = cell(1, outputs_num);
    for iIndex = 1:1:outputs_num
        num = components_index(iIndex)/base_fre;
        if mod(num,1) == 0
            labels{iIndex} = [num2str(num), ' X'];
        else
            labels{iIndex} = sprintf('%d/%d X', components_index(iIndex), base_fre);
        end % end if
    end % end for

    % plot
    for iSignal = 1:1:signal_num
        title_str = ['FFT for each revolution for ',num2str(iSignal),'-th signal'];
        figure('Name', title_str)
        for iComponent = 1:1:outputs_num
            plot(x_axis(1:end), abs(X{iSignal}(iComponent,:))); hold on          
        end % end for iComponent
        hold off

        %
        switch output_type
            case 'time'
                xlabel('Time (s)')
            case 'rpm'
                xlabel('RPM')
            case 'rev'
                xlabel('Number of Revolution')
        end % end switch

        ylabel('(m)')
        title(title_str)
        grid on
        legend(labels);
    end % end for iSignal  
end % end if is_plot_result


% reshape angular signal respect to input
if row_num > column_num
    for iSignal = 1:1:signal_num
        X{iSignal} = X{iSignal}';
    end
end


end