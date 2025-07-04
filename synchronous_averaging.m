%% synchronous_averaging - Perform synchronous averaging for vibration signal noise reduction
%
% This function implements synchronous averaging to enhance periodic components 
% in vibration signals while suppressing random noise. It transforms signals 
% to the angular domain for precise period-based averaging and provides 
% comprehensive visualization of noise reduction effects.
%
%% Syntax
%   [data, time_new, tk1] = synchronous_averaging(raw_data, time, tk, sampling_frequency)
%   [data, time_new, tk1] = synchronous_averaging(_, NameValues)
%
%% Description
% |synchronous_averaging| applies advanced noise reduction through:
% * Angular domain transformation using tacho pulse timing
% * Multi-period synchronous averaging
% * Adaptive period overlap control
% * Time-domain signal reconstruction
% * Optional 3D spectral comparison visualization
%
%% Input Arguments
% * |raw_data| - Raw vibration data [matrix]:
%   * Rows: Signals (automatically oriented)
%   * Columns: Time samples
% * |time| - Time vector [1×n]:
%   * Corresponding time points (seconds)
% * |tk| - Tacho pulse times [vector]:
%   * Start times of rotational periods (seconds)
% * |sampling_frequency| - Original sampling rate [Hz]
%
%% Name-Value Pair Arguments
% * |'average_num'| - Averaging period count [integer]:
%   * Default: 7
%   * Example: 5 → Average 5 consecutive periods
% * |'angular_sampling_frequency'| - Angular domain resolution [Hz]:
%   * Default: 100
%   * Determines points per revolution
% * |'is_plot_result'| - Visualization flag [logical]:
%   * |false|: No plots (default)
%   * |true|: Generate 3D spectrum comparisons
% * |'T_window'| - Spectrogram window duration [s]:
%   * Default: 0.5
%   * Relative to rotational period
% * |'overlap'| - Spectrogram overlap ratio [0-1]:
%   * Default: 2/3
% * |'frequency_lim'| - Spectral display limit [Hz]:
%   * Default: 200
% * |'plot_index'| - Signal indices to plot [vector]:
%   * Default: 1
%   * Example: [1 3] → Plot 1st and 3rd signals
%
%% Output Arguments
% * |data| - Noise-reduced data [matrix]:
%   * Same orientation as input
%   * Reduced length due to averaging
% * |time_new| - New time vector [1×m]:
%   * Corresponds to processed data
% * |tk1| - Processed period start times [vector]:
%   * Updated tacho pulse times
%
%% Algorithm
% 1. Angular Domain Transformation:
%    $\theta_k = \text{time2angular}(t_k)$
% 2. Period Processing:
%    * Calculate new period count: 
%      $N_{\text{new}} = \left\lfloor \frac{N_{\text{total}} - N_{\text{avg}}}{N_{\text{avg}} - N_{\text{overlap}}} \right\rfloor + 1$
% 3. Synchronous Averaging:
%    $x_{\text{avg}}[n] = \frac{1}{N_{\text{avg}}} \sum_{k=0}^{N_{\text{avg}}-1} x[n + k \cdot N_{\theta}]$
% 4. Time Domain Reconstruction:
%    * Piecewise cubic interpolation
%
%% Noise Reduction Mechanism
% * Enhances periodic components locked to rotation
% * Attenuates:
%   - Random noise
%   - Non-synchronous vibrations
%   - Transient disturbances
% * Preserves:
%   - Harmonic content
%   - Sub/super-harmonics
%   - Periodic modulations
%
%% Example
% Process vibration data with visualization
% fs = 5000; % Sampling frequency (Hz)
% t = 0:1/fs:10; % Time vector
% raw_vib = 20*randn(3,length(t)) + 10*sin(2*pi*50*t); % 3-channel vibration
% tk = 0:0.1:10; % Tacho pulses (10Hz rotation)
% 
% % Apply synchronous averaging
% [clean_data, t_new, tk_new] = synchronous_averaging(raw_vib, t, tk, fs, ...
%     'average_num', 5, ...
%     'angular_sampling_frequency', 120, ...
%     'is_plot_result', true, ...
%     'frequency_lim', 300, ...
%     'plot_index', [1 3]);
%
%% Application Areas
% * Gearbox fault detection
% * Bearing defect diagnosis
% * Rotor imbalance analysis
% * Periodic component extraction
% * Machine condition monitoring
%
%% Implementation Notes
% 1. Signal Orientation:
%    * Automatically handles row/column vectors
%    * Maintains original channel orientation
% 2. Angular Sampling:
%    * Uniform angular resampling via |time2angular|
% 3. Overlap Control:
%    * Overlap periods = average_num - 1
% 4. Interpolation:
%    * Piecewise cubic Hermite interpolation (PCHIP)
%    * Preserves signal shape and extrema
%
%% References
% 1. Randall, R.B. (2011). Vibration-Based Condition Monitoring.
% 2. McFadden, P.D. (1986). "Synchronous Averaging of Gear Vibration."
%
%% See Also
% time2angular, spectrogram, interp1, pchip
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function [data, time_new, tk1] = synchronous_averaging(raw_data, time, tk, sampling_frequency, NameValue)

arguments
raw_data
time
tk
sampling_frequency
NameValue.average_num = 7
NameValue.angular_sampling_frequency = 100
NameValue.is_plot_result = false
NameValue.T_window = 0.5
NameValue.overlap = 2/3
NameValue.frequency_lim = 200
NameValue.plot_index = 1
end

T_window = NameValue.T_window;
overlap = NameValue.overlap;
frequency_lim = NameValue.frequency_lim;
plot_index = NameValue.plot_index;

% define some important parameter
average_num = NameValue.average_num; % number of revolutions used in average (noise reduction)
angular_sampling_frequency = NameValue.angular_sampling_frequency; % sampling frequency in angular domain (each period)
overlap_num = average_num-1; % number of overlapping revolutions



% reshape time signal (each signal in row)
row_num = size(raw_data,1);
column_num = size(raw_data,2);
if row_num > column_num
    raw_data = raw_data';
end


% convert signal to angular domain
[angular_signal, ~, thetaK] = time2angular( raw_data, ...
                                            time,...
                                            tk, ...
                                            angular_sampling_frequency = angular_sampling_frequency);


% calculate key parameters after reducing noise
revolution_num_new = floor((length(thetaK)-1-average_num)/(average_num-overlap_num)) + 1; % number of revolution after reducing noise
thetaK1 = 0:2*pi:revolution_num_new*2*pi; % end point of each period after reducing noise
delta_theta = 2*pi/angular_sampling_frequency; % the gap between adjacent points in angular domain
thetaW1 = 0:delta_theta:thetaK1(end);
tk1 = tk(floor(average_num/2)+1:length(thetaK1)+floor(average_num/2)); % start point of new revolution
tW1 = interp1(thetaK1,tk1,thetaW1,'pchip'); % interpolation of time corresponding to thetaW


% Synchronous Averaging
angular_signal_new = zeros(size(angular_signal,1),length(tW1));% initial
for iRow = 1:1:size(angular_signal_new,1)
    for iAve = 1:1:revolution_num_new
        index1 = (iAve-1)*angular_sampling_frequency + 1; % start point of averaging
        index2 = index1+angular_sampling_frequency-1; % end point of averaging (1st loop)
        signal_here = zeros(1,angular_sampling_frequency); % initial
        for iRev=1:1:average_num
            signal_one_revolution = angular_signal(iRow,index1:index2);
            signal_here = signal_here + signal_one_revolution;
            index1 = index2 + 1;
            index2 = index2 + angular_sampling_frequency; % update index
        end % end for iRev
        angular_signal_new(iRow,(iAve-1)*angular_sampling_frequency+1:iAve*angular_sampling_frequency) = signal_here/average_num;
    end % end for iAve
end % end for iRow


% map the angular data to time domain
% generate the time with certain gap (1/sampling Frequency)
time_new = tW1(1):1/sampling_frequency:tW1(end);
% interpolation of the signal after reduction noise in correspondence of t1
data = zeros(size(raw_data,1),size(time_new,2));
for iRow=1:1:size(raw_data,1)
    data(iRow, :) = interp1(tW1, angular_signal_new(iRow,:), time_new, 'pchip');
end


% plot
if NameValue.is_plot_result
    % plot setting
    window = round(T_window*sampling_frequency); % time window [s]
    noverlap = round(window*overlap); % overlap point number
    nfft = 1:frequency_lim;
    
    for iFigure = 1:1:length(plot_index)
        figure_name = ['3D Spectrum for the ',num2str(plot_index(iFigure)),'-th Data'];
        figure('Name',figure_name)
        % plot original 3d fft [Hz]
        subplot(1,2,1)
        spectrogram(raw_data(plot_index(iFigure),:),window,noverlap,nfft,sampling_frequency,'yaxis');
        ylabel('Frequency (Hz)')
        xlabel('Time (s)')
        c = colorbar;
        c.Label.String = 'dB/Hz';
        grid on
        colormap(cool)
        % plot 3d fft after noisy reduction
        subplot(1,2,2)
        spectrogram(data(plot_index(iFigure),:),window,noverlap,nfft,sampling_frequency,'yaxis');
        ylabel('Frequency (Hz)')
        xlabel('Time (s)')
        c = colorbar;
        c.Label.String = 'dB/Hz';
        grid on
        colormap(turbo)
        set(gcf,'Position',[200, 200, 1500, 400])
    end % end for
end % end if is plot


% reshape angular signal respect to input
if row_num > column_num
    data = data';
end

end