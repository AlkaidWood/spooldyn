%% time2angular - Convert time-domain signals to angular domain
%
% This function transforms time-domain vibration signals into the angular 
% domain based on tacho pulse information, enabling rotation-synchronous 
% analysis of periodic machinery behavior.
%
%% Syntax
%   [angular_signal, thetaW, thetaK] = time2angular(time_signal, time, tk)
%   [angular_signal, thetaW, thetaK] = time2angular(_, Name, Value)
%
%% Description
% |time2angular| performs rotational domain conversion through:
% * Tacho pulse-driven period segmentation
% * Uniform angular resampling
% * Time-angle mapping via interpolation
% * Automatic signal orientation handling
%
%% Input Arguments
% * |time_signal| - Time-domain signals [matrix]:
%   * Rows: Signals (automatically oriented)
%   * Columns: Time samples
% * |time| - Time vector [1×n]:
%   * Corresponding time points (seconds)
% * |tk| - Tacho pulse times [vector]:
%   * Start times of rotational periods (seconds)
%
%% Name-Value Pair Arguments
% * |'angular_sampling_frequency'| - Angular resolution [points/rev]:
%   * Default: 100
%   * Determines angular sampling density per revolution
%
%% Output Arguments
% * |angular_signal| - Angular-domain signals [matrix]:
%   * Same orientation as input signals
%   * Uniformly sampled in rotational space
% * |thetaW| - Angular positions [vector]:
%   * Resampling points (radians)
%   * Range: 0 to 2π × revolution_count
% * |thetaK| - Period end-points [vector]:
%   * Angular positions of revolution boundaries (radians)
%
%% Algorithm
% 1. Angular Domain Setup:
%    $\theta_k = 0:2\pi:R \cdot 2\pi$
%    where $R$ = number of revolutions
% 2. Uniform Sampling:
%    $\Delta\theta = \frac{2\pi}{f_{\theta}}$
%    $\theta_W = 0:\Delta\theta:\theta_k(\text{end})$
% 3. Time-Angle Mapping:
%    $t_W = \text{cubic}(\theta_K, t_k, \theta_W)$
% 4. Signal Resampling:
%    $\text{signal}_\theta = \text{pchip}(t, \text{signal}_t, t_W)$
%
%% Key Features
% * Automatic Signal Orientation:
%   - Maintains original row/column layout
% * Precise Angular Resampling:
%   - Uniform angular sampling regardless of speed variation
% * Robust Interpolation:
%   - Cubic interpolation for time-angle mapping
%   - PCHIP for signal preservation
% * Full Angular Range:
%   - Covers 0 to 2π × revolution_count
%
%% Implementation Notes
% 1. Revolution Counting:
%    $R = \text{length}(t_k) - 1$
% 2. Angular Grid:
%    $\theta_K = [0, 2\pi, 4\pi, \dots, 2\pi R]$
% 3. Resampling Parameters:
%    $\Delta\theta = \frac{2\pi}{f_{\theta}}$
% 4. Time Interpolation:
%    Uses cubic interpolation for smooth rotation mapping
% 5. Signal Interpolation:
%    Uses Piecewise Cubic Hermite Interpolating Polynomial (PCHIP)
%
%% Example
% % Generate tacho pulses (10Hz rotation)
% fs = 1000; % Sampling frequency (Hz)
% t = 0:1/fs:5; % Time vector (5 seconds)
% tk = 0:0.1:5; % Tacho pulses (10Hz rotation)
% 
% % Create test signal (1x revolution harmonic)
% signal = sin(2*pi*10*t); 
% 
% % Convert to angular domain
% [ang_sig, theta, thetaK] = time2angular(signal, t, tk, ...
%     'angular_sampling_frequency', 360);
% 
% % Visualize angular-domain signal
% figure;
% plot(theta, ang_sig);
% xlabel('Angle (radians)'); 
% ylabel('Amplitude');
% title('1-Per-Revolution Signal in Angular Domain');
%
%% See Also
% interp1, pchip, synchronous_averaging
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function [angular_signal, thetaW, thetaK] = time2angular(time_signal, time, tk, NameValue)

arguments
    time_signal
    time
    tk
    NameValue.angular_sampling_frequency = 100
end

div_num = NameValue.angular_sampling_frequency;

% create angular domain
rev_num = numel(tk)-1; % number of revolution
thetaK = 0:2*pi:rev_num*2*pi; % end point of each period
delta_theta = 2*pi/div_num;
thetaW = 0:delta_theta:thetaK(end);

% interpolation of time corresponding to thetaW
tW = interp1(thetaK,tk,thetaW,'cubic');

% reshape time signal (each signal in row)
row_num = size(time_signal,1);
column_num = size(time_signal,2);
if row_num > column_num
    time_signal = time_signal';
end


% interpolation of the original signal in correspondence of tw
angular_signal = zeros(size(time_signal,1),size(tW,2));
for iRow=1:1:size(time_signal,1)
    angular_signal(iRow, :) = interp1(time, time_signal(iRow,:), tW, 'pchip');
end % end for

% reshape angular signal respect to input
if row_num > column_num
    angular_signal = angular_signal';
end

end