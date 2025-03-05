%--------------------------------------------------------------------------
% Function: process_tacho
% Description:
%   This function processes a raw tacho signal to extract the start time of each 
%   pulse and calculate the rotational speed in both RPM (revolutions per minute) 
%   and rad/s (radians per second). It also provides several optional processing 
%   steps such as deleting low - speed points, smoothing the speed curve, smoothing 
%   the tacho signal, and plotting the speed curve.
%
% Inputs:
%   - tacho: 
%       The raw tacho signal, which is a vector representing the tacho sensor 
%       output over time.
%   - sampling_frequency: 
%       The sampling frequency of the tacho signal. It is used to generate the 
%       time series corresponding to the tacho signal.
%
% Optional Inputs:
%   - NameValue: 
%       A structure with the following name - value pairs:
%       - is_delete_low_speed_points: 
%           Default value is false. If set to true, the function will delete all 
%           points where the speed is less than the specified low - speed threshold.
%       - low_speed_threshold: 
%           Default value is 1. This parameter is used when is_delete_low_speed_points 
%           is set to true. It defines the speed threshold below which points are deleted.
%       - is_smooth_speed: 
%           Default value is false. If set to true, the function will smooth the 
%           speed curve using the Savitzky - Golay filter.
%       - is_smooth_tacho: 
%           Default value is false. If set to true, the speed will be smoothed first, 
%           and then the start time of each pulse will be smoothed based on the 
%           smoothed speed.
%       - is_plot_speed: 
%           Default value is false. If set to true, the function will generate a 
%           figure showing the speed curve.
%
% Outputs:
%   - varargout: 
%       The number and type of outputs depend on the number of output arguments 
%       specified when calling the function:
%       - If nargout = 1:
%           varargout{1} is a vector containing the start time of each pulse in the tacho signal.
%       - If nargout = 2:
%           varargout{1} is the start time of each pulse, and varargout{2} is a vector 
%           containing the rotational speed in RPM.
%       - If nargout = 3:
%           varargout{1} is the start time of each pulse, varargout{2} is the rotational 
%           speed in RPM, and varargout{3} is the rotational speed in rad/s.
%
% Example:
%   % Get only the start time of each pulse
%   tk = process_tacho(tacho, sampling_frequency);
%   % Get the start time of each pulse and the speed in RPM
%   [tk, omega_rpm] = process_tacho(tacho, sampling_frequency, ...
%       'is_delete_low_speed_points', true, 'low_speed_threshold', 0.5);
%   % Get all outputs and plot the speed curve
%   [tk, omega_rpm, omega_rad] = process_tacho(tacho, sampling_frequency, ...
%       'is_smooth_speed', true, 'is_plot_speed', true);
%
% Notes:
%   - The function uses linear interpolation to find the exact time when the tacho 
%     signal crosses a threshold.
%   - The Savitzky - Golay filter is used for smoothing the speed curve, with a 
%     window size of 20.
%   - The low - speed point deletion is based on the time difference between consecutive 
%     start times of pulses.
%--------------------------------------------------------------------------

function [varargout] = process_tacho(tacho, sampling_frequency, NameValue)

arguments
    tacho 
    sampling_frequency
    NameValue.is_delete_low_speed_points = false
    NameValue.low_speed_threshold = 1
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
timeEnd = pointNum/sampling_frequency;
t = linspace(0,timeEnd, pointNum)'; % generate time series

% interpolation
tk = zeros(1,length(index1)); % this point represents the start point of new revolution
for i=1:1:length(index1)
    i1 = index1(i);
    i2 = index2(i);
    tk(i) = t(i1) + (t(i2)-t(i1)) / (tacho(i2)-tacho(i1)) * (thr-tacho(i1));
end % end for

% delete low speed point (optional)
if NameValue.is_delete_low_speed_points
    indexDel = zeros(length(tk),1);
    for i = 1:1:length(tk)-1
        if tk(i+1)-tk(i) > 1
            indexDel(i) = i;
            indexDel(i+1) = i+1;
        end % end if
    end % end for
    indexDel(indexDel==0) = [];
    tk(indexDel) = [];
end % end if

% calculate speed curve (optional)
if nargout > 1
    deltat = gradient(tk);
    omega_rad = 2*pi ./ deltat;
    omega_rpm = omega_rad/(2*pi)*60;
    omega_rpm_origin = omega_rpm; % for ploting speed curve
end


% smooth the speed curve (optional)
if (nargout>1 && NameValue.is_smooth_speed) || NameValue.is_smooth_tacho
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
        plot(tk, omega_rpm_origin, 'o'); hold on
        plot(tk, omega_rpm, 'r');
        legend('origin data', 'after smooth')
    else
        plot(tk, omega_rpm, 'o');
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