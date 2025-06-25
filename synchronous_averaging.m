%--------------------------------------------------------------------------
% Function: synchronous_averaging
% Description:
%   This function performs synchronous averaging on vibration data in the time domain. 
%   Synchronous averaging is a noise reduction technique that averages multiple periods 
%   of a signal to enhance the periodic components and reduce random noise. 
%   The function converts the input signal to the angular domain, performs the averaging, 
%   and then maps the result back to the time domain. It also provides an option to plot 
%   the 3D spectrum of both the raw data and the noise - reduced data.
%
% Inputs:
%   - raw_data: 
%       Vibration data in the time domain. It can be a vector or a matrix. 
%       If it is a matrix, each row or column can represent a different signal. 
%       The function will automatically handle the data arrangement.
%   - time: 
%       A vector representing the time sequence corresponding to the raw_data. 
%       It should have the same length as the raw_data if it is a vector, 
%       or the same number of columns as the raw_data if it is a matrix.
%   - tk: 
%       A vector containing the start time of each pulse in the tacho signal. 
%       This information is used to define the periods of the signal for averaging.
%   - sampling_frequency: 
%       The sampling frequency of the raw_data. It is used for time - domain interpolation 
%       and plotting the spectrogram.
%
% Optional Inputs:
%   - NameValue: 
%       A structure with the following name - value pairs:
%       - average_num: 
%           Default value is 7. It represents the number of raw signal periods used in the 
%           noise reduction. For example, if average_num = 7, the first 7 periods of the 
%           raw signal will be averaged to generate the first piece of the noise - reduced 
%           signal. Then, the 2nd to 8th periods will be averaged for the second piece, and so on.
%       - angular_sampling_frequency: 
%           Default value is 100. It is the sampling frequency for each period in the angular domain. 
%           This parameter determines the resolution of the angular - domain signal.
%       - is_plot_result: 
%           Default value is false. If set to true, the function will generate a 3D spectrum plot 
%           for both the raw data and the noise - reduced data.
%       - T_window: 
%           Default value is 0.5. It is used for plotting the result. The sampling window 
%           for the spectrogram is calculated as T_window * T, where T is one revolution 
%           from the tacho signal.
%       - overlap: 
%           Default value is 2/3. It is used for plotting the result. It represents the number 
%           of overlapped samples in the spectrogram. The number of overlapped samples is 
%           calculated as round(T_window * sampling_frequency * overlap).
%       - frequency_lim: 
%           Default value is 200. It is used for plotting the result. It limits the frequency 
%           range in the spectrogram.
%       - plot_index: 
%           Default value is 1. It is a vector indicating the index of the data to be plotted. 
%           For example, if plot_index = [1, 3], the 1st and 3rd signals in the raw_data 
%           will be plotted.
%
% Outputs:
%   - data: 
%       The vibration data after synchronous averaging. It has the same number of rows as 
%       the input raw_data (after potential reshaping) and a number of columns determined 
%       by the new time sequence.
%   - time_new: 
%       A vector representing the new time sequence corresponding to the noise - reduced data. 
%       The length of the signal is reduced after noise reduction.
%   - tk1: 
%       A vector containing the start time of each new period after noise reduction.
%
% Example:
%   [data, time_new, tk1] = synchronous_averaging(raw_data, time, tk, sampling_frequency, ...
%       'average_num', 5, 'angular_sampling_frequency', 150, 'is_plot_result', true, ...
%       'T_window', 0.6, 'overlap', 0.7, 'frequency_lim', 250, 'plot_index', [2, 4]);
%
% Notes:
%   - The input raw_data and time should be consistent in terms of length or number of columns.
%   - The function uses interpolation to convert the signal between the time and angular domains.
%   - The spectrogram plot is generated using the specified window, overlap, and frequency limit.
%--------------------------------------------------------------------------

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