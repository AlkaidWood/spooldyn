%--------------------------------------------------------------------------
% Function: get_fft_components
% Description:
%   This function extracts the frequency components related to the rotation 
%   speed from the vibration displacement signal in the time domain. 
%   For example, 1X represents the FFT component corresponding to the 
%   rotational speed, and 2X is twice that value.
%
% Inputs:
%   - time_signal: 
%       The signal to be processed in the time domain. It can be a vector 
%       or a matrix. If it is a matrix, the function will automatically 
%       identify the dimensions. Signals can be arranged as columns or rows 
%       in the matrix, and the function will return the correct result. 
%       For instance, if time_signal is a 4x10000 matrix, 4 different signals 
%       are saved in each row.
%   - time: 
%       A vector representing the time series.
%   - tk: 
%       A vector containing the start time of each pulse in the tacho signal.
%
% Optional Inputs:
%   - components_index: 
%       Default value is 1. It can be a vector specifying the output FFT 
%       components. For example, if you want to get 1X, 3X, 6X, 10X components, 
%       set components_index to [1 3 6 10].
%   - NameValue: 
%       A structure with the following name - value pairs:
%       - is_output_angular: 
%           Default value is true. If true, the output x - axis will be in 
%           angular speed (rpm). If false, it will be in the time domain.
%       - is_plot_result: 
%           Default value is false. If set to true, n figures will be generated, 
%           where n is the number of input signals. These figures show the 
%           different FFT components as specified by components_index.
%       - base_frequency_denominator: 
%           Default value is 1. It is a scalar. If you set base_frequency_denominator = n 
%           and components_index = [1 3 6 10], you will get (1/n)X, (3/n)X, 
%           (6/n)X, (10/n)X FFT components.
%
% Outputs:
%   - X: 
%       A n x 1 cell array, where n is the number of input signals. 
%       For example, if time_signal is a 4x10000 matrix, n = 4. 
%       Each element of X is an a x b matrix, where one of [a, b] is the 
%       length of components_index. For instance, if time_signal is a 
%       4x10000 matrix and components_index = [1 3 6 10 11], a = 5, b = 10000; 
%       if time_signal is a 10000x4 matrix and components_index = [1 3 6 10 11 13], 
%       a = 10000, b = 6.
%   - x_axis: 
%       A vector corresponding to each element in X. If is_output_angular is true, 
%       it represents the angular speed in rpm. If false, it represents the time series.
%
% Example:
%   [X, x_axis] = get_fft_components(time_signal, time, tk, [1 3 6], ...
%       'is_output_angular', false, 'is_plot_result', true, 'base_frequency_denominator', 2);
%
% Notes:
%   - This function assumes that the input signals are properly sampled and 
%     the time series is consistent with the signal data.
%   - The function uses interpolation to map the results from the angular 
%     domain to the time domain when is_output_angular is false.
%--------------------------------------------------------------------------

function [X, x_axis] = get_fft_components(time_signal, time, tk, components_index, NameValue)

arguments
    time_signal
    time
    tk
    components_index = 1
    NameValue.is_output_angular = true
    NameValue.is_plot_result = false
    NameValue.base_frequency_denominator = 1
end
is_output_angular = NameValue.is_output_angular;
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

if is_output_angular
    % calculate speed (for angular domain)
    deltat = gradient(tk);
    omega_rad = 2*pi ./ deltat;
    reshaped_omega_rad = reshape(omega_rad(1:end-rem(rev_num, base_fre)-1), base_fre, []);
    group_means = mean(reshaped_omega_rad,1);
    omega_rad = group_means;
    omega_rpm = omega_rad/(2*pi)*60;
    x_axis = omega_rpm;
else
    %calculate time series (for time domain)
    new_tk = tk(1:base_fre:rev_num+1);
    new_time = (new_tk(1:end-1) + new_tk(2:end))./2;
    x_axis = time;
    % initial output in time domain
    X_time = cell(signal_num, 1); % save the fft components
    for iSignal = 1:1:signal_num
        X_tisme{iSignal} = zeros(outputs_num,length(time));
    end
    % map the X from angular domain to time domain
    for iSignal = 1:1:signal_num
        for iComponent = 1:1:outputs_num
            X_time{iSignal}(iComponent,:) = interp1(new_time, X{iSignal}(iComponent,:), time, 'pchip');
        end
    end
    % output
    X = X_time; 
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
            if is_output_angular
                plot(omega_rpm(1:end), abs(X{iSignal}(iComponent,:))); hold on
            else
                plot(time(1:end), abs(X{iSignal}(iComponent,:))); hold on
            end % end if
        end % end for iComponent
        hold off
        if is_output_angular
            xlabel('\Omega (rpm)')
        else
            xlabel('t (s)')
        end
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