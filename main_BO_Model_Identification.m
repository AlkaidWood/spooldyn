% Identify model parameters by using Bayesian Optimization, experimental
% data, and simulation model

clc
clear
close all

%% load experiment data

% file path
root = 'G:\Experiment Data\20240325DualRotor\datafile';
fileName = [...
    'F_LP_T_P.csv';...
    'F_HP_T_P.csv' ];
fullPath = [
    root, '\', fileName(1,:);...
    root, '\', fileName(2,:) ];

% load header of experiment data files
line1 = readmatrix(fullPath(1,:), 'Range', [1 1 1 4]);
sampling_frequency = line1(1); % sampling frequency
channelNum = line1(3); % number of channels

% Sensor type: 2-displacement(um), 0-acceleration(m/s/s), 4-rotational speed
sensorType = readmatrix(fullPath(1,:), 'Range', [3 1 3 channelNum]);

% load raw data
headerLineNum = 4; % the number of lines saving the header information
rawData{1} = readmatrix(fullPath(1,:), 'HeaderLines', headerLineNum);
rawData{2} = readmatrix(fullPath(2,:), 'HeaderLines', headerLineNum);

% extract displacement data from raw data
q_exper_LP = rawData{1}(:,sensorType==2);
q_exper_HP = rawData{2}(:,sensorType==2);

% extract tacho signal from raw data
tacho_LP = rawData{1}(:,sensorType==4);
tacho_HP = rawData{2}(:,sensorType==4);
tacho_LP = tacho_LP';
tacho_HP = tacho_HP';

% generate time
time_LP = linspace(0,length(q_exper_LP)/sampling_frequency, length(q_exper_LP));
time_HP = linspace(0,length(q_exper_HP)/sampling_frequency, length(q_exper_HP));

% cut speed-up data
cut_time_HP = 29.7; % seconds
cut_time_LP = 35.6;
[~, cut_index_HP] = min(abs(time_HP-cut_time_HP));
[~, cut_index_LP] = min(abs(time_LP-cut_time_LP));
q_exper_HP = q_exper_HP(1:cut_index_HP, :);
q_exper_LP = q_exper_LP(1:cut_index_LP, :);
tacho_HP = tacho_HP(1:cut_index_HP);
tacho_LP = tacho_LP(1:cut_index_LP);
time_HP = time_HP(1:cut_index_HP);
time_LP = time_LP(1:cut_index_LP);

% unit transfer
q_exper_LP = q_exper_LP/1e6; % unit: um -> m
q_exper_HP = q_exper_HP/1e6; % unit: um -> m


%% generate agent function for experimental amplitude-frequency curve

agentFun_HP = generateAgent(q_exper_HP, tacho_HP, time_HP,...
    is_smooth_speed = true, ...
    is_smooth_tacho = true,...
    is_plot_result = true, ...
    components_index = 1);

agentFun_LP = generateAgent(q_exper_LP, tacho_LP, time_LP,...
    is_smooth_speed = true, ...
    is_smooth_tacho = true,...
    is_plot_result = true, ...
    components_index = 1);


%% establish simulation model


%% Bayesian optimization


%% test process experimental tacho signal

% extract tacho signal
[tk, omega_rpm, omega_rad] = process_tacho( tacho,...
                                            sampling_frequency,...
                                            is_delete_low_speed = true,...
                                            is_smooth_speed = true,...
                                            is_smooth_tacho = true);


%% test Synchronous Averaging

[clean_data, time_new, tk1] = synchronous_averaging(  ...
                                            q_exper_LP, ...
                                            time_LP,...
                                            tk, ...
                                            sampling_frequency, ...
                                            average_num = 7,...
                                            is_plot_result = true, ...
                                            plot_index = [1 2 3], ...
                                            frequency_lim = 200);
%% test get_fft_componets

[X, x_axis] = get_fft_componets(q_exper_LP, ...
                                time_LP,...
                                tk1, ...
                                [1 2 3 4 5 6 7 8 9],...
                                is_output_angular = true,...
                                is_plot_result = true,...
                                base_frequency_denominator = 1);

%% Sub-function: cost function

function cost = cost()
cost = [];
end


%% Sub-function: generate agent functon from experiment data

% input a signal in time domain.
% output a set of function representing the 1X~nX components of input
% signal in time domain

function agentFun = generateAgent(signal_time, tacho, time, NameValue)

arguments
signal_time
tacho
time
NameValue.base_frequency_denominator = 1
NameValue.components_index = 1
NameValue.is_clean_data = true
NameValue.average_num = 7
NameValue.is_delete_low_speed = true
NameValue.is_smooth_speed = false
NameValue.is_smooth_tacho = false
NameValue.is_plot_result = false
end

% get sampling frequency
sampling_frequency = round(mean(1./diff(time)));


% extract tacho signal
[tk, ~, ~] = process_tacho( tacho,...
                            sampling_frequency,...
                            is_delete_low_speed = NameValue.is_delete_low_speed,...
                            is_smooth_speed = NameValue.is_smooth_speed,...
                            is_smooth_tacho = NameValue.is_smooth_tacho);


if NameValue.is_clean_data
    % clean the data
    [signal_time, time, tk1] = synchronous_averaging(  ...
                                                signal_time, ...
                                                time,...
                                                tk, ...
                                                sampling_frequency, ...
                                                average_num = NameValue.average_num);
end


% get fft components
[Components, x_axis] = get_fft_componets( ...
                                signal_time, ...
                                time,...
                                tk1, ...
                                NameValue.components_index,...
                                is_output_angular = true,...
                                base_frequency_denominator = NameValue.base_frequency_denominator);

% initial for saving agent function
agentFun = cell(length(Components),1);


% fit as function
for iSignal = 1:1:length(Components)

    signal_here = Components{iSignal};

    % reshape the signal
    [row_num, column_num] = size(signal_here);
    if row_num<column_num
        signal_here = signal_here';
    end
    [row_num, column_num] = size(x_axis);
    if row_num<column_num
        x_axis = x_axis';
    end
    
    % initial agentFun for each component
    agentFun{iSignal} = cell(length(NameValue.components_index), 1);
    % fit
    for iComponent=1:1:length(NameValue.components_index)
        % 1. load data
        y = abs(signal_here(:, iComponent));
        f = x_axis; % f means frequency, here is rotational speed
        
        % 2. smooth
        y = smoothdata(y, 'sgolay', round(length(y)/20));

        % 3. fit
        smoothing_param = 0.8;  % smooth parameter（0~1，bigger smoother）
        model_pars = csaps(f, y, smoothing_param);
        model = @(x) fnval(model_pars, x);
        % model = fit(f, y, 'smoothingspline');
          
        % 3. save data
        agentFun{iSignal}{iComponent} = model;

        % 4. plot the result
        if NameValue.is_plot_result
            f_fit = linspace(f(1),f(end),length(f));
            title_str = ['Raw FFT Components and fit model [', num2str(iSignal), '-th] Signal [', num2str(iComponent), '-th] Component'];
            figure('Name',title_str)
            plot(f, abs(signal_here(:, iComponent))); hold on
            plot(f_fit, model(f_fit)); hold off
            xlabel('RPM')
            ylabel('Amplitude')
        end % end if
    end % end for iComponent
end % end for iSignal


end

%% Sub-function: process tacho signal

% input：raw tacho signal
%
% output: each start time of pulse, speed (rpm), speed (rad/s)
%
% optional: 
%
% is_delete_low_speed_points = false (by default). If you choose true, the
% function would delete all points where the speed is less than a threshold
%
% low_speed_threshold = 1 (by default). If you set 
% is_delete_low_speed_points = true, this option will work.
%
% is_smooth_speed = false (by default). If you choose true, this function 
% will smooth your speed curve.
%
% is_smooth_tacho = false (by default). If you choose true, the speed will
% be smoothed as first. According to the smoothed speed, each start time of 
% pulse will be smoothed.
%
% is_plot_speed = false (by default). If you choose true, the figure of
% speed will be generate

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

%% Sub function: Synchronous Averaging


% input:
%
% raw_data: vibration data in time domain
%
% time:
%
% tk: each start time of pulse in tacho signal
%
% sampling_frequency
% 
% output vibration data after synchronous averaging and corresponding new
% time, new tk. Because the length of the signal will reduce after noise 
% reduction
%
% optional input:
%
% average_num: the number of raw signal period used in the noise reduction. 
% For instance, if average_num = 7, 1~7 periods of raw signal will be
% averaged. Then generate the first piece of new signal after noise
% reduction. In the next step, 2~8 periods of raw signal will be
% averaged. Then generate the second piece of new signal after noise 
% reduction.
%
% angular_sampling_frequency: sampling frequency for each period in angular
% domain
%
% is_plot_result: = false by default. If you set true, the 3d spectrum will
% be gerated for both raw data and data with noise reduction
%
% T_window: = 0.5 by default. It is used for ploting result. For instance,
% if T_window = 0.5, the sampling window will be 0.5*T where T is one 
% revolution from tacho signal
%
% overlap: = 2/3 by default. It is used for ploting result. Number of 
% overlapped samples. This function uses spectrogram() to plot. In the
% spectrogram() setting, there is noverlap where
% noverlap = round(T_window*sampling_frequency*overlap)
%
% frequency_lim: = 200 by default. It is used for ploting result.
%
% plot_index: = 1 by default. It is a vector indicating the index of the
% data wating for ploting

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
        c = colorbar;
        c.Label.String = 'dB/Hz';
        grid on
        colormap jet
        % plot 3d fft after noisy reduction
        subplot(1,2,2)
        spectrogram(data(plot_index(iFigure),:),window,noverlap,nfft,sampling_frequency,'yaxis');
        ylabel('Frequency (Hz)')
        c = colorbar;
        c.Label.String = 'dB/Hz';
        grid on
        colormap jet
        set(gcf,'Position',[200, 200, 1500, 400])
    end % end for
end % end if is plot


% reshape angular signal respect to input
if row_num > column_num
    data = data';
end

end


%% Sub function: time2angular

% this function is to transfer the signal from time domain to angular
% domain by using tacho signal
%
% input: time signal, time sesquence, corresponding tacho signal (each 
% start time of pulse, tk)
%
% output: 
%
% angular_signal: angular signal
%
% thetaW: corresponding angular arrary
%
% thetaK: end point of each period
% 
% optional:
% 
% angular_sampling_frequency: number of points in each revolution for
% resampling
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

%% Sub function: get fft components


% This function is to get the frequency component respect to rotation speed
% from the vibrition displacement signal in time domain. For instance, 1X
% is the fft component represting the rotational speed, 2X is 2 times of it
%
% input: 
% time_signal: signal waiting for processing in time domain (vector or 
% % matrix). If time_signal is a matrix, this function will identify the
% dimension automatically, which means whatever your signals are arranged
% as column or row in matrix the function always return the right result.
% For instance, time_signal is a 4*10000 matrix where 4 different signals
% are saved in each row.
%
% time: time series (vector)
%
% tk: each start time of pulse in tacho signal
%
% input (optional):
%
% components_index: = 1 by default. This parameter could be a vector
% denoting the output fft components. If you want to get 1X, 3X, 6X, 10X
% components, components_index should be [1 3 6 10].
% 
% is_plot_result: = false by default, which is a name-value parameters. If
% you input is_plot_result=true, n figures will be generated where n is the
% number of your input signals. These figure represent the different fft
% components as you inputted.
%
% base_frequency_denominator: = 1 by default. which is a scalar. If you
% input base_frequency_denominator=n and components_index=[1 3 6 10], you
% would get (1/n)X, (3/n)X, (6/n)X, (10/n)X fft components.
%
% output:
% X: is a n*1 cell data where n is the number of your input signals (For 
% instance, if time_signal is a 4*10000 matrix, n = 4). In each elements of
% X, there is a a*b matrix where one of [a,b] is the length of 
% components_index you inputed. For instance, if time_signal is a 
% 4*10000 matrix and components_index=[1 3 6 10 11], a=5, b=10000; if 
% time_signal is a 10000*4 matrix and components_index=[1 3 6 10 11 13], 
% a=10000, b=6.099
%
% omega_rpm: is a vector corresponding to the each elements in X (rpm)

function [X, x_axis] = get_fft_componets(time_signal, time, tk, components_index, NameValue)

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