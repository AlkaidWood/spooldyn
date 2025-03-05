%--------------------------------------------------------------------------
% Function: time2angular
% Description:
%   This function is designed to convert a signal from the time domain to 
%   the angular domain using a tacho signal. The tacho signal provides the 
%   start time of each pulse, which helps in establishing the relationship 
%   between time and angle.
%
% Inputs:
%   - time_signal: 
%       The signal in the time domain that needs to be converted. It can be 
%       a vector or a matrix. If it is a matrix, the function will handle 
%       it appropriately regardless of whether the signals are arranged as 
%       columns or rows.
%   - time: 
%       A vector representing the time sequence corresponding to the 
%       time_signal. It should have the same length as the time_signal if 
%       it is a vector, or the same number of columns as the time_signal 
%       if it is a matrix.
%   - tk: 
%       A vector containing the start time of each pulse in the tacho signal. 
%       This information is crucial for mapping the time domain to the 
%       angular domain.
%
% Optional Inputs:
%   - NameValue: 
%       A structure with the following name - value pair:
%       - angular_sampling_frequency: 
%           Default value is 100. It represents the number of points in each 
%           revolution for resampling. This parameter determines the 
%           resolution of the angular signal.
%
% Outputs:
%   - angular_signal: 
%       The signal converted to the angular domain. It has the same number 
%       of rows as the input time_signal (after potential reshaping) and 
%       a number of columns determined by the angular sampling frequency.
%   - thetaW: 
%       A vector representing the corresponding angular array. It is used 
%       to define the angular positions for the resampled signal.
%   - thetaK: 
%       A vector containing the end - point of each period in the angular 
%       domain. It helps in dividing the angular range into revolutions.
%
% Example:
%   [angular_signal, thetaW, thetaK] = time2angular(time_signal, time, tk, ...
%       'angular_sampling_frequency', 200);
%
% Notes:
%   - The input time_signal and time should be consistent in terms of length 
%     or number of columns.
%   - The interpolation methods ('cubic' for time interpolation and 'pchip' 
%     for signal interpolation) are used to ensure smooth resampling.
%   - The function reshapes the input and output signals to handle both 
%     row - and column - arranged signals properly.
%--------------------------------------------------------------------------

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