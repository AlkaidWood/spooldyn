%{
================================================================================
Function Name:
    get_tk_from_simulation

Description:
    This function computes the time points (tk) corresponding to each full revolution 
    of a rotating system based on the given rotational phase (omega) and time data. 
    It also offers an option to eliminate low - speed regions at the start and end 
    of the calculated time series.

Inputs:
    - omega: A matrix or vector representing the rotational phase of the rotating system 
             over time. Each row corresponds to a different signal if it's a matrix. 
             The values are in radians.
    - time: A matrix or vector containing the time instants corresponding to the 
            rotational phase data. It must have the same dimensions as the 'omega' input.
    - NameValue: A structure with the following optional fields:
        - is_delete_low_speed_points: A logical value (default is false). If set to true, 
                                      the function will remove low - speed regions at the 
                                      start and end of the calculated time series.
        - low_speed_threshold: A numerical value (default is 2*pi). Points with rotational 
                               speeds below this threshold are considered low - speed points.

Outputs:
    - tk: A cell array where each element stores the time points corresponding to 
          the completion of each revolution for the respective input signal. The size 
          of the cell array is equal to the number of rows in the 'omega' input.

Notes:
    - The interpolation method used is cubic interpolation via the 'interp1' function. 
      This might lead to non - physical results in some cases, so use with caution.
    - The function assumes that the input data does not contain NaN or Inf values. 
      It only checks the dimensional consistency of 'omega' and 'time'.
================================================================================
%}
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