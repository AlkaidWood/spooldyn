
function tk = get_tk_from_simulation2(Status, time, shaftNum, NameValue)

arguments
Status
time
shaftNum = 1
NameValue.is_delete_low_speed_points = false
NameValue.low_speed_threshold = 2*pi
end



%% check input

% check input Status
referenceFields = {'ratio', 'vmax', 'acceleration', 'duration', 'isDeceleration', 'vmin', 'isUseCustomize', 'customize'};
fields_name = fieldnames(Status);
missingFields = cell(1, length(referenceFields));
missingCount = 0;
for i = 1:length(referenceFields)
    field = referenceFields{i};
    if ~ismember(field, fields_name)
        missingCount = missingCount + 1;
        missingFields{missingCount} = field;
    end
end
missingFields = missingFields(1:missingCount);
% report error
if ~isempty(missingFields)
    error('Input Status struct miss fields: %s\n', strjoin(missingFields, ', '));
end

% check input time
if ~isvector(time)
    error('The input time series must be a vector.');
end

if any(diff(time) <= 0)
    error('The input time series must be monotonically increasing.');
end


%% calculate rotational angular in time domain

% load some constants
vmax            = Status.vmax;
duration        = Status.duration;
acceleration    = Status.acceleration;
isDeceleration  = Status.isDeceleration;
vmin            = Status.vmin;
ratio           = Status.ratio;
ratio           = [1; ratio]; % the first shaft is basic

% initial 
omega = zeros(shaftNum,length(time));

if ~Status.isUseCustomize % using default status
    % calculate the status matrix
    for iShaft = 1:1:shaftNum
        iVmax           = vmax * ratio(iShaft);
        iVmin           = vmin * ratio(iShaft);
        iAcceleration   = acceleration*ratio(iShaft);
        status_here = rotationalStatus(time, iVmax, duration, iAcceleration,...
                                          isDeceleration, iVmin);
        omega(iShaft,:) = status_here(1,:);
    end % end for iShaft
else 
    % using customized status
    % calculate status
    for iTime = 1:1:length(time)
        [~, ~, omega_here] = Status.customize(time(iTime));
        for iShaft = 1:1:shaftNum
            omega(iShaft, iTime) = omega_here(iShaft);
        end % end for iShaft
    end % end for iTime
    
end % end if


%% get tk (output)

time_repeat = repmat(time, shaftNum, 1);
tk = get_tk_from_simulation(omega, time_repeat, ...
    is_delete_low_speed_points = NameValue.is_delete_low_speed_points, ...
    low_speed_threshold = NameValue.low_speed_threshold);

end