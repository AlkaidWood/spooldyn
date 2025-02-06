%% calculateBalance
% calculate the balancing positon of the rotor system in the static
% condition
%% Syntax
% q0 = calculateBalance(Parameter, targetTime)
%% Description
% Parameter: is a struct saving the model data
%
% targetTime: is a 1*2 vector indicating the cutting time where the signal 
% will be extracted to calculate. e.g. if targetTime = [2, 2.5], the 
% response signal in t=2s~2.5s will be used to get average value as
% balancing position
%
% q0 is column vector denoting the balcancing positon of each dof
%
% Note: static condition here means that the rotational speed is near 0 and
% the load is only gravity


function q0 = calculateBalance(Parameter, targetTime)

arguments
    Parameter 
    targetTime = [1,1.5];
end

%% set the static condition
Parameter.Status.acceleration = 1;
Parameter.Status.duration = 1e8;
Parameter.Status.isDeceleration = false;
Parameter.Status.ratio = 1;
Parameter.Status.vmin = 0;
Parameter.Status.vmax = 0.001;

%% Calculate the response in static condition

TSTART = 0;
TEND = targetTime(2);
SAMPLINGFREQUENCY = 5000;
ISPLOTSTATUS = false;
REDUCEINTERVAL = 1;
calculateMethod = 'ode15s'; % RK: classic Runge-Kutta; ode45: using ode45(); ode15s: using ode15s()
[q, ~, t, convergenceStr] = calculateResponse(...
    Parameter,...
    [TSTART, TEND],...
    SAMPLINGFREQUENCY,...
    'isPlotStatus', ISPLOTSTATUS,...
    'reduceInterval', REDUCEINTERVAL, ...
    'calculateMethod',calculateMethod);
if ~isempty(convergenceStr)
    fprintf('%s \n', convergenceStr)
end

%% extract response signal

% find the index of the starting time
diffs = abs(t - targetTime(1));
[~, index] = min(diffs);
indexTStart = index(1); % maybe two index here

% cut the response
qInterval = q(:, indexTStart:end);

% average the response
qBalance = mean(qInterval, 2);

%% Output 1: save the balancing displacement of the system

save('balancePosition', 'qBalance')

%% Output 2: q0

q0 = qBalance;

end