%% calculateBalance - Calculate static equilibrium position of rotor system
%
% This function computes the static equilibrium position of a rotor system
% under gravity loading at near-zero rotational speeds.
%
%% Syntax
%  q0 = calculateBalance(Parameter)
%  q0 = calculateBalance(Parameter, targetTime)
%
%% Description
% |calculateBalance| determines the static equilibrium position of a rotor
% system by simulating its behavior under near-static conditions (minimal 
% rotational speed) with gravity as the only external load. The function:
% * Configures near-static simulation parameters
% * Computes system response using numerical integration
% * Extracts and averages displacement during target time interval
% * Saves the equilibrium position to a MAT-file
%
%% Input Arguments
% * |Parameter| - System parameter structure containing model configuration
% * |targetTime| - [start, end] time interval for response averaging [s] 
%   (default = [1, 1.5])
%
%% Output Arguments
% * |q0| - Equilibrium position vector (displacements for each DOF) [m]
%
%% Static Condition Configuration
% The function automatically configures near-static conditions:
% * Rotational speed: 0.001 rad/s (effectively static)
% * Acceleration phase: 1e8 seconds (effectively constant speed)
% * No deceleration
% * These settings ensure gravity dominates the system response
%
%% Response Calculation
% 1. Simulation settings:
%   * Time range: [0, targetTime(2)]
%   * Solver: ode15s (stiff system solver)
%   * Sampling: 5000 Hz
% 2. Convergence monitoring:
%   * Checks solver convergence status
%   * Displays warning if non-convergence detected
%
%% Averaging Process
% 1. Identifies time indices corresponding to |targetTime| interval
% 2. Extracts displacement vectors within this interval
% 3. Computes mean displacement along each DOF:
%      q0 = mean(q(t) for t ∈ [targetTime(1), targetTime(2)])
%
%% Output Files
% Creates MAT-file 'balancePosition.mat' containing:
% * |qBalance| - Equilibrium position vector (same as q0)
%
%% Example
%   % Load system parameters (After modeling and data saving)
%   load('modelParameter.mat');
%   % Calculate equilibrium position using default time interval
%   eqPosition = calculateBalance(rotorParams);
%   % Calculate with custom time interval
%   eqPosition = calculateBalance(rotorParams, [1.5, 2.0]);
%
%% Dependencies
%  Requires |calculateResponse| function for system simulation
%
%% Notes
% * Simulation assumes no significant dynamic effects (rotating at 0.001 rad/s)
% * Results are sensitive to gravity configuration in |Parameter|
% * Recommended targetTime values ensure startup transients have decayed
%
%% See Also
%  calculateResponse, ode15s
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


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