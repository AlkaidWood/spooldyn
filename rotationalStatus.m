%% rotationalStatus - Calculate rotational parameters for machinery operation
%
% This function calculates angular position, velocity, and acceleration 
% profiles for rotating machinery during startup, steady-state, and shutdown 
% operations.
%
%% Syntax
%   status = rotationalStatus(t, vmax, duration)
%   status = rotationalStatus(t, vmax, duration, acceleration)
%   status = rotationalStatus(t, vmax, duration, acceleration, isDeceleration)
%   status = rotationalStatus(t, vmax, duration, acceleration, isDeceleration, vmin)
%
%% Description
% |rotationalStatus| computes three-phase rotational dynamics parameters:
% * Angular position (phase) [rad]
% * Angular velocity [rad/s]
% * Angular acceleration [rad/s²]
% The function models both constant-speed operations and complex operations 
% including acceleration/deceleration phases.
%
%% Inputs
% * |t| - Time vector [1×n]:
%   * Uniformly sampled time points [s]
%   * Must start at t(1) ≥ 0
%
% * |vmax| - Maximum angular velocity [scalar]:
%   * Operational speed during steady-state [rad/s]
%   * Must be > 0
%
% * |duration| - Steady-state duration [scalar]:
%   * Time at maximum speed [s]
%   * Must be ≥ 0
%
%% Optional Inputs
% * |acceleration| - Acceleration magnitude [scalar]:
%   * Absolute value for both acceleration/deceleration [rad/s²]
%   * Default: 0 (constant speed operation)
%
% * |isDeceleration| - Deceleration flag [logical]:
%   * |false|: Acceleration only (default)
%   * |true|: Full acceleration/steady/deceleration cycle
%
% * |vmin| - Minimum final speed [scalar]:
%   * Angular velocity after deceleration [rad/s]
%   * Default: 0 (complete stop)
%   * Must satisfy |vmin| ≤ |vmax|
%
%% Outputs
% * |status| - Rotational parameters [3×n matrix]:
%   * Row 1: Angular position ω(t) [rad]
%   * Row 2: Angular velocity dω/dt [rad/s]
%   * Row 3: Angular acceleration d²ω/dt² [rad/s²]
%
%% Algorithm
% Models two operational modes:
%
% 1. Constant Speed Operation (acceleration = 0):
%    d²ω/dt² = 0
%    dω/dt = vmax
%    ω(t) = vmax * t
%
% 2. Accelerated Operation (acceleration > 0):
%    a) Acceleration-only mode (isDeceleration = false):
%        - t ≤ t1 = vmax/acceleration: 
%             d²ω/dt² = acceleration
%             dω/dt = acceleration * t
%             ω(t) = 0.5 * acceleration * t²
%        - t > t1:
%             d²ω/dt² = 0
%             dω/dt = vmax
%             ω(t) = 0.5 * acceleration * t1² + vmax*(t - t1)
%
%    b) Full cycle mode (isDeceleration = true):
%        Phase 1 (acceleration): t ≤ t1 = vmax/acceleration
%        Phase 2 (steady-state): t1 < t ≤ t2 = t1 + duration
%        Phase 3 (deceleration): t2 < t ≤ t3 = t2 + (vmax-vmin)/acceleration
%        Phase 4 (final speed): t > t3
%
%% Examples
% % Constant speed operation (200 rad/s for 5 seconds)
% t = linspace(0, 8, 1000); % Time vector for 8 seconds
% status = rotationalStatus(t, 200, 5);
% plot(t, status(2,:)); title('Constant Speed Operation');
% 
% % Acceleration to 300 rad/s with 100 rad/s²
% status = rotationalStatus(t, 300, 0, 100);
% figure; plot(t, status(2,:)); title('Acceleration Only');
%
% % Full cycle: accelerate to 250 rad/s, hold for 3s, decelerate to 50 rad/s
% status = rotationalStatus(t, 250, 3, 50, true, 50);
% figure; 
% subplot(3,1,1); plot(t, status(1,:)); title('Angular Position');
% subplot(3,1,2); plot(t, status(2,:)); title('Angular Velocity');
% subplot(3,1,3); plot(t, status(3,:)); title('Angular Acceleration');
%
%% Application Notes
% 1. Time Vector:
%    * Should cover entire operational cycle (tₘₐₓ ≥ t₃)
% 2. Acceleration Handling:
%    * Positive acceleration magnitude used for both acceleration and deceleration
% 3. Edge Cases:
%    * Duration = 0: Immediate transition to deceleration after acceleration
%    * vmin = vmax: No deceleration occurs
% 4. Physical Constraints:
%    * All time values must be ≥ 0
%    * Speed parameters must satisfy |vmin| ≤ |vmax|
%
%% See Also
% gradient, cumtrapz, diff
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function status = rotationalStatus(t, vmax, duration, acceleration, isDeceleration, vmin)

% default value
if nargin < 6
    vmin = 0;
end
    
if nargin < 5
    isDeceleration = false;
end

if nargin < 4
    acceleration = 0;
end

%%

% check input
if t(1)<0 || duration < 0 
    error('time and duration must larger than 0 or equal 0');
end

% if  vmax <= 0
%     error('vmax must larger than 0');
% end

if abs(vmin) > abs(vmax)
    error('vmin should smaller than vmax')
end

%%

% constants
dataNum = length(t);
a = acceleration;
omega   = zeros(1,dataNum);
domega  = zeros(1,dataNum);
ddomega = zeros(1,dataNum);

%%

% acceleration = 0 
if (a == 0) && (isDeceleration == false)
    ddomega = zeros(1,dataNum);
    domega = vmax .* ones(1,dataNum);
    omega = vmax .* t;
elseif (acceleration == 0) && (isDeceleration == true)
    error('when acceleration equal 0, isDeceleration must be false')
end

%%

% acceleration > 0
if (abs(a) > 0) && (isDeceleration == false)
    t1 = abs(vmax/a); % the first key point (a>0 -> a=0)
    phase1 = 0.5 * a * t1^2; % the phase at t1
    
    ddomega = a                     .* (t <= t1)...
              + 0                   .* (t > t1);
          
    domega  = a .* t                .* (t <= t1)...
              + vmax                .* (t > t1);
          
    omega   = 0.5 * a .* t.^2                .* (t <= t1)...
              + ( phase1 + vmax .*(t-t1) )   .* (t > t1);
    
elseif (abs(a) > 0) && (isDeceleration == true)
    t1 = abs(vmax/a); % the first key point (a>0 -> a=0)
    phase1 = 0.5 * a * t1^2; % the phase at t1
    t2 = t1 + duration; % the end of uniform motion (a=0 -> a<0)
    phase2 = phase1 + vmax * (t2 - t1);
    t3 = t2 + (abs(vmax)-abs(vmin))/abs(a); % (a<0 -> a=0)
    phase3 = phase2 + vmax*(t3-t2) - 0.5*a*(t3-t2)^2;
    
    ddomega = a                     .* (t <= t1)...
              + 0                   .* ((t>t1) & (t<=t2))...
              - a                   .* ((t>t2) & (t<=t3))...
              + 0                   .* (t > t3);
          
    domega  = a .* t                .* (t <= t1)...
              + vmax                .* ((t>t1) & (t<=t2))...      
              + (vmax - a.*(t-t2))  .* ((t>t2) & (t<=t3))...
              + vmin                .* (t > t3);
          
    omega   = 0.5 * a .* t.^2                .* (t <= t1)...
              + (phase1 + vmax.*(t-t1))      .* ((t>t1) & (t<=t2))...
              + (phase2 + vmax.*(t-t2) - 0.5*a.*(t-t2).^2)  .* ((t>t2) & (t<=t3))...
              + (phase3 + vmin.*(t-t3))      .* (t > t3);
     
end % end if 

%%

% output
status = [omega; 
          domega; 
          ddomega];

end


