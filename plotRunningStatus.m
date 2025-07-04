%% plotRunningStatus - Visualize rotor system operational status
%
% This function generates time-domain plots of rotational dynamics parameters 
% including phase, speed, and acceleration for multi-shaft rotor systems.
%
%% Syntax
%  plotRunningStatus(t, status)
%
%% Description
% |plotRunningStatus| creates standardized visualizations of rotor system 
% operational parameters during runtime:
% * Generates three separate plots: phase, speed, and acceleration
% * Supports multi-shaft systems with automatic legend generation
% * Saves figures in dedicated directory
% * Implements standardized formatting for publication
%
%% Input Arguments
% * |t| - Time vector [1×m]:
%   * Simulation time points [s]
%   * Must be monotonically increasing
%
% * |status| - System status data [n×1 cell array]:
%   * n: Number of shafts
%   * Each cell contains [3×m] matrix for one shaft:
%     - Row 1: Phase (ω) [rad]
%     - Row 2: Rotational speed (dω/dt) [rad/s]
%     - Row 3: Acceleration (d²ω/dt²) [rad/s²]
%   * m: Must match length of |t|
%
%% Output
% Creates in './runningStatusDiagram' directory:
% * |statusOfPhase.fig|       % Phase plot (MATLAB figure)
% * |statusOfSpeed.fig|        % Speed plot (MATLAB figure)
% * |statusOfAcceleration.fig| % Acceleration plot (MATLAB figure)
%
%% Visualization Details
% 1. Plot Configuration:
%    * Phase Plot:
%      - Y-axis: Angular position [rad]
%    * Speed Plot:
%      - Y-axis: Rotational speed [rad/s]
%    * Acceleration Plot:
%      - Y-axis: Rotational acceleration [rad/s²]
% 2. Formatting:
%    * Automatic legend: 'Shaft 1', 'Shaft 2', ...
%    * Uniform figure size: 15×8 cm
%    * Consistent time axis across plots
% 3. Layout:
%    * Phase plot position: [5, 12] cm
%    * Speed plot position: [22, 12] cm
%    * Acceleration plot position: [5, 1.5] cm
%
%% Implementation
% 1. Directory Management:
%    * Creates 'runningStatusDiagram' if missing
%    * Clears existing figures in directory
% 2. Legend Generation:
%    * Automatic shaft labeling ('Shaft n')
% 3. Plot Generation:
%    * Creates separate figures for each parameter
%    * Overlays all shafts on each plot
% 4. Output Handling:
%    * Saves figures in compact .fig format
%    * Closes figures after saving
%
%% Example
% % Simulate dual-shaft system
% t = linspace(0, 10, 1000);
% status{1} = [sin(2*pi*t); 2*pi*cos(2*pi*t); -4*pi^2*sin(2*pi*t)];
% status{2} = [1.5*sin(3*pi*t); 4.5*pi*cos(3*pi*t); -13.5*pi^2*sin(3*pi*t)];
% 
% % Generate status plots
% plotRunningStatus(t, status);
%
%% Application Notes
% * Typical Usage:
%    - Post-simulation analysis
%    - Transient response verification
%    - Speed profile validation
% * Interpretation:
%    - Phase plots show angular position evolution
%    - Speed plots reveal rotational velocity changes
%    - Acceleration plots indicate torque requirements
%
%% See Also
% calculateResponse, establishModel, plot2DStandard
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function plotRunningStatus(t, status)

% gnerate folder to save figures
hasFolder = exist('runningStatusDiagram','dir');
if hasFolder
    delete runningStatusDiagram/*.fig;
else
    mkdir('runningStatusDiagram');
end

%%

% line name
lineName = cell(length(status),1);
for iShaft = 1:1:length(status)
    lineName{iShaft} = ['Shaft ', num2str(iShaft)];
end

%%

% plot phase
figureName = 'Phase in running status';
h1 = figure('name',figureName,'Visible', 'off');

for iShaft = 1:1:length(status)
    plot(t,status{iShaft}(1,:)); hold on
end

legend(lineName{:})
set(gcf, 'unit', 'centimeters', 'position', [5 12 15 8])
% save figure
set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
figureName2 = ['runningStatusDiagram/statusOfPhase', '.fig'];
savefig(h1,figureName2,'compact')

%%

% plot speed
figureName = 'Speed in running status';
h2 = figure('name',figureName,'Visible', 'off');

for iShaft = 1:1:length(status)
    plot(t,status{iShaft}(2,:)); hold on
end

legend(lineName{:})
set(gcf, 'unit', 'centimeters', 'position', [22 12 15 8])
% save figure
set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
figureName2 = ['runningStatusDiagram/statusOfSpeed', '.fig'];
savefig(h2,figureName2,'compact')
    
%%

% plot accelerate
figureName = 'Acceleration in running status';
h3 = figure('name',figureName,'Visible', 'off');

for iShaft = 1:1:length(status)
    plot(t,status{iShaft}(3,:)); hold on
end

legend(lineName{:})
set(gcf, 'unit', 'centimeters', 'position', [5 1.5 15 8])
% save figure
set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
figureName2 = ['runningStatusDiagram/statusOfAcceleration', '.fig'];
savefig(h3,figureName2,'compact')

close([h1 h2 h3])

end