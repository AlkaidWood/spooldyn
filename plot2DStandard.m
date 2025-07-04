%% plot2DStandard - Create standardized 2D line plots for technical documentation
%
% This function creates publication-quality 2D line plots with standardized 
% formatting for engineering documents and technical reports.
%
%% Syntax
%  hFig = plot2DStandard(x, y)
%  hFig = plot2DStandard(x, y, xlabelname)
%  hFig = plot2DStandard(x, y, xlabelname, ylabelname)
%  hFig = plot2DStandard(x, y, xlabelname, ylabelname, isUsedInA4)
%  hFig = plot2DStandard(x, y, xlabelname, ylabelname, isUsedInA4, isOnlySet)
%
%% Description
% |plot2DStandard| creates standardized 2D plots with:
% * Publication-ready formatting
% * A4 document optimization
% * Flexible plotting controls
%
%% Input Arguments
% * |x| - X-axis data [vector]
% * |y| - Y-axis data [vector]
% * |xlabelname| - (Optional) X-axis label [string]:
%   * Default: 'x'
% * |ylabelname| - (Optional) Y-axis label [string]:
%   * Default: 'y'
% * |isUsedInA4| - (Optional) A4 document optimization flag [logical]:
%   * |false|: Standard view size (10×5 cm) [default]
%   * |true|: A4 document size (7.2×4 cm)
% * |isOnlySet| - (Optional) Plotting control flag [logical]:
%   * |false|: Generate plot [default]
%   * |true|: Only configure axes without plotting
%
%% Output Arguments
% * |hFig| - Figure handle:
%   * Handle to created figure
%   * [] when |isOnlySet=true|
%
%% Formatting Specifications
% 1. Line Style:
%    * Solid line ('-')
%    * Color: Deep blue (RGB: 0, 0.3008, 0.6289)
%    * Width: 0.5 points
% 2. Axes:
%    * Box: On
%    * Grid: Both axes enabled
%    * Minor ticks: Disabled
%    * Tick direction: Inward
%    * Line width: 0.5 points
% 3. Text:
%    * Font: Times New Roman
%    * Font size:
%        - Axes: 7 pt
%        - Labels: 9 pt
%    * LaTeX interpreter for math symbols
% 4. Figure Size:
%    * A4: 7.2 cm (width) × 4 cm (height)
%    * Standard: 10 cm (width) × 5 cm (height)
%
%% Application Examples
% 1. Basic plot:
%    x = 0:0.1:10; y = sin(x);
%    h = plot2DStandard(x, y);
%
% 2. A4-formatted plot with labels:
%    plot2DStandard(x, y, 'Time (s)', 'Amplitude ($\mu$m)', true);
%
%% Implementation Details
% * Default Handling:
%   * Progressive parameter initialization
%   * Sensible defaults for technical plotting
% * Efficient Workflow:
%   * Separation of plotting and configuration
%   * Reusable formatting specifications
% * Quality Assurance:
%   * Resolution-independent rendering
%
%% Best Practices
% * Use A4 size for journal publications
% * Use LaTeX syntax for mathematical labels
% * Set |isOnlySet=true| for multi-line plots
% * Customize color via additional plot commands
%
%% See Also
% plot, xlabel, ylabel, gca, gcf
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function picture = plot2DStandard(x, y, xlabelname, ylabelname, isUsedInA4, isOnlySet)

% check input
if nargin < 6
    isOnlySet = false;
end

if nargin < 5
    isUsedInA4 = false;
end

if nargin < 4
    ylabelname = 'y';
end

if nargin < 3
    xlabelname = 'x';
end

%%

% plot
if ~isOnlySet
    picture = plot(x,y,'-','LineWidth',0.5,'color',[0 0.30078125 0.62890625]);%plot
else
    picture = [];
end

%%

% set 
legend off
set(gca, ...
    'Box'         , 'on'                        , ...
    'LooseInset'  , [0,0,0,0]                   , ...
    'TickDir'     , 'in'                        , ...
    'XMinorTick'  , 'off'                       , ...
    'YMinorTick'  , 'off'                       , ...
    'TickLength'  , [.01 .01]                   , ...
    'LineWidth'   , 0.5                         , ...
    'XGrid'       , 'on'                        , ...
    'YGrid'       , 'on'                        , ...
    'FontSize'    , 7                          , ... 
    'FontName'    ,'Times New Roman'            ) 


if isUsedInA4
    set(gcf,'Units','centimeters','Position',[6 6 7.2 4]);%Set the size of figure(for A4)
else
    set(gcf,'Units','centimeters','Position',[6 6 10 5]);%Set the size of figure(for viewing)
end

xlabel(xlabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',9);
ylabel(ylabelname, 'Interpreter','latex', 'Fontname', 'Times New Roman','FontSize',9);

end
