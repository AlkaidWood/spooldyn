%% change2dTransientFormat - Format 2D transient plot for publication
%
% This function standardizes the appearance of 2D transient plots 
% (e.g., spectrograms) for consistent publication-quality visualization.
%
%% Syntax
%  h = change2dTransientFormat(h, fftXlim)
%
%% Description
% |change2dTransientFormat| applies standardized formatting to 2D transient 
% plots, including:
% * Axes and tick formatting
% * Label styling with LaTeX interpreter
% * Colorbar configuration
% * Figure positioning and sizing
%
%% Input Arguments
% * |h| - Figure handle (unused in current implementation)
% * |fftXlim| - Upper frequency limit for Y-axis [Hz]
%
%% Output Arguments
% * |h| - Modified figure handle (same as input)
%
%% Formatting Specifications
% 1. Axes properties:
%   * Box: on
%   * Tick direction: in
%   * Minor ticks: off
%   * Tick length: [0.01 0.01]
%   * Line width: 0.5 pt
%   * Grid: off
%   * Font: Times New Roman, 7 pt
%   * Layer: top
%
% 2. Labels:
%   * X-label: "$t$ (s)" with LaTeX interpreter
%   * Y-label: "$f$ (Hz)" with LaTeX interpreter
%   * Font: Times New Roman, 9 pt
%
% 3. Colorbar:
%   * Label: "dB/Hz" with LaTeX interpreter
%   * Font size: 7 pt
%   * Colormap: parula
%
% 4. Figure properties:
%   * Size: 7.2 cm (width) × 6 cm (height)
%   * Position: [6, 6] cm from bottom-left corner
%   * Title: removed
%
%% Example
%   % Create spectrogram plot
%   h = figure;
%   imagesc(t, f, Pxx);
%   % Apply standardized formatting
%   h = change2dTransientFormat(h, 1000); % Set y-limit to 1000 Hz
%
%% Application Notes
% * Designed for time-frequency representations (spectrograms)
% * Ensures consistent formatting across all transient plots
% * Optimized for journal publication requirements
% * Uses LaTeX for professional mathematical typography
%
%% See Also
%  imagesc, spectrogram, colormap, colorbar
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function h = change2dTransientFormat(h, fftXlim)

set(gca, ...
        'Box'         , 'on'                        , ...
        'TickDir'     , 'in'                        , ...
        'XMinorTick'  , 'off'                       , ...
        'YMinorTick'  , 'off'                       , ...
        'TickLength'  , [.01 .01]                   , ...
        'LineWidth'   , 0.5                         , ...
        'XGrid'       , 'off'                       , ...
        'YGrid'       , 'off'                       , ...
        'FontSize'    , 7                           , ... 
        'FontName'    ,'Times New Roman'            ,...
        'layer'       , 'top') 


ylabelname = '$f$ (Hz)';
xlabelname = '$t$ (s)';

xlabel(xlabelname, 'FontName', 'Times New Roman', 'Interpreter','latex', 'FontSize',9);
ylabel(ylabelname, 'FontName', 'Times New Roman', 'Interpreter','latex', 'FontSize',9);


c = colorbar;
c.FontSize = 7;
c.Label.String = 'dB/Hz';
c.Label.Interpreter ='latex';

colormap(parula)   
ylim([0 fftXlim])
title('');
set(gcf,'Units','centimeters','Position',[6 6 7.2 6]);

end