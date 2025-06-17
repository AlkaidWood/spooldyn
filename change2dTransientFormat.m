
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

%set(gca, 'LooseInset'  , [0,0,0,0]);
% set(gca, 'OuterPosition', [0,0,1.09,0.92]);

end