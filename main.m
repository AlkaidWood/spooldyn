clc
clear
close all

%% Input initial parameters

InitialParameter = inputEssentialParameter(); % first, input essential parameters
InitialParameter = inputBearingHertz(InitialParameter);
InitialParameter = inputIntermediateBearing(InitialParameter);

% If you need change some parameters, please change the data in the struct:
% InitialParameter, then use establishModel( ) to get the different models

%% Establish models

% manualGrid{1} = [1 4 1]; manualGrid{2} = [3];
grid = 'low';
Parameter = establishModel(InitialParameter,...
                           'gridfineness', grid,...
                           'isPlotModel',  true,...
                           'isPlotMesh',   true);
save('modelParameter','Parameter')     


%%  Generate the dynamic equation

fclose('all');
generateDynamicEquation(Parameter);                  
 

%% Calculate response

% calculate parameter
TSTART = 0;
TEND = 10;
SAMPLINGFREQUENCY = 20000;
ISPLOTSTATUS = true;
REDUCEINTERVAL = 1;
calculateMethod = 'ode15s'; % RK: classic Runge-Kutta; ode45: using ode45(); ode15s: using ode15s()
options = odeset(); % if you want to use ode45() or ode15s(), you would control the options here
isUseBalanceAsInitial = true;
isFreshInitial = false;

% calculate
tic
[q, dq, t, convergenceStr] = calculateResponse(...
    Parameter,...
    [TSTART, TEND],...
    SAMPLINGFREQUENCY,...
    'isPlotStatus', ISPLOTSTATUS,...
    'reduceInterval', REDUCEINTERVAL, ...
    'calculateMethod',calculateMethod,...
    'options', options,...
    'isFreshInitial', isFreshInitial,...
    'isUseBalanceAsInitial', isUseBalanceAsInitial);
toc
if ~isempty(convergenceStr)
    fprintf('%s \n', convergenceStr)
end

%save('response','t','q','dq')

%% Post Proccessing

% signalProccessing
tSpan = [TSTART TEND];
SwitchFigure.displacement       = true;
SwitchFigure.axisTrajectory     = false;
SwitchFigure.axisTrajectory3d   = false;
SwitchFigure.phase              = false;
SwitchFigure.fftSteady          = false;
SwitchFigure.fftTransient       = true;
SwitchFigure.poincare           = false;
SwitchFigure.saveFig            = true;
SwitchFigure.saveEps            = false;

signalProcessing(q, dq, t,...
                 Parameter, SwitchFigure, tSpan, SAMPLINGFREQUENCY,...
                 'reduceInterval', REDUCEINTERVAL,...
                 'fftTimeInterval', 0.5,...
                 'fftisPlot3DTransient', true,...
                 'fftSuperpositionRatio', 0.5,...
                 'fftXlim', 150,...
                 'isPlotInA4', true,...
                 'fftSteadyLog', true)

