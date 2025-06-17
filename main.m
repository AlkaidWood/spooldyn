clc
clear
close all

%% Input initial parameters

InitialParameter = inputEssentialParameter(); % first, input essential parameters
InitialParameter = inputBearingHertz(InitialParameter);
InitialParameter = inputIntermediateBearing(InitialParameter);

% InitialParameter = inputEssentialParameterBO(); 
% InitialParameter = inputBearingHertzBO(InitialParameter);
% InitialParameter = inputIntermediateBearingBO(InitialParameter);
% InitialParameter = inputCustomFunction(InitialParameter);
% 
% ac = 500 * (2*pi) /60; % rad/s^2
% status_LP_ac = @(tn) deal([ac, 0], [ac * tn, 0], [0.5 * ac * tn^2, 0]);% LP rotor speed up; HP rotor speed = 0
% status_HP_ac = @(tn) deal([0, ac], [0, ac * tn], [0, 0.5 * ac * tn^2]); % HP rotor speed up; LP rotor speed = 0
% InitialParameter.Status.customize = status_LP_ac;
% InitialParameter.Status.isUseCustomize = false;

% If you need change some parameters, please change the data in the struct:
% InitialParameter, then use establishModel( ) to get the different models

%% Establish models

% manualGrid{1} = [1 4 1]; manualGrid{2} = [3];
mygrid = 'low';
Parameter = establishModel(InitialParameter,...
                           'gridfineness', mygrid,...
                           'isPlotModel',  true,...
                           'isPlotMesh',   true);
% save('modelParameter','Parameter')     


%%  Generate the dynamic equation

fclose('all');
generateDynamicEquation(Parameter);            
 

%% Calculate response

% calculate parameter
TSTART = 0;
TEND = 20;
SAMPLINGFREQUENCY = 20000;
ISPLOTSTATUS = true;
REDUCEINTERVAL = 1;
calculateMethod = 'ode15s'; % RK: classic Runge-Kutta; ode45: using ode45(); ode15s: using ode15s()
options = odeset(); % if you want to use ode45() or ode15s(), you would control the options here
isUseBalanceAsInitial = true;
isFreshInitial = true;

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
tSpan = [0 20];
SwitchFigure.displacement       = true;
SwitchFigure.axisTrajectory     = true;
SwitchFigure.axisTrajectory3d   = true;
SwitchFigure.phase              = true;
SwitchFigure.fftSteady          = true;
SwitchFigure.fftTransient       = true;
SwitchFigure.poincare           = true;
SwitchFigure.poincare_phase     = true;
SwitchFigure.saveFig            = true;
SwitchFigure.saveEps            = false;

signalProcessing(q, dq, t,...
                 Parameter, tSpan, SAMPLINGFREQUENCY, SwitchFigure,...
                 'reduceInterval', REDUCEINTERVAL,...
                 'T_window', 0.7,...
                 'fftisPlot3DTransient', false,...
                 'overlap', 0.7,...
                 'fftXlim', 50,...
                 'isPlotInA4', true,...
                 'fftSteadyLog', false, ...
                 'f', 0:0.1:50)

%% Monitor
sim_dof = [9 10 41 42]; % dof used to compare with experimental data
component_index = 1;

% get tk
tk = get_tk_from_simulation2(Parameter.Status, t, Parameter.Shaft.amount,"is_delete_low_speed_points", true);

% plot title
title_str = {'LP-x', 'LP-y', 'HP-x', 'HP-y'};

iPlot = 1;
figure(Name='1X Component in Run-up', Position=[20, 150, 450, 200])
for iDof = sim_dof
    signal_here = detrend(q(iDof, :)',0); % get data and detrend 
    [X, rpm] = get_fft_components(signal_here, t, tk{1}, component_index, "output_type", 'rpm');
    subplot(2,2,iPlot)
    plot(rpm, abs(X{1}))
    xlabel('RPM')
    ylabel('1X')
    title(title_str{iPlot})
    set(gca, fontsize=7)
    xlim([0 10000])
    grid on
    

    iPlot = iPlot + 1;
end
