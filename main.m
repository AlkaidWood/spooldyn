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
TEND = 3;
SAMPLINGFREQUENCY = 20000;
ISPLOTSTATUS = true;
REDUCEINTERVAL = 1;
calculateMethod = 'ode15s'; % RK: classic Runge-Kutta; ode45: using ode45(); ode15s: using ode15s()
isUseBalanceAsInitial = true;
isFreshInitial = true;

% initial value
if isUseBalanceAsInitial && isFreshInitial
    q0 = calculateBalance(Parameter);
elseif isUseBalanceAsInitial && ~isFreshInitial
    % detect exist initial value
    isExistInitialValue = exist('balancePosition.mat', 'file');
    if isExistInitialValue
        % check the dimension of the initial value
        load('balancePosition.mat','qBalance')
        if length(qBalance)== Parameter.Mesh.dofNum
            q0 = qBalance;
        else
            q0 = calculateBalance(Parameter);
        end % end if length(qBalance)~=
    else
        q0 = calculateBalance(Parameter);
    end % end if isExistInitialValue
else
    q0 = zeros(Parameter.Mesh.dofNum,1);
end

% calculate
tic
[q, dq, t, convergenceStr] = calculateResponse(...
    Parameter,...
    [TSTART, TEND],...
    SAMPLINGFREQUENCY,...
    q0,...
    'isPlotStatus', ISPLOTSTATUS,...
    'reduceInterval', REDUCEINTERVAL, ...
    'calculateMethod',calculateMethod);
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
SwitchFigure.fftSteady          = true;
SwitchFigure.fftTransient       = false;
SwitchFigure.poincare           = false;
SwitchFigure.saveFig            = true;
SwitchFigure.saveEps            = false;

signalProcessing(q, dq, t,...
                 Parameter, SwitchFigure, tSpan, SAMPLINGFREQUENCY,...
                 'reduceInterval', REDUCEINTERVAL,...
                 'fftTimeInterval', 0.5,...
                 'fftisPlot3DTransient', false,...
                 'fftSuperpositionRatio', 0.5,...
                 'fftXlim', 400,...
                 'isPlotInA4', true,...
                 'fftSteadyLog', true)


%% Real-time monitor

% plot part of response
% close all
% dofNo = 2;
% 
% figure
% plot(t,q(dofNo,:))
% set(gcf,'unit','centimeters','position',[3 12 35 8])
% set(gca,'Position',[.05 .1 .92 .75])
% 
% figure
% plot(t,dq(dofNo,:))
% set(gcf,'unit','centimeters','position',[3 1.5 35 8])
% set(gca,'Position',[.05 .1 .92 .75])
% 
% figure
% plot(q(dofNo*4-3,:), q(dofNo*4-2,:))
% set(gcf,'unit','centimeters','position',[3 3 20 16])
% set(gca,'Position',[.05 .1 .92 .75])
