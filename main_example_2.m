%-----------------------------------------------------------------------------------
% This example creates a twin-spool rotor system with speed ratio 1.3 during run-up.
%-----------------------------------------------------------------------------------

clc
clear
close all

% input all positional, physical, and geometric parameters of the rotor
InitialParameter = inputEssentialParameterTwinSpool(); % all shaft, disk, operation parameters are saved in this function file; you can open and check
InitialParameter = inputBearingHertzTwinSpool(InitialParameter); % all bearing parameters are saved in this file (But Hertzian force is not used in this example. You can control in this file)
InitialParameter = inputIntermediateBearingTwinSpool(InitialParameter); % all inter-shft bearing parameters are saved in this file

% establish model automatically
Parameter = establishModel(InitialParameter); % this function will create the global matrices in workspace (check Parameter), model and mesh diagram in folders: <modelDiagram>, <meshDiagram>
% % establish model manually
% manualGrid{1} = [1,2,1,7,1,1,3]; % for shaft 1
% manualGrid{2} = [1,3,4,3]; % for shaft 2
% Parameter = establishModel(InitialParameter, "gridFineness", manualGrid);

% generate dynamic equations
generateDynamicEquation(Parameter); % function to generate the dynamic equations function file <dynamicEquation.m> in root folder

% calculate response (set simulation time from 0 to 10 seconds, sampling frequency 2^14)
[q, dq, t] = calculateResponse(Parameter, [0,10], 2^14, calculateMethod='ode15s'); % function to calculate the system response

% signal post-processing
SwitchFigure.displacement       = true; % true-> output time history in <signalProcess> folder
SwitchFigure.axisTrajectory     = false;
SwitchFigure.axisTrajectory3d   = false;
SwitchFigure.phase              = false;
SwitchFigure.fftSteady          = false;
SwitchFigure.fftTransient       = false;
SwitchFigure.poincare           = false;
SwitchFigure.poincare_phase     = false;
SwitchFigure.saveFig            = false;
SwitchFigure.saveEps            = false;

signalProcessing(q, dq, t, Parameter, [0,10], 2^14, SwitchFigure) % output figures with time span [0,10]; sampling frequency should be the same as above


%-----------------------------------------------------------------------------------------------------------
% After running this script, please check folders in the root: <meshDiagram> <modelDiagram> <signalProcess>
%-----------------------------------------------------------------------------------------------------------

