%--------------------------------------------------------------------------
% This example creates a two-disk rotor system during run-up [0,3000] RPM.
%--------------------------------------------------------------------------

clc
clear
close all

% input all positional, physical, and geometric parameters of the rotor
InitialParameter = inputEssentialParameterSingle2(); % all input parameters are saved in this function file; you can open and check

% establish model automatically
Parameter = establishModel(InitialParameter); % this function will create the global matrices in workspace (check Parameter), model and mesh diagram in folders: <modelDiagram>, <meshDiagram>

% generate dynamic equations
generateDynamicEquation(Parameter); % function to generate the dynamic equations function file <dynamicEquation.m> in root folder

% calculate response (set simulation time from 0 to 10 seconds, sampling frequency 2^15)
[q, dq, t] = calculateResponse(Parameter, [0,50], 2^14, calculateMethod='ode15s'); % function to calculate the system response

% signal post-processing
SwitchFigure.displacement       = true; % true-> output time history in <signalProcess> folder
SwitchFigure.axisTrajectory     = false;
SwitchFigure.axisTrajectory3d   = false;
SwitchFigure.phase              = false;
SwitchFigure.fftSteady          = false;
SwitchFigure.fftTransient       = false;
SwitchFigure.poincare           = false;
SwitchFigure.poincare_phase     = false;
SwitchFigure.saveFig            = true;
SwitchFigure.saveEps            = false;

signalProcessing(q, dq, t, Parameter, [0,50], 2^14, SwitchFigure) % output figures with time span [0,10]; sampling frequency should be the same as above


%-----------------------------------------------------------------------------------------------------------
% After running this script, please check folders in the root: <meshDiagram> <modelDiagram> <signalProcess>
%-----------------------------------------------------------------------------------------------------------

