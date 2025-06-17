%CALCULATERESPONSE Compute time-domain response of rotor-bearing system
%
% Syntax:
%   [q, dq, t] = calculateResponse(Parameter, tSpan, samplingFrequency)
%   [q, dq, t, convergenceStr] = calculateResponse(Parameter, tSpan, samplingFrequency, q0, NameValueArgs)
%
% Input Arguments:
%   Parameter - System configuration structure containing:
%       .Mesh: [1×1 struct]            Discretization data with .dofNum
%       .Status: [1×1 struct]         Operation parameters:
%           .vmax, .vmin: double      Max/min rotational speeds (rad/s)
%           .acceleration: double     Rotational acceleration (rad/s²)
%           .isUseCustomize: logical  Custom speed profile flag
%   tSpan - [1×2 double]              Time range [tStart, tEnd] (seconds)
%   samplingFrequency - double        Sampling rate (Hz)
%   q0 - [n×1 double]                 Initial displacement vector (optional)
%   NameValueArgs - Optional parameters:
%       .isPlotStatus: logical       Plot rotational status (default: true)
%       .reduceInterval: integer     Data downsampling factor (default: 1)
%       .calculateMethod: char       Solver type: 'RK'/'ode45'/'ode15s' (default: 'RK')
%       .options: struct             ODE solver options (ode45/ode15s only)
%       .isUseBalanceAsInitial: logical Use balanced position as initial
%       .isFreshInitial: logical      Force new balance calculation
%
% Output Arguments:
%   q - [n×m double]                 Displacement time history (DOFs × time)
%   dq - [n×m double]                Velocity time history
%   t - [1×m double]                 Time vector
%   convergenceStr - char            Convergence status message
%
% Description:
%   Solves rotor system dynamics using specified numerical method:
%   - Runge-Kutta (4th order) for time-domain integration
%   - MATLAB ODE solvers (ode45/ode15s) for stiff/non-stiff systems
%   - Supports automatic initial condition generation from static balance
%   - Includes data downsampling for large datasets
%   - Monitors numerical convergence during simulation
%
% Examples:
%   % Basic usage with default parameters
%   [q, dq, t] = calculateResponse(sysParams, [0 10], 1000);
%
%   % Use ODE15s solver with custom options
%   opts = odeset('RelTol',1e-6);
%   [q, dq, t] = calculateResponse(sysParams, [0 5], 2000, ...
%       'calculateMethod', 'ode15s', 'options', opts);
%
% See also DYNAMICEQUATION, RUNGEKUTTA, ODE45, ODE15S, CALCULATEBALANCE
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.

function [q, dq, t, convergenceStr] = calculateResponse(Parameter, tSpan, samplingFrequency, q0, NameValueArgs)
% check input
arguments % name value pair
    Parameter
    tSpan
    samplingFrequency
    q0 = 0; % initial value of the displacement
    NameValueArgs.isPlotStatus = true; 
    NameValueArgs.reduceInterval = 1;
    NameValueArgs.calculateMethod = 'RK';
    
    NameValueArgs.options = odeset();
    NameValueArgs.isUseBalanceAsInitial = false;
    NameValueArgs.isFreshInitial = false
end


%%

% initial value
if q0 ==0
    isUseBalanceAsInitial = NameValueArgs.isUseBalanceAsInitial;
    isFreshInitial = NameValueArgs.isFreshInitial;
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
    end % end if isUseBalanceAsInitial && isFreshInitial
end % end if q0 ~= 0

%%

% generate time series
tStart = tSpan(1);
tEnd = tSpan(2);
tNum = floor((tEnd - tStart) * samplingFrequency);
step = 1/samplingFrequency;
t = linspace(tStart,tEnd,tNum);

%%

% initial the response
dofNum   = Parameter.Mesh.dofNum;
dyn = zeros(dofNum,1);
if q0==0
    yn = zeros(dofNum,1); % initial condition of differential euqtion
else
    if length(q0)~=dofNum
        error('Wrong dimension of the initial value. The dimension should equal to total dof number.')
    else
        yn = q0;
    end % end if 
end % end if


convergenceStr = [];
% calculate with classic Runge-Kutta Method
if strcmp(NameValueArgs.calculateMethod, 'RK')
    q = zeros(dofNum, tNum); % for saving response
    dq = zeros(dofNum, tNum);
    equation = @(tn,yn,dyn)dynamicEquation(tn,yn,dyn,Parameter);
    % calculate response
    for iT = 1:1:tNum
        [q(:,iT), dq(:,iT)] = rungeKutta(equation, t(iT), yn, dyn, step);
        yn = q(:,iT);
        dyn = dq(:,iT);
        if isnan(dyn)
            convergenceStr = ['t=', num2str(t(iT)), 's non-convergent'];
            break
        end % end if isnan
    end % end for iT
elseif strcmp(NameValueArgs.calculateMethod, 'ode45') % use ode45 as solver
    odefun = @(tn, yn) [yn(dofNum+1:end, 1); dynamicEquation(tn,yn(1:dofNum, 1), yn(dofNum+1:end, 1), Parameter)];
    [~, yode] = ode45(odefun, t, [yn; dyn], NameValueArgs.options);
    yode = yode';
    q = yode(1:dofNum, :);
    dq = yode(dofNum+1:end, :);
elseif strcmp(NameValueArgs.calculateMethod, 'ode15s') % use ode15s as solver
    odefun = @(tn, yn) [yn(dofNum+1:end, 1); dynamicEquation(tn,yn(1:dofNum, 1), yn(dofNum+1:end, 1), Parameter)];
    [~, yode] = ode15s(odefun, t, [yn; dyn], NameValueArgs.options);
    yode = yode';
    q = yode(1:dofNum, :);
    dq = yode(dofNum+1:end, :);
else
    error('Error: wrong nameValue, please use RK or ode45 or ode15s.')
end % end if NameValueArgs.calculateMethod

%%

%reduce data (re-sampling)
reduce_index = 1 : NameValueArgs.reduceInterval : size(q, 2); % index of the sampling data
if reduce_index(end) ~= size(q, 2)
   reduce_index = [reduce_index, size(q, 2)]; 
end
q  = q(:, reduce_index);% re_sampling
dq = dq(:,reduce_index);
t  = t(:,reduce_index);

%% 

% calculate the acceleration, rotational velocity and phase
if NameValueArgs.isPlotStatus
    % load constants
    Status   = Parameter.Status;
    shaftNum = Parameter.Shaft.amount;
    dofNum   = Parameter.Mesh.dofNum;
    vmax            = Status.vmax;
    duration        = Status.duration;
    acceleration    = Status.acceleration;
    isDeceleration  = Status.isDeceleration;
    vmin            = Status.vmin;
    ratio           = Status.ratio;
    ratio           = [1; ratio]; % the first shaft is basic

    
    % initial the status matrix
    omega 	= zeros(dofNum,length(t));
    domega  = omega;
    ddomega = omega;

    status = cell(shaftNum,1);

    if ~Status.isUseCustomize % using default status
        % calculate the status matrix
        for iShaft = 1:1:shaftNum
            iVmax           = vmax * ratio(iShaft);
            iVmin           = vmin * ratio(iShaft);
            iAcceleration   = acceleration*ratio(iShaft);
            status{iShaft} = rotationalStatus(t, iVmax, duration, iAcceleration,...
                                              isDeceleration, iVmin);
        end % end for iShaft
    else 
        % using customized status
        % initial
        for iShaft = 1:1:shaftNum
            status{iShaft} = zeros(3,length(t)); % initial
        end % end for iShaft
        % calculate status
        for iTime = 1:1:length(t)
            [ddomega_here, domega_here, omega_here] = Status.customize(t(iTime));
            for iShaft = 1:1:shaftNum
                status{iShaft}(1,iTime) = omega_here(iShaft);
                status{iShaft}(2,iTime) = domega_here(iShaft);
                status{iShaft}(3,iTime) = ddomega_here(iShaft);
            end % end for iShaft
        end % end for iTime
        
    end % end if


    % assemble the status matrix
    Node = Parameter.Mesh.Node;
    dofOnNodeNo = Parameter.Mesh.dofOnNodeNo;
    for iDof = 1:1:dofNum
        isBearing = Node( dofOnNodeNo(iDof) ).isBearing;
        if ~isBearing
            shaftNo = Node( dofOnNodeNo(iDof) ).onShaftNo;
            omega(iDof,:)   = status{shaftNo}(1,:);
            domega(iDof,:)  = status{shaftNo}(2,:);
            ddomega(iDof,:) = status{shaftNo}(3,:);
        end
    end


    % plot running status
    plotRunningStatus(t,status)
end



end