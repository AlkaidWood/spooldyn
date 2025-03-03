%% calculateResponse
% calculate the time history of the rotor system
%% Syntax
% [q, dq, t] = calculateResponse(Parameter, tSpan, samplingFrequency)
%% Description
% Parameter: is a struct saving the model data
%
% tSpan = [tStart, tEnd] is a 1*2 array saving the solving information
%
% samplingFrequency: is a integer equaling to 1/step
%
% q0 is n*1 vector indicating the initial value of the displacement
%
% isPlotStatus: is boolean to contol the plot of running status
%
% reduceInterval: denotes the re-sampling interval (scaler)
%
% q is time history of displacemnet (2D matrix, node * tNum)
%
% dq is time history of velocity (2D matrix, node * tNum)
% 
% t is time series (row)
%
% convergenceStr: is a str deliver the message of convergence
%
% options is the output of the Matlab function odeset() for ode solver 


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


    % calculate the status matrix
    status = cell(shaftNum,1);
    for iShaft = 1:1:shaftNum
        iVmax           = vmax * ratio(iShaft);
        iVmin           = vmin * ratio(iShaft);
        iAcceleration   = acceleration*ratio(iShaft);
        status{iShaft} = rotationalStatus(t, iVmax, duration, iAcceleration,...
                                          isDeceleration, iVmin);
    end


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