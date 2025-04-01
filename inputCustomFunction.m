%% inputRubImpact
% Input parameters about rub impact fault
%% Syntax
%  OutputParameter = inputRubImpact(InputParameter)
%% Description
% InputParameter is a struct data saving all initial parameters except 
% parameters about rub impact fault.
% 
% OutputParameter is a strct data recording all InputParameter and the data
% about rub impact fault.
%
% All of parameters about rub impact should be typed into this .m file.


%%
function OutputParameter = inputCustomFunction(InputParameter)

% input 1
% to create nodes at the force position
% -------------------------------------------------------------------------
Custom.amount                  = 2;
Custom.inShaftNo               = [1; 2];
Custom.positionOnShaftDistance = [0.5; 0.2];
% -------------------------------------------------------------------------

checkInputData(Custom);

% input 2
% to create your force
% Please edit your function by using qn, dqn, tn, omega, domega, ddomega as
% input; and output force column vector (n*1), n is dofs number of entire
% system.
% *qn: displacement of system at time tn
% *dqn: The first derivative of qn
% *tn: the n-th time step
% *omega: rotational phase of shafts
% *domega: rotational speed of shafts
% *ddomega: rotational acceleration of shafts
% -------------------------------------------------------------------------
Custom.force = @(qn, dqn, tn, omega, domega, ddomega, Parameter) customForce(qn, dqn, tn, omega, domega, ddomega, Parameter);
% -------------------------------------------------------------------------

%%

% output
OutputParameter = InputParameter;
OutputParameter.Custom = Custom;
OutputParameter.ComponentSwitch.hasCustom = true;
end