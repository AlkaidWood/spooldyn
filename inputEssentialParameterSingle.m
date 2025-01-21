%% inputEssentialParameter
% Input parameters about shaft(s), disk(s), linear bearings and running
% status
%% Syntax
%  inputEssentialParameter()
%% Description
% There is no input parameter.
% 
% All essential initial parameters should be typed into this .m file.
% 
% All vectors in this .m file should be in column.


%%
function InitialParameter = inputEssentialParameterSingle()

% typing the parameter about shaft
Shaft.amount            = 1;
Shaft.totalLength       = [1.5]; % all vectors in column (m)
Shaft.dofOfEachNodes    = 4 * ones(Shaft.amount,1);
Shaft.outerRadius       = [0.0125]; % m
Shaft.innerRadius       = [0]; % m
Shaft.density           = 7850 * ones(Shaft.amount,1); % kg/m^3
Shaft.elasticModulus    = 2.1e11 * ones(Shaft.amount,1); % Pa
Shaft.poissonRatio      = 0.3 * ones(Shaft.amount,1);
checkInputData(Shaft)
Shaft.rayleighDamping   = [10, 0]; % [alpha, beta] 

%%

% typing the parameter about running status
Status.ratio            = []; % [v-shaft2/v-shaft1; v-shaft3/v-shaft2]
Status.vmax             = 240; % rad/s, the maximum rotational speed for shaft 1
Status.acceleration     = 0; % rad/s^2, acceleration of shaft 1
Status.duration         = 3; % s, the duration of shaft 1 in vmax
Status.isDeceleration   = false; % boolean, add a deceleration in status
Status.vmin             = 0; % s, the minimum speed afterdeceleration

% check input
if  length(Status.ratio) >= Shaft.amount
    error('too much input parameter in Status.ratio')
end

%%

% typing the parameter about disk
Disk.amount             = 2;
Disk.inShaftNo          = [1; 1]; % disks in the i-th shaft
Disk.dofOfEachNodes     = 4 * ones(Disk.amount,1);
Disk.innerRadius        = 0 * ones(Disk.amount,1); % m
Disk.outerRadius        = 0.125 * ones(Disk.amount,1); % m
Disk.thickness          = [0.00273; 0.00273]; % m
Disk.positionOnShaftDistance = [0.5; 1]; %from left end (m)
Disk.density            = 7850 * ones(Disk.amount,1); % kg/m^3
Disk.eccentricity       = [5e-4; 5e-4]; % m

% check input
checkInputData(Disk)

for iDisk = 1:1:Disk.amount
   if Shaft.dofOfEachNodes(Disk.inShaftNo(iDisk)) ~= Disk.dofOfEachNodes(iDisk)
      error(['the dof of each disk should equal to the dof of the shaft'...
            ,' this disk locating']); 
   end
end

%%

% typing the parameter about linear bearing
Bearing.amount          = 2;
Bearing.inShaftNo       = [1; 1];
Bearing.dofOfEachNodes  = [2; 2]; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = [0; 1.5];
Bearing.stiffness       = [2.5e6; 2.5e6]; % N*m
Bearing.damping         = [500; 500]; % N*s/m
Bearing.mass            = [0.8; 0.8]; % kg

checkInputData(Bearing)

%%

% ComponentSwitch will be changed by corresponding input..() function
ComponentSwitch.hasIntermediateBearing = false;
ComponentSwitch.hasLoosingBearing = false;
ComponentSwitch.hasRubImpact = false;
ComponentSwitch.hasCouplingMisalignment = false;

%%

% Output initialParameter without optional parameter
InitialParameter.Status          = Status;
InitialParameter.Shaft           = Shaft;
InitialParameter.Disk            = Disk;
InitialParameter.Bearing         = Bearing;
InitialParameter.ComponentSwitch = ComponentSwitch;


end