%% inputEssentialParameterTwinSpool - Input essential parameters for a dual-rotor system
%
% This function initializes and returns the essential parameters for 
% modeling a rotor system, including shaft configurations, disk 
% properties, system status, and component switches. The parameters are 
% organized in a structured format for dynamic analysis.
%
%% Syntax
%  InitialParameter = inputEssentialParameterTwinSpool()
%
%% Description
% |inputEssentialParameterTwinSpool()| initializes parameters for shafts, disks, 
% system running status, and component configuration switches. These 
% parameters are essential for building the mathematical model of a 
% rotor system in subsequent dynamic analysis.
%
%% Output Parameters
% * InitialParameter - Structure containing all system parameters with fields:
%   * Status       - System running status parameters (acceleration, speed profile)
%   * Shaft        - Geometric and material properties of shafts
%   * Disk         - Geometric and inertial properties of disks
%   * ComponentSwitch - Boolean flags for system component activation
%
%% Shaft Parameters (Shaft structure)
% * amount           - Number of shafts (scalar)
% * totalLength      - Column vector of shaft lengths [m]
% * dofOfEachNodes   - Degrees of freedom per node (column vector)
% * outerRadius      - Outer radii of shafts [m] (column vector)
% * innerRadius      - Inner radii of shafts [m] (column vector)
% * density          - Material densities [kg/m³] (column vector)
% * elasticModulus   - Elastic moduli [Pa] (column vector)
% * poissonRatio     - Poisson's ratios (column vector)
% * rayleighDamping  - Rayleigh damping coefficients [alpha, beta]
%
%% Disk Parameters (Disk structure)
% * amount               - Number of disks (scalar)
% * inShaftNo            - Shaft index for each disk (column vector)
% * dofOfEachNodes       - DOF per node (column vector)
% * innerRadius          - Disk inner radii [m] (column vector)
% * outerRadius          - Disk outer radii [m] (column vector)
% * thickness            - Disk thicknesses [m] (column vector)
% * positionOnShaftDistance - Mounting positions from shaft ends [m] (column)
% * density              - Material densities [kg/m³] (column vector)
% * eccentricity         - Mass eccentricities [m] (column vector)
% * eccentricityPhase    - the phase of Mass eccentricities [rad] (column vector)
%
%% Status Parameters (Status structure)
% * ratio            - Speed ratio between shafts (vector)
% * vmax             - Maximum rotational speed of shaft 1 [rad/s]
% * acceleration     - Rotational acceleration of shaft 1 [rad/s²]
% * duration         - Duration at maximum speed [s]
% * isDeceleration   - Flag for deceleration phase (logical)
% * vmin             - Minimum speed after deceleration [rad/s]
% * isUseCustomize   - Flag for custom speed profile (logical)
% * customize        - Handle to custom speed profile function
%
%% Component Switches (ComponentSwitch structure)
% * hasGravity              - Enable gravitational effects (logical)
% * hasIntermediateBearing - Enable intermediate bearings (logical)
% * hasLoosingBearing      - Enable bearing clearance (logical)
% * hasRubImpact            - Enable rotor-stator rub (logical)
% * hasCouplingMisalignment - Enable coupling misalignment (logical)
% * hasHertzianForce        - Enable Hertzian contact forces (logical)
%
%% Example
%   InitialParams = inputEssentialParameterTwinSpool();
%   % Access shaft parameters:
%   shaftLengths = InitialParams.Shaft.totalLength;
%
%% See Also
%  checkInputData, calculateStatus
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


%%
function InitialParameter = inputEssentialParameterTwinSpool()

% typing the parameter about shaft
Shaft.amount            = 2;
Shaft.totalLength       = [962; 382]*10^-3; % all vectors in column (m)
Shaft.dofOfEachNodes    = 4 * ones(Shaft.amount,1);
Shaft.outerRadius       = [10; 32.5]*10^-3; % m
Shaft.innerRadius       = [0; 20]*10^-3; % m
Shaft.density           = 7850 * ones(Shaft.amount,1); % kg/m^3
Shaft.elasticModulus    = 210e9 * ones(Shaft.amount,1); % Pa
Shaft.poissonRatio      = 0.296 * ones(Shaft.amount,1);
checkInputData(Shaft)
Shaft.rayleighDamping   = [0, 3e-4]; % [alpha, beta] CShaft = alpha*(MShaft+MDisk) + beta*KShaft

%%

% typing the parameter about running status
Status.ratio            = [1.3]; % [v-shaft2/v-shaft1; v-shaft3/v-shaft1]
Status.vmax             = 200; % rad/s, the maximum rotational speed for shaft 1
Status.acceleration     = 20; % rad/s^2, acceleration of shaft 1
Status.duration         = 0; % s, the duration of shaft 1 in vmax
Status.isDeceleration   = true; % boolean, add a deceleration in status
Status.vmin             = 0; % s, the minimum speed afterdeceleration

% (otherwise) you can define your own simulation status function
% define your own function in calculateStatus() where the single time point
% is input, the output must be [acceleration, speed, angular] corresponding
% to input time "tn", in each output the dimension is m*X, X is the number
% of the shaft, m is number of elements in tn
Status.isUseCustomize   = false;
Status.customize        = @(tn) calculateStatus(tn);

% check input
if  length(Status.ratio) >= Shaft.amount
    error('too much input parameter in Status.ratio')
end

%%

% typing the parameter about disk
Disk.amount             = 4;
Disk.inShaftNo          = [1, 1, 2, 2]'; % disks in the i-th shaft
Disk.dofOfEachNodes     = 4 * ones(Disk.amount,1);
Disk.innerRadius        = [10, 10, 32.5, 32.5]' *10^-3; % m
Disk.outerRadius        = [125, 125, 125, 125]' *10^-3; % m
Disk.thickness          = [0.015*ones(1,4)]'; % m
Disk.positionOnShaftDistance = [270.5, 718.5, 150.5, 292.5]' * 10^-3; %from left end (m)
Disk.density            = [7850*ones(1,4)]'; % kg/m^3
Disk.eccentricity       = [0.0979e-3*ones(1,4)]'; % m
Disk.eccentricityPhase  = zeros(Disk.amount, 1); % rad/s

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
% If you choose to input the bearing parameter here, you should not use the
% inputBearingHertz()
% model: shaft--k1c1--mass--k2c2--basement
% Bearing.amount          = 3;
% Bearing.inShaftNo       = [1; 1; 2];
% Bearing.dofOfEachNodes  = [2; 2; 2]; % if mass=0, dof must be 0 
% Bearing.positionOnShaftDistance = 1e-3 * [176.5; 718.5; 343.5];
% % stiffness = [bearing1_k1, bearing1_k2; bearing2_k1, bearing2_k2]
% Bearing.stiffness       = [1e8, 1e8; 1e8, 1e8; 1e8, 1e8]; % N*m
% % damping = [bearing1_c1, bearing1_c2; bearing2_c1, bearing2_c2]
% Bearing.damping         = [300, 300; 300, 300; 300, 300]; % N*s/m
% Bearing.mass            = [3; 3; 3]; % kg

Bearing.amount          = 0;
Bearing.inShaftNo       = [];
Bearing.dofOfEachNodes  = []; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = [];
Bearing.stiffness       = []; % N*m
Bearing.damping         = []; % N*s/m
Bearing.mass            = []; % kg

checkInputData(Bearing)

%%

% ComponentSwitch will be changed by corresponding input..() function
ComponentSwitch.hasGravity = true; % boolean, true-> take gravity into account; false->no gravity in the dynamic equation
ComponentSwitch.hasIntermediateBearing = false;
ComponentSwitch.hasLoosingBearing = false;
ComponentSwitch.hasRubImpact = false;
ComponentSwitch.hasCouplingMisalignment = false;
ComponentSwitch.hasHertzianForce = false;
ComponentSwitch.hasCustom = false;


%%

% Output initialParameter without optional parameter
InitialParameter.Status          = Status;
InitialParameter.Shaft           = Shaft;
InitialParameter.Disk            = Disk;
InitialParameter.Bearing         = Bearing;
InitialParameter.ComponentSwitch = ComponentSwitch;


end