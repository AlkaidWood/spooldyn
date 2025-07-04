%% inputEssentialParameterSingle2 - Input essential parameters for a simple rotor system
%
% This function initializes and returns the essential parameters for 
% modeling a rotor system, including shaft configurations, disk 
% properties, bearing characteristics, system status, and component 
% switches. The parameters are organized in a structured format for 
% dynamic analysis.
%
%% Syntax
%  InitialParameter = inputEssentialParameterSingle2()
%
%% Description
% |inputEssentialParameterSingle2()| initializes parameters for shafts, disks, 
% bearings, system running status, and component configuration switches. 
% These parameters are essential for building the mathematical model of a 
% rotor system in subsequent dynamic analysis. All vectors should be in column format.
%
%% Output Parameters
% * InitialParameter - Structure containing all system parameters with fields:
%   * Status       - System running status parameters (acceleration, speed profile)
%   * Shaft        - Geometric and material properties of shafts
%   * Disk         - Geometric and inertial properties of disks
%   * Bearing      - Stiffness and damping properties of bearings
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
% * positionOnShaftDistance - Mounting positions from shaft ends [m] (column vector)
% * density              - Material densities [kg/m³] (column vector)
% * eccentricity         - Mass eccentricities [m] (column vector)
%
%% Bearing Parameters (Bearing structure)
% * amount                   - Number of bearings (scalar)
% * inShaftNo                - Shaft index for each bearing (column vector)
% * dofOfEachNodes           - DOF per bearing node (column vector)
% * positionOnShaftDistance  - Mounting positions from shaft ends [m] (column vector)
% * stiffness                - Horizontal stiffness coefficients [N/m] (matrix)
% * stiffnessVertical        - Vertical stiffness coefficients [N/m] (matrix)
% * damping                  - Horizontal damping coefficients [N·s/m] (matrix)
% * dampingVertical          - Vertical damping coefficients [N·s/m] (matrix)
% * mass                     - Bearing masses [kg] (column vector)
% * isHertzian               - Hertzian contact flag (column vector)
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
% * hasGravity               - Enable gravitational effects (logical)
% * hasIntermediateBearing   - Enable intermediate bearings (logical)
% * hasLoosingBearing        - Enable bearing clearance (logical)
% * hasRubImpact             - Enable rotor-stator rub (logical)
% * hasCouplingMisalignment  - Enable coupling misalignment (logical)
% * hasHertzianForce         - Enable Hertzian contact forces (logical)
% * hasCustom                - Enable custom components (logical)
%
%% Example
%   InitialParams = inputEssentialParameterSingle2();
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
function InitialParameter = inputEssentialParameterSingle2()

% typing the parameter about shaft
Shaft.amount            = 1;
Shaft.totalLength       = [500]*10^-3; % all vectors in column (m)
Shaft.dofOfEachNodes    = 4 * ones(Shaft.amount,1);
Shaft.outerRadius       = [5]*10^-3; % m
Shaft.innerRadius       = [0]*10^-3; % m
Shaft.density           = 7850 * ones(Shaft.amount,1); % kg/m^3
Shaft.elasticModulus    = 207e9 * ones(Shaft.amount,1); % Pa
Shaft.poissonRatio      = 0.3 * ones(Shaft.amount,1);
checkInputData(Shaft)
Shaft.rayleighDamping   = [0, 1e-4]; % [alpha, beta] CShaft = alpha*(MShaft+MDisk) + beta*KShaft

%%

% typing the parameter about running status
Status.ratio            = []; % [v-shaft2/v-shaft1; v-shaft3/v-shaft1]
Status.vmax             = 5000 / 60 * (2*pi); % rad/s, the maximum rotational speed for shaft 1
Status.acceleration     = 50; % rad/s^2, acceleration of shaft 1
Status.duration         = 10; % s, the duration of shaft 1 in vmax
Status.isDeceleration   = false; % boolean, add a deceleration in status
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
Disk.amount             = 2; % [propeller, motor]
Disk.inShaftNo          = [1, 1]'; % disks in the i-th shaft
Disk.dofOfEachNodes     = 4 * ones(Disk.amount,1);
Disk.innerRadius        = [5, 5]' * 10^-3; % m
Disk.outerRadius        = [50, 50]' * 10^-3; % m 
Disk.thickness          = [25, 25]' * 10^-3; % m
Disk.positionOnShaftDistance = [200, 300]' *10^-3; %from left end (m)
Disk.density            = 7850*ones(Disk.amount, 1); % kg/m^3
Disk.eccentricity       = [0.2, 0.2]' * 1e-3; % m

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

Bearing.amount          = 2;
Bearing.inShaftNo       = [1, 1]';
Bearing.dofOfEachNodes  = [0, 0]'; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = [0, 500]' * 10^-3; % m
Bearing.stiffness       = 1e8 * ones(Bearing.amount, 1); % N*m
Bearing.stiffnessVertical = 1e8 * ones(Bearing.amount, 1); % N*m
Bearing.damping         = 100 * ones(Bearing.amount, 1); % N*s/m
Bearing.dampingVertical = 100 * ones(Bearing.amount, 1); % N*s/m
Bearing.mass            = [0, 0]'; % kg
Bearing.isHertzian      = zeros(Bearing.amount, 1);

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