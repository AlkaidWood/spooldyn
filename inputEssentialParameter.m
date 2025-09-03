%% inputEssentialParameter - Input essential parameters for rotor system
%
% This function initializes and returns the essential parameters for 
% modeling a rotor system, including shaft configurations, disk 
% properties, system status, and component switches. The parameters are 
% organized in a structured format for dynamic analysis.
%
%% Syntax
%  InitialParameter = inputEssentialParameter()
%
%% Description
% |inputEssentialParameter()| initializes parameters for shafts, disks, 
% bearings, system running status, and component configuration switches. 
% These parameters are essential for building the mathematical model of a 
% rotor system in subsequent dynamic analysis. All vectors must be in column format.
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
% * eccentricityPhase    - the phase of Mass eccentricities [rad] (column vector)
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
%   InitialParams = inputEssentialParameter();
%   % Access shaft parameters:
%   shaftLengths = InitialParams.Shaft.totalLength;
%
%% See Also
%  checkInputData, calculateStatus
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.


%%
function InitialParameter = inputEssentialParameter()

% typing the parameter about shaft
Shaft.amount            = [];
Shaft.totalLength       = []; % all vectors in column (m)
Shaft.dofOfEachNodes    = [];
Shaft.outerRadius       = []; % m
Shaft.innerRadius       = []; % m
Shaft.density           = []; % kg/m^3
Shaft.elasticModulus    = []; % Pa
Shaft.poissonRatio      = [];
checkInputData(Shaft)
Shaft.rayleighDamping   = []; % [alpha, beta] CShaft = alpha*(MShaft+MDisk) + beta*KShaft

%%

% typing the parameter about running status
Status.ratio            = []; % [v-shaft2/v-shaft1; v-shaft3/v-shaft1]
Status.vmax             = []; % rad/s, the maximum rotational speed for shaft 1
Status.acceleration     = []; % rad/s^2, acceleration of shaft 1
Status.duration         = []; % s, the duration of shaft 1 in vmax
Status.isDeceleration   = []; % boolean, add a deceleration in status
Status.vmin             = []; % rad/s, the minimum speed afterdeceleration

% (otherwise) you can define your own simulation status function
% define your own function in calculateStatus() where the single time point
% is input, the output must be [acceleration, speed, angular] corresponding
% to input time "tn"
Status.isUseCustomize   = [];
Status.customize        = @(tn) calculateStatus(tn);

% check input
if  length(Status.ratio) >= Shaft.amount
    error('too much input parameter in Status.ratio')
end

%%

% typing the parameter about disk
Disk.amount             = [];
Disk.inShaftNo          = []; % disks in the i-th shaft
Disk.dofOfEachNodes     = [];
Disk.innerRadius        = []; % m
Disk.outerRadius        = []; % m
Disk.thickness          = []; % m
Disk.positionOnShaftDistance = []; %from left end (m)
Disk.density            = []; % kg/m^3
Disk.eccentricity       = []; % m
Disk.eccentricityPhase  = []; % rad/s

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

Bearing.amount          = [];
Bearing.inShaftNo       = [];
Bearing.dofOfEachNodes  = []; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = [];
Bearing.stiffness       = []; % N*m
Bearing.stiffnessVertical = []; % N*m
Bearing.damping         = []; % N*s/m
Bearing.dampingVertical = []; % N*s/m
Bearing.mass            = []; % kg
Bearing.isHertzian      = zeros(Bearing.amount, 1); % should not be revised

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