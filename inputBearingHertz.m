%% inputBearingHertz - Configure bearing parameters with Hertzian contact
%
% This function configures bearing parameters including Hertzian contact
% modeling between rollers and races for rotor dynamics analysis.
%
%% Syntax
%  Parameter = inputBearingHertz(InitialParameter)
%
%% Description
% |inputBearingHertz| configures bearing parameters with Hertzian contact 
% modeling for rotor systems. It supports four connection configurations:
% 1. Hertzian contact without intermediate masses
% 2. Linear spring-damper without masses
% 3. Mass-spring chain connections
% 4. Hertzian contact with intermediate masses
%
% * Inputs:
%   * |InitialParameter| - Preconfigured system parameters structure
%
% * Outputs:
%   * |Parameter| - Updated parameter structure with bearing configuration
%
%% Bearing Parameters (Bearing structure)
% * amount          - Number of bearings (scalar)
% * inShaftNo       - Shaft index for each bearing (column vector)
% * dofOfEachNodes  - DOF per node [n×3 matrix] (0 if no mass, 2 for x/y directions)
% * positionOnShaftDistance - Mounting positions from shaft ends [m] (column vector)
% * isHertzian      - Hertzian contact activation flags (logical column)
% * stiffness       - Horizontal stiffness [N/m] (matrix)
% * stiffnessVertical - Vertical stiffness [N/m] (matrix)
% * damping         - Horizontal damping [Ns/m] (matrix)
% * dampingVertical - Vertical damping [Ns/m] (matrix)
% * mass            - Bearing masses [kg] (matrix)
% * rollerNum       - Number of rolling elements (column vector)
% * radiusInnerRace - Inner race radii [m] (column vector)
% * radiusOuterRace - Outer race radii [m] (column vector)
% * clearance       - Bearing clearances [m] (column vector)
% * contactStiffness - Hertzian stiffness [N/m^1.5] (column vector)
% * coefficient     - Contact force exponent (column vector)
%
%% Model Configuration Rules
% 1. Hertzian contact without mass:
%    shaft -- Hertz+k1c1 -- basement
% 2. Linear spring-damper without mass:
%    shaft -- k1c1 -- basement
% 3. Mass-spring chain:
%    shaft -- k1c1 -- m1 -- k2c2 -- m2 -- ... -- kncn -- basement
% 4. Hertzian with intermediate masses:
%    shaft -- Hertz+k1c1 -- m1 -- k2c2 -- ... -- kncn -- basement
%
%% Automatic Processing
% * Parameters are automatically sorted by shaft index and position
% * If any bearing has |isHertzian=true|, enables |hasHertzianForce| flag
%
%% Example
%   % Initialize system parameters
%   sysParams = inputEssentialParameterBO();
%   % Configure bearings with Hertzian contact
%   sysParams = inputBearingHertz(sysParams);
%
%% See Also
%  checkInputData, sortRowsWithShaftDis, inputEssentialParameterBO
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function Parameter = inputBearingHertz(InitialParameter)
% typing the parameters about bearing considering the Hertz contact between
% rollers and races
Bearing.amount          = [];
Bearing.inShaftNo       = [];
Bearing.dofOfEachNodes  = []; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = []; % from the left end of the shaft (m)
Bearing.isHertzian      = []; % boolean
% M K C, elements in the same row: the MKC at the same position of the
% shaft; mass(1,1) -> mass(1,n):
% the mass near the shaft the bearing connecting ->
% the mass near the basement of bearing.
%
% If isHertizian and no mass, the corresponding k c will be added in
% global matrix normally; the model:
% shaft--Hertz+k1c1--basement;
% If is no Hertzian and with mass: there are n mass in a row, and n+1 k c 
% for a bearing; the model will be established as:
% shaft--k1c1--m1--k2c2--m2--k3c3--m3--k4c4- ...-mn--k(n+1)c(n+1)--basement;
% If is no Hertzian and no mass, the k c will be added in global 
% matrix normally; the model:
% shaft--k1c1--basement;
% If isHertzian and with mass, the hertzian force will be added at the mass
% in the first column (near the shaft); the model:
% shaft--Hertz+k1c1--m1--k2c2--m2--k3c3--m3--k4c4- ...-mn--k(n+1)c(n+1)--basement;

Bearing.stiffness       =  []; % N*m
Bearing.stiffnessVertical = []; % N*m
Bearing.damping         =  []; % N*s/m
Bearing.dampingVertical =  []; % N*s/m
% the first n mass in each row must be non-zero, n is the number of mass
% of bearings
Bearing.mass            =  []; % kg
Bearing.rollerNum = [];
Bearing.radiusInnerRace = []; % m
Bearing.radiusOuterRace = []; % m
Bearing.clearance = []; % m
Bearing.contactStiffness = []; % N*m^-3/2
Bearing.coefficient = []; % =3/2 in a ball bearing; = 10/9 in a roller bearing

% check input data
checkInputData(Bearing)
% order the column in struct with shaft no and distance on the shaft
Bearing = sortRowsWithShaftDis(Bearing);

% Outpt initialParameter
Parameter = InitialParameter;
Parameter.Bearing = Bearing;
if sum(Bearing.isHertzian)~=0
    Parameter.ComponentSwitch.hasHertzianForce = true;
end % end if
end