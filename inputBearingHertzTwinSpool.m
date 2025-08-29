%% inputBearingHertzTwinSpool - Configure bearing parameters with Hertzian contact
%
% This function configures bearing parameters considering Hertzian contact 
% between rolling elements and races for rotor dynamics analysis.
%
%% Syntax
%  Parameter = inputBearingHertzTwinSpool(InitialParameter)
%
%% Description
% |inputBearingHertzTwinSpool| configures bearing parameters with Hertzian contact 
% characteristics and integrates them into the initial system parameter structure.
%
% * Inputs:
%   * |InitialParameter| - Preconfigured system parameters structure created by |inputEssentialParameter|
%
% * Outputs:
%   * |Parameter| - Updated parameter structure with bearing configuration
%
%% Bearing Parameters (Bearing structure)
% * amount              - Total number of bearings (scalar)
% * inShaftNo           - Shaft index for each bearing (column vector)
% * dofOfEachNodes      - Degrees of freedom per node (all zeros) (column vector)
% * positionOnShaftDistance - Bearing positions from shaft left end [m] (column)
% * isHertzian          - Hertzian contact activation flags (logical column)
% * stiffness           - Horizontal stiffness values [N/m] (column vector)
% * stiffnessVertical   - Vertical stiffness values [N/m] (column vector)
% * damping             - Horizontal damping coefficients [Ns/m] (column vector)
% * dampingVertical     - Vertical damping coefficients [Ns/m] (column vector)
% * mass                - Bearing masses [kg] (column vector)
% * rollerNum           - Number of rolling elements per bearing (column vector)
% * radiusInnerRace     - Inner race radii [m] (column vector)
% * radiusOuterRace     - Outer race radii [m] (column vector)
% * clearance           - Bearing clearances [m] (column vector)
% * contactStiffness    - Hertzian contact stiffness [N/m^1.5] (column vector)
% * coefficient         - Contact force exponent (3/2=ball, 10/9=roller) (column)
%
%% Model Configuration Notes
% * Hertzian Contact Model:
%   * |isHertzian=true|: Shaft connects through nonlinear Hertzian stiffness
%   * |isHertzian=false|: Linear spring-damper connection
% * Multi-Mass Bearing Models:
%   * Configure intermediate masses with corresponding stiffness/damping
%   * First mass in chain receives Hertzian forces when enabled
%
%% Component Activation
% Automatically enables |hasHertzianForce| flag in |ComponentSwitch| when 
% any bearing has |isHertzian=true|.
%
%% Example
%   % Initialize system parameters
%   InitialParam = inputEssentialParameterTwinSpool();
%   % Add bearing configuration
%   FullParam = inputBearingHertzTwinSpool(InitialParam);
%   % Access bearing stiffness
%   k_bearing3 = FullParam.Bearing.stiffness(3);
%
%% See Also
%  checkInputData, sortRowsWithShaftDis, main_contactStiffness
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function Parameter = inputBearingHertzTwinSpool(InitialParameter)
% typing the parameters about bearing considering the Hertz contact between
% rollers and races
Bearing.amount          = 6;
Bearing.inShaftNo       = [1; 1; 2; 1; 1; 1];
Bearing.dofOfEachNodes  = [0; 0; 0; 0; 0; 0]; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = 1e-3 * [214; 778; 28.5; 0; 80; 962]; % from the left end of the shaft (m)
Bearing.isHertzian      = [false; false; false; false; false; false]; % boolean: true-> activate hertzian force
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

Bearing.stiffness       =  [1e8; 1e8; 1e8; 1e7; 1e7; 1e7]; % N*m
Bearing.stiffnessVertical = [1e8; 1e8; 1e8; 1e7; 1e7; 1e7]; % N*m
Bearing.damping         =  [500; 500; 500; 10; 10; 10]; % N*s/m
Bearing.dampingVertical =  [500; 500; 500; 10; 10; 10]; % N*s/m
% the first n mass in each row must be non-zero, n is the number of mass
% of bearings
Bearing.mass            =  [0; 0; 0; 0; 0; 0]; % kg
Bearing.rollerNum = [0; 0; 0; 0; 0; 0];
Bearing.radiusInnerRace = [0; 0; 0; 0; 0; 0]*1e-3; % m
Bearing.radiusOuterRace = [0; 0; 0; 0; 0; 0]*1e-3; % m
Bearing.clearance = [0; 0; 0; 0; 0; 0]*1e-6; % m
Bearing.contactStiffness = [0; 0; 0; 0; 0; 0]; % N*m^-3/2
Bearing.coefficient = [0; 0; 0; 0; 0; 0]; % =3/2 in a ball bearing; = 10/9 in a roller bearing

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