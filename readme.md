# Multispool Rotordynamics Open Source Software (SpoolDyn)
SpoolDyn is an open-source MATLAB tool designed for the automated generation, simulation, and analysis of rotordynamic models for multispool systems (including single-spool systems such as Jeffcott rotors).

The software is developed using the Finite Element Method, Timoshenko Beam Theory, and lumped-parameter modeling. It includes four basic elements—shaft, disk, bearing, and inter-shaft bearing—which use positional, physical, and geometric parameters as inputs. Users can also customize external forces and rotational conditions. The programming style follows a procedural approach.


# Example 1: Two-disk Rotor
```
% all input parameters are saved in this function file
InitialParameter = inputEssentialParameterSingle2();

% this function will create the global matrices in workspace (check <Parameter>), model and mesh diagram in folders: <modelDiagram>, <meshDiagram>
Parameter = establishModel(InitialParameter); 

% function to generate the dynamic equations function file <dynamicEquation.m> in root folder
generateDynamicEquation(Parameter); 

% calculate response (set simulation time from 0 to 50 seconds, sampling frequency 2^14)
[q, dq, t] = calculateResponse(Parameter, [0,50], 2^14, calculateMethod='ode15s'); 

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

signalProcessing(q, dq, t, Parameter, [0,50], 2^14, SwitchFigure)
```

![example1-mesh](readme/example1/MeshResultOfShaft1.png)

![example1-model](readme/example1/theWholeModel.png)


# Example 2: Twin-spool Rotor
```
% all shaft, disk, operation parameters are saved in this function file
InitialParameter = inputEssentialParameterTwinSpool();

% all bearing parameters are saved in this file (But Hertzian force is not used in this example. You can control in this file)
InitialParameter = inputBearingHertzTwinSpool(InitialParameter);

% all inter-shft bearing parameters are saved in this file
InitialParameter = inputIntermediateBearingTwinSpool(InitialParameter); 

 % this function will create the global matrices in workspace (check Parameter), model and mesh diagram in folders: <modelDiagram>, <meshDiagram>
Parameter = establishModel(InitialParameter);

% function to generate the dynamic equations function file <dynamicEquation.m> in root folder
generateDynamicEquation(Parameter);

% calculate response (set simulation time from 0 to 10 seconds, sampling frequency 2^14)
[q, dq, t] = calculateResponse(Parameter, [0,10], 2^14, calculateMethod='ode15s');

% signal post-processing
SwitchFigure.displacement       = true; % true-> output time history in <signalProcess> folder
SwitchFigure.axisTrajectory     = false;
SwitchFigure.axisTrajectory3d   = false;
SwitchFigure.phase              = false;
SwitchFigure.fftSteady          = false;
SwitchFigure.fftTransient       = false;
SwitchFigure.poincare           = false;
SwitchFigure.poincare_phase     = false;
SwitchFigure.saveFig            = false;
SwitchFigure.saveEps            = false;

signalProcessing(q, dq, t, Parameter, [0,10], 2^14, SwitchFigure)
```

![example2-mesh1](readme/example2/MeshResultOfShaft1.png)
![example2-mesh1](readme/example2/MeshResultOfShaft2.png)

Model diagram of shaft 1
![example2-model](readme/example2/diagramOfShaft1.png)
Model diagram of shaft 2
![example2-model](readme/example2/diagramOfShaft2.png)
Model diagram of full rotor
![example2-model](readme/example2/theWholeModel.png)

# Installation
## Install MATLAB
The first step is to install MATLAB **(MATLAB Version>=2025a)**.

## Install SpoolDyn
### Method 1: Install via Git (Recommended)

1. **Install Git**  
   If you don't have Git installed:  
   - Windows: Download from [git-scm.com](https://git-scm.com/download/win)  
   - Linux: Use your package manager (e.g., `sudo apt install git` for Ubuntu/Debian)  
   - macOS: Use Homebrew (`brew install git`) or download from [git-scm.com](https://git-scm.com/download/mac)

2. **Clone the repository**  
   Open a terminal and run:
   ```
   git clone https://github.com/AlkaidWood/spooldyn.git
   cd spooldyn
   ```
3. **You already installed SpoolDyn in the folder spooldyn**

### Method 2: Install via ZIP Download

1. **Download the ZIP**  
Go to the repository page:  
[https://github.com/AlkaidWood/spooldyn](https://github.com/AlkaidWood/spooldyn)  
Click the green "Code" button → "Download ZIP".

2. **Extract the files**  
Unzip the downloaded file to your preferred directory.
3. **You already installed SpoolDyn in your preferred directory**


# User Guide

A typical workflow in SpoolDyn is: 
* Input Module -> Modeling Module -> Dynamic equation generation Module -> Equation-solving Module -> Post-processing Module

In the Input Module, four function files store all input parameters.

The remaining modules each correspond to one interface function, resulting in four additional functions.

By calling these eight functions within the MATLAB `main` file, users can complete the simulation.

## Input Module
### Overview
All input parameters are contained within several MATLAB function files (.m). These functions validate the parameters and reorganize them into a struct data type for use by the modeling module.

In the `main` file:
```
InitialParameter = inputEssentialParameter();
InitialParameter = inputBearingHertz(InitialParameter);
InitialParameter = inputEssentialParameter(InitialParameter);
InitialParameter = inputCustomFunction(InitialParameter);

...(Modeling Module)
```

There are four types of input files: `inputEssentialParameter()`, `inputBearingHertz()`, `inputIntermediateBearing()`, `inputCustomFunction()`.
* **Mandatory**
    * `inputEssentialParameter()`: Contains parameters for shafts, disks, bearings, and operational settings.
* **Optional**
    * `inputBearingHertz()`: Contains parameters for bearings and their corresponding Hertzian contact properties.
    * `inputIntermediateBearing()`: Contains parameters for inter-shaft bearings and their corresponding Hertzian contact properties.
    * `inputCustomFunction()`: Allows users to define custom external forces and specify additional node positions.

Note 1: Bearing parameters should be defined *either* in `inputBearingHertz()` **or** `inputEssentialParameter()`, but not both.

Note 2: The use of these optional functions depends on the specific requirements of the user's simulation.

Note 3: Vectors must be defined as column vectors.


### Shafts
Within the `inputEssentialParameter()` function, for a rotor system consisting of a single shaft:
```
Shaft.amount            = 1;
Shaft.totalLength       = 517.2*10^-3;
Shaft.dofOfEachNodes    = 4; % The current version only supports a value of 4 for this parameter. Two lateral translational degrees of freedom and two rotational degrees of freedom.
Shaft.outerRadius       = 5*10^-3; % m
Shaft.innerRadius       = 0; % m
Shaft.density           = 7850; % kg/m^3
Shaft.elasticModulus    = 207e9; % Pa
Shaft.poissonRatio      = 0.3;
checkInputData(Shaft) % Verify parameter correctness
Shaft.rayleighDamping   = [0, 1.9e-4];
```
Note 1: The specified Rayleigh damping applies only to rotational components. For example, `Shaft.rayleighDamping = [alpha, beta]`: CShaft = alpha*(MShaft+MDisk) + beta*KShaft

Note 2: The shear modulus is automatically calculated based on the provided Poisson's ratio and elastic modulus.

Within the `inputEssentialParameter()` function, for a rotor system consisting of two shafts:
```
Shaft.amount            = 2;
Shaft.totalLength       = [962; 382]*10^-3; % all vectors in column (m)
Shaft.dofOfEachNodes    = [4; 4];
Shaft.outerRadius       = [10; 32.5]*10^-3; % m
Shaft.innerRadius       = [0; 20]*10^-3; % m
Shaft.density           = [7850; 7850]; % kg/m^3
Shaft.elasticModulus    = [210e9; 210e9]; % Pa
Shaft.poissonRatio      = [0.296; 0.296];
checkInputData(Shaft)
Shaft.rayleighDamping   = [0, 3e-4];
```
Note 1: In multi-shaft configurations, all parameters for shaft 1 are stored in the first row of each array, while parameters for shaft 2 are stored in the second row.

Note 2: The software supports rotor systems with any number of shafts (no upper limit).


### Disks
Within the `inputEssentialParameter()` function, for a rotor system consisting of four disks:
```
Disk.amount             = 4;
Disk.inShaftNo          = [1, 1, 2, 2]'; % disks in the i-th shaft
Disk.dofOfEachNodes     = 4 * ones(Disk.amount,1); % corresponding to `Shaft.dofOfEachNodes`
Disk.innerRadius        = [10, 10, 32.5, 32.5]' *10^-3; % m
Disk.outerRadius        = [125, 125, 125, 125]' *10^-3; % m
Disk.thickness          = [0.015*ones(1,4)]'; % m
Disk.positionOnShaftDistance = [270.5, 718.5, 150.5, 292.5]' * 10^-3; %from left end (m)
Disk.density            = [7850*ones(1,4)]'; % kg/m^3
Disk.eccentricity       = [0.0979e-3*ones(1,4)]'; % m
Disk.eccentricityPhase  = zeros(Disk.amount, 1); % rad/s
```
Note 1: `Disk.inShaftNo` specifies the shaft number on which the disk is mounted. The shaft numbering corresponds to the order defined in the shaft parameters section.

Note 2: `Disk.positionOnShaftDistance` defines the axial position of the disk along the shaft, measured from the shaft's left end.

Note 3: `Disk.eccentricity` indicates the offset between the geometric center and the center of mass of the disk, expressed in meters.


### Bearings
Users can choose between `inputEssentialParameter()` and `inputBearingHertz()` to input bearing parameters.

If the user just wants to use the linear bearing element, the bearing parameters can be inputted into `inputEssentialParameter()`, and then inputs code below in the `main` file:
```
InitialParameter = inputEssentialParameter();

Parameter = establishModel(InitialParameter);

...
```
If the user wants to use the linear bearing element with Hertzian contact, the bearing parameters can be inputted into `inputBearingHertz()`, and then input code below in the `main` file::
```
InitialParameter = inputEssentialParameter();
InitialParameter = inputBearingHertz(InitialParameter);

Parameter = establishModel(InitialParameter);

...
```

**Case 1:** **Massless linear bearings** (in `inputEssentialParameter()`):
```
Bearing.amount          = 2;
Bearing.inShaftNo       = [1; 1];
Bearing.dofOfEachNodes  = [0; 0]; % if mass=0, dof must be 0 
Bearing.positionOnShaftDistance = [0; 517.2] * 10^-3; % m
Bearing.stiffness       = [1e7;  2e7]; % N*m
Bearing.stiffnessVertical = [1e7;  2e7]; % N*m
Bearing.damping         = [1e4; 2e4]; % N*s/m
Bearing.dampingVertical = [1e4; 2e4]; % N*s/m
Bearing.mass            = [0; 0]; % kg
Bearing.isHertzian      = [0; 0];
```
Note 1: If a bearing element does not contain any mass point, the `dofOfEachNodes` should be zero.

Note 2: The parameters `inShaftNo` and `positionOnShaftDistance` in a bearing element are similar to those in a disk element.

Note 3: `stiffness` denotes the stiffness in the horizontal direction; `stiffnessVertical` denotes the stiffness in the vertical direction.

Note 4: The defined massless bearing elements in a single direction are shown below.

<img src="readme/bearing_model/massless-bearing.png" width="100%" height="" />

**Case 2:** **Mass linear bearings with Hertzian contact** (in `inputBearingHertz()`):
```
Bearing.amount          = 2;
Bearing.inShaftNo       = [1; 1];
Bearing.dofOfEachNodes  = [2; 2]; % The current version only supports a value of 0 or 2 for this parameter. two lateral translational degrees of freedom for one mass point.
Bearing.positionOnShaftDistance = [0; 517.2] * 10^-3; % m
Bearing.stiffness       = [1e7, 1e8;  2e7, 2e8]; % N*m
Bearing.stiffnessVertical = [1e7, 1e8;  2e7, 2e8]; % N*m
Bearing.damping         = [1e4, 1e5; 2e4, 2e5]; % N*s/m
Bearing.dampingVertical = [1e4, 1e5; 2e4, 2e5]; % N*s/m
Bearing.mass            = [1; 1]; % kg
Bearing.isHertzian      = [true; true];
Bearing.rollerNum = [16; 16];
Bearing.radiusInnerRace = [1e-2; 1e-2]; % m
Bearing.radiusOuterRace = [1.5e-2; 1.5e-2]; % m
Bearing.clearance = [1e-7; 1e-7]; % m
Bearing.contactStiffness = [1e8; 1e8]; % N*m^-3/2 or N*m^-10/9
Bearing.coefficient = [3/2; 3/2]; % =3/2 in a ball bearing; = 10/9 in a roller bearing
```
Note 1: If the user adds mass to the bearing element, the `dofOfEachNodes` should be 2. The current version only supports a value of 0 or 2 for this parameter, representing two lateral translational degrees of freedom per mass point.

Note 2: Each row of parameters corresponds to the same bearing element.

Note 3: If the number of mass points is n, the number of stiffness/damping parameters should be n+1.

Note 4: The equation for Hertzian contact can be found in the reference paper.

Note 5: The configured bearing elements with mass in one direction are shown below.

<img src="readme/bearing_model/mass-bearing.png" width="100%" height="" />

**Case 3:** **Mixed bearings** (in `inputBearingHertz()`):
* Bearing 1: massless bearing
* Bearing 2: bearing with one mass block
* Bearing 3: bearing with two mass block
* Bearing 4: bearing with three mass block and Hertzian contact force
```
Bearing.amount          = 4;
Bearing.inShaftNo       = [1; 1; 1; 1];
Bearing.dofOfEachNodes  = [0, 0, 0;...
                           2, 0, 0;...
                           2, 2, 0;...
                           2, 2, 2];
Bearing.positionOnShaftDistance = [0; 100; 200; 300] * 10^-3; % m
Bearing.stiffness         = [1e6, 0,   0,   0;...
                             2e6, 2e7, 0,   0;...
                             3e6, 3e7, 3e8, 0;...
                             4e6, 4e7, 4e8, 4e9]; % N*m
Bearing.stiffnessVertical = [1e6, 0,   0,   0;...
                             2e6, 2e7, 0,   0;...
                             3e6, 3e7, 3e8, 0;...
                             4e6, 4e7, 4e8, 4e9]; % N*s/m
Bearing.damping         = [1e4, 0,   0,   0;...
                           2e4, 2e5, 0,   0;...
                           3e4, 3e5, 3e6, 0;...
                           4e4, 4e5, 4e6, 4e7]; % N*s/m
Bearing.dampingVertical = [1e4, 0,   0,   0;...
                           2e4, 2e5, 0,   0;...
                           3e4, 3e5, 3e6, 0;...
                           4e4, 4e5, 4e6, 4e7]; % N*s/m
Bearing.mass            = [0, 0, 0;...
                           2, 0, 0;...
                           3, 4, 0;...
                           5, 6, 7]; % kg
Bearing.isHertzian      = [false; false; false; true];
Bearing.rollerNum = [0; 0; 0; 16];
Bearing.radiusInnerRace = [0; 0; 0; 1e-2]; % m
Bearing.radiusOuterRace = [0; 0; 0; 1.5e-2]; % m
Bearing.clearance = [0; 0; 0; 1e-7]; % m
Bearing.contactStiffness = [0; 0; 0; 1e8]; % N*m^-3/2 or N*m^-10/9
Bearing.coefficient = [0; 0; 0; 3/2]; % =3/2 in a ball bearing; = 10/9 in a roller bearing
```

***Note 1: The number (No.) of bearings depends on their positions. SpoolDyn will automatically order the bearings and disks according to the components' positions along the shafts.***

Note 2: If bearing elements have different numbers of mass blocks, the dimensions of certain input parameters (such as `dofOfEachNodes`, `stiffness`, `damping`, and `mass`) are set based on the following: the number of rows corresponds to the total number of bearing elements, and the number of columns corresponds to the maximum number of mass blocks in any single bearing element.

Note 3: Each row represents one bearing element. **Use zero-padding to match the specified matrix dimensions after entering the parameters.**

Note 4: The configured mixed bearing elements in one direction are shown below.

<img src="readme/bearing_model/mixed-bearing.png" width="100%" height="" />


### Inter-shaft Bearings
Users can input and save inter-shaft bearing parameters using `inputIntermediateBearing`.

To add an inter-shaft bearing element in the rotor system, input the parameters in `inputIntermediateBearing()`, and then include the following code in the `main` file:
```
InitialParameter = inputEssentialParameter();
InitialParameter = inputIntermediateBearing(InitialParameter);

Parameter = establishModel(InitialParameter);

...
```
Inside `inputIntermediateBearing()`, for a rotor system containing three **inter-shaft bearings with Hertzian contact force**:

```
IntermediateBearing.amount          = 3;
IntermediateBearing.betweenShaftNo  =  [1, 2; 1, 2; 1, 2]; % n*2
IntermediateBearing.dofOfEachNodes =  [2; 2; 2];% if mass=0, dof must be 0. Now, only x and y directions are supported. So, here is 0 or 2
IntermediateBearing.positionOnShaftDistance = [0.1, 0; 0.8, 0.7; 1, 0.9]; % from the left end of the shaft
IntermediateBearing.isHertzian      = [true; true; true]; % boolean
IntermediateBearing.isHertzianTop   = [true; true; true];
IntermediateBearing.stiffness           =  [0, 1e7; 0, 1e7; 0, 1e7]; % N/m, in column, n*1
IntermediateBearing.stiffnessVertical   =  [0, 1e7; 0, 1e7; 0, 1e7]; % N/m, in column, n*1
IntermediateBearing.damping             =  [0, 0; 0, 0; 0, 0]; % N/s^2, in column, n*1
IntermediateBearing.dampingVertical     =  [0, 0; 0, 0; 0, 0]; % N/s^2, in column, n*1
IntermediateBearing.mass                =  [0.3; 0.3; 0.3]; % kg                           
IntermediateBearing.rollerNum        = [15; 15; 15];
IntermediateBearing.radiusInnerRace = [15e-3; 15e-3; 15e-3]; % m
IntermediateBearing.radiusOuterRace = [25e-3; 25e-3; 25e-3]; % m
IntermediateBearing.innerShaftNo = [1; 1; 1]; % indicates Inner shaft No. 
IntermediateBearing.clearance = [5e-6; 5e-6; 5e-6]; % m
IntermediateBearing.contactStiffness = [13.34e9; 13.34e9; 13.34e9]; % N*m^-3/2
IntermediateBearing.coefficient = [3/2; 3/2; 3/2]; % =3/2 in a ball bearing; = 10/9 in a roller bearing
```

Note 1: The parameter `betweenShaftNo` indicates the connection relationship of the inter-shaft bearing. For example, `betweenShaftNo = [1, 2; 2, 3; 1, 3]` means the first inter-shaft bearing connects shafts 1 and 2; the second connects shafts 2 and 3; and the third connects shafts 1 and 3.

Note 2: The parameters `dofOfEachNodes`, `isHertzian`, `stiffness`, `damping`, and `mass` are similar to those in the bearing parameters.

Note 3: The parameter `positionOnShaftDistance` is an n×2 matrix, where n is the number of inter-shaft bearings. The first column represents the distance from the left end of the first shaft to the connection point. The second column represents the distance from the left end of the second shaft to the connection point. The order of these shafts (first and second) corresponds to the order specified in `betweenShaftNo`.

Note 4: The parameter `isHertzianTop` controls where the Hertzian contact force is applied within the mass-spring-damping chain.
<img src="readme/inter_shaft_bearing_model/inter-shaft-bearing.png" width="100%" height="" />



### Operation Status
In the `inputEssentialParameter()` function, users can control the rotational status parameters using two methods:

The first method is to use predefined operating conditions:

```
Status.ratio            = [1.3]; % [v-shaft2/v-shaft1; v-shaft3/v-shaft1]
Status.vmax             = 200; % rad/s, the maximum rotational speed for shaft 1
Status.acceleration     = 20; % rad/s^2, acceleration of shaft 1
Status.duration         = 0; % s, the duration of shaft 1 in vmax
Status.isDeceleration   = true; % boolean, add a deceleration in status
Status.vmin             = 0; % s, the minimum speed afterdeceleration
Status.isUseCustomize   = false;
Status.customize        = @(tn) calculateStatus(tn);
```
By editing `ratio`, `vmax`, `acceleration`, `duration`, `isDeceleration`, and `vmin`, SpoolDyn will load different predefined rotational status functions as shown below.

<img src="readme/rotation_status/rotation_status.png" width="100%" height="auto" />

Note 1: The parameter `ratio` represents [v_shaft2/v_shaft1; v_shaft3/v_shaft1; v_shaft4/v_shaft1;...]. If there is only one shaft, set `ratio=[]`.

Note 2: Parameters `vmax`, `acceleration`, and `vmin` denote the maximum speed, acceleration, and minimum speed of shaft 1, respectively. The rotational speeds of other shafts are calculated by multiplying these values by the speed ratio.

Note 3: If `acceleration = 0`, the rotor will maintain a constant speed.

Note 4: If `isDeceleration = false`, the rotor will remain at the maximum speed once it is reached.

The second method is to define a custom function `calculateStatus(tn)` to control the rotational phase, speed, and acceleration of each shaft:
```
Status.ratio            = [1.3]; % [v-shaft2/v-shaft1; v-shaft3/v-shaft1]
Status.vmax             = 200; % rad/s, the maximum rotational speed for shaft 1
Status.acceleration     = 20; % rad/s^2, acceleration of shaft 1
Status.duration         = 0; % s, the duration of shaft 1 in vmax
Status.isDeceleration   = true; % boolean, add a deceleration in status
Status.vmin             = 0; % s, the minimum speed afterdeceleration
Status.isUseCustomize   = true;
Status.customize        = @(tn) calculateStatus(tn);
```
When `isUseCustomizeis` set to `true`, the other status parameters will be ignored.


### External Force
SpoolDyn provides an interface for adding custom external forces to the rotor system.

If the user wishes to add an external force, the corresponding parameters can be entered in `inputCustomFunction()`, and the following code should be included in the `main` file:
```
InitialParameter = inputEssentialParameter();
InitialParameter = inputCustomFunction(InitialParameter);

Parameter = establishModel(InitialParameter);

...
```
Within `inputCustomFunction()`, users can configure parameters to add additional nodes on the shafts and define the external force function:
```
Custom.amount                  = 2;
Custom.inShaftNo               = [1; 1];
Custom.positionOnShaftDistance = [20; 497.2]*1e-3;
Custom.force = @(qn, dqn, tn, omega, domega, ddomega, Parameter) customForce(qn, dqn, tn, omega, domega, ddomega, Parameter);
```
Note 1: The first three parameters are used to add additional nodes. In the example above, two nodes are added to the first shaft at positions 20 mm and 497.2 mm from its left end.

Note 2: By editing `customForce()`, users can define any arbitrary external force. This function will be used by SpoolDyn when solving the dynamic equations. `customForce()` accepts the time `tn`, displacement vector `qn`, velocity vector `dqn`, rotational phase `omega`, rotational speed `domega`, rotational acceleration `ddomega` at time `tn`, and the `Parameter` struct. It should return the external force vector at time tn.


## Modeling Module
### Basic Usage
Once the user has entered all parameters and obtained the `InitialParameter` struct containing all inputs, they can pass `InitialParameter` to `establishModel()` to automatically generate the mesh and global matrices for further calculations.

```
InitialParameter = inputEssentialParameter();

Parameter = establishModel(InitialParameter);
```
Note 1: Users can view the 3D model diagram and the mesh diagram in the `modelDiagram` and `meshDiagram` folders located in the root directory.

Note 2: Node information can be found in `Parameter.Mesh.Node`, and the global matrices are stored in `Parameter.Matrix`.

Note 3: The dynamic equations implemented in this module are described in the reference papers.


### Mesh
In the Modeling Module `establishModel()`, there are two methods for mesh generation: automatic (default) and manual.

The main concept of mesh generation in SpoolDyn involves two steps: 1. identify key points as **Nodes** on the shaft (such as locations of disks and bearings); 2. generate additional **Nodes** between these key points.

**For automatic mesh generation:**
```
InitialParameter = inputEssentialParameter();

Parameter = establishModel(InitialParameter, 'gridFineness', 'low');
```
* `'gridFineness', 'low'`: uses only key points as **Nodes** (default)
* `'gridFineness', 'middle'`: use key points as **Nodes** and generate *some* equidistant **Nodes** between key points.
* `'gridFineness', 'high'`: use key points as **Nodes** and generate *more* equidistant **Nodes** between key points.


**For manual mesh generation:**
For example, in the file `main_example_2`:
```
InitialParameter = inputEssentialParameterTwinSpool();
InitialParameter = inputBearingHertzTwinSpool(InitialParameter);
InitialParameter = inputIntermediateBearingTwinSpool(InitialParameter);

manualGrid{1} = [1,2,1,7,1,1,3]; % for shaft 1
manualGrid{2} = [1,3,4,3]; % for shaft 2
Parameter = establishModel(InitialParameter, "gridFineness", manualGrid);
```
<img src="readme/example2/MeshResultOfShaft1_manual_mesh.png" width="100%" height="" />
<img src="readme/example2/MeshResultOfShaft2_manual_mesh.png" width="100%" height="" />

Note 1: For manual mesh generation, `manualGrid` should be a cell array with dimensions n×1 or 1×n, where n is the number of shafts in the rotor system.

Note 2: Each element of the cell array `manualGrid` is a 1×m vector, where m equals the number of **Key Nodes** on a single shaft minus one.

Note 3: The elements of the 1×m vector represent the number of segments between two adjacent **Key Nodes.**

Note 4: Users can typically review the mesh diagram generated with the default configuration first, and then manually adjust the mesh using the `manualGrid` configuration.


### Model and Mesh Diagrams
By default, SpoolDyn outputs 3D model diagrams and mesh diagrams into the `modelDiagram` and `meshDiagram` folders in the root directory.

To disable the generation of model or mesh diagrams, use the name-value pairs `'isPlotModel', false` and `'isPlotMesh', false`:
```
InitialParameter = inputEssentialParameter();

Parameter = establishModel(InitialParameter, 'isPlotModel', false, 'isPlotMesh', false);
```

## Dynamic Equation Generation Module
The dynamic equation generation module improves solving efficiency by dynamically generating an equation function file `dynamicEquation.m` and other related function files adapted to different rotor systems and rotational conditions in SpoolDyn.

In the `main` file:
```
InitialParameter = inputEssentialParameter(); % input module

Parameter = establishModel(InitialParameter); % modeling module

generateDynamicEquation(Parameter); % dynamic equation generation module
```


## Equation-solving Module
### Basic Usage
The primary function of the equation-solving module in SpoolDyn is to compute the vibration response of the modeled rotor system by utilizing the file generated by the dynamic equation generation module and performing numerical integration.

In the `main` file:
```
InitialParameter = inputEssentialParameter(); % input module

Parameter = establishModel(InitialParameter); % modeling module

generateDynamicEquation(Parameter); % dynamic equation generation module

time_span = [0,10]; % seconds
sampling_frequency = 2^14; % time step during soving is 1/sampling_frequency

[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency); % equation-solving module
```
* `q` is an n×m matrix representing the displacement time history; n is the number of degrees of freedom of the system; m is the number of time points
* `dq` is an n×m matrix representing the velocity time history
* `t` is a 1×m vector representing the time series
* `time_span` is a 1×2 vector specifying the start and end time of the simulation
* `sampling_frequency` defines the sampling frequency used during solving


### Initial conditions
Three types of initial conditions are supported:
* Zero initial values
* Custom initial values
* Using the equilibrium position as the initial condition

**Zero initial values**
```
time_span = [0,10]; 
sampling_frequency = 2^14; 
q0 = 0;
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, q0);
```

**Custom initial values**
```
time_span = [0,10];
sampling_frequency = 2^14; 
q0 = [...]; % n*1 vector representing the initial position for every DOF
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, q0); 
```

**Using the equilibrium position as the initial condition**
```
time_span = [0,10]; 
sampling_frequency = 2^14; 
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'isUseBalanceAsInitial', true); 
```
The equilibrium position will be calculated and saved as a .mat file when the name-value pair `'isUseBalanceAsInitial', true` is used for the first time. The equilibrium position will not be recalculated if the model remains unchanged. To force recalculation of the equilibrium position, use the name-value pair `'isFreshInitial', true`.

```
time_span = [0,10]; 
sampling_frequency = 2^14; 
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'isUseBalanceAsInitial', true, 'isFreshInitial', true);
```


### Solvers
There are three solvers in SpoolDyn:
* `RK`: Fixed-step fourth-order Runge-Kutta method (default)
* `ode45` in MATLAB
* `ode15s` in MATLAB
* `ode23s` in MATLAB

```
time_span = [0,10]; 
sampling_frequency = 2^14; 
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'calculateMethod', 'ode15s');
```

To configure MATLAB solvers, use the `odeset()` function:
```
time_span = [0,10]; 
sampling_frequency = 2^14; 
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'calculateMethod', 'ode15s', 'options', odeset());
```


### Downsampling
```
time_span = [0,10]; 
sampling_frequency = 2^14; 
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'reduceInterval', 5);
```
The name-value pair `'reduceInterval', 5` specifies the output sampling interval for `q`, `dq`, and `t`.


### Operation Status diagram
By default, this module outputs diagrams of rotational phase, speed, and acceleration into the `runningStatusDiagram` folder.

To disable generating these diagrams, use the name-value pair `'isPlotStatus', false`:

```
time_span = [0,10]; 
sampling_frequency = 2^14; 
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'isPlotStatus', false);
```

## Post-processing Module
```
InitialParameter = inputEssentialParameter();

Parameter = establishModel(InitialParameter); 

generateDynamicEquation(Parameter); 

time_span_solving = [0,50];
sampling_frequency = 2^14;
[q, dq, t] = calculateResponse(Parameter, time_span_solving, sampling_frequency); 

SwitchFigure.displacement       = true; % true-> output time history in <signalProcess> folder
SwitchFigure.axisTrajectory     = true;
SwitchFigure.axisTrajectory3d   = true;
SwitchFigure.phase              = true;
SwitchFigure.fftSteady          = true;
SwitchFigure.fftTransient       = true;
SwitchFigure.poincare           = true;
SwitchFigure.poincare_phase     = true;
SwitchFigure.saveFig            = true;
SwitchFigure.saveEps            = false;

time_span =  [25,35] % seconds
signalProcessing(q, dq, t, Parameter, time_span, sampling_frequency, SwitchFigure) % post-processing module
```

All post-processing figures are saved in the `signalProcess` folder.

* `time_span`: a 1×2 vector specifying the time range (in seconds) for processing vibration signals
* `SwitchFigure.displacement = true`: outputs time history diagrams
* `SwitchFigure.axisTrajectory = true`: outputs 2D rotor center orbit diagrams
* `SwitchFigure.axisTrajectory3d = true`: outputs 3D rotor center orbit diagrams
* `SwitchFigure.phase = true`: outputs phase portraits
* `SwitchFigure.fftSteady = true`: outputs steady-state spectrum diagrams
* `SwitchFigure.fftTransient = true`: outputs transient spectrum diagrams using STFT
* `SwitchFigure.poincare = true`: outputs Poincaré sections
* `SwitchFigure.poincare_phase = true`: outputs Poincaré sections with phase portraits
* `SwitchFigure.saveFig = true`: saves .fig files for all diagrams (SpoolDyn saves .png files by default)
* `SwitchFigure.saveEps = true`: saves .eps files for all diagrams

Note 1: If the name-value pair `'reduceInterval', downsampling` is used in `calculateResponse()`, the same downsampling value should be specified in the post-processing module.
```
time_span = [0,10]; 
sampling_frequency = 2^14;
downsampling = 5;
[q, dq, t] = calculateResponse(Parameter, time_span, sampling_frequency, 'reduceInterval', downsampling);

SwitchFigure.displacement       = true; % true-> output time history in <signalProcess> folder
SwitchFigure.axisTrajectory     = true;
SwitchFigure.axisTrajectory3d   = true;
SwitchFigure.phase              = true;
SwitchFigure.fftSteady          = true;
SwitchFigure.fftTransient       = true;
SwitchFigure.poincare           = true;
SwitchFigure.poincare_phase     = true;
SwitchFigure.saveFig            = true;
SwitchFigure.saveEps            = false;

time_span =  [25,35] % seconds
signalProcessing(q, dq, t, Parameter, time_span, sampling_frequency, SwitchFigure, 'reduceInterval', downsampling)
```

Note 2: For detailed name-value pairs available in this module, right-click the `signalProcessing()` function in the MATLAB `main` file and select "Help" to view the documentation.

# references
* Haopeng Zhang, Steven Chatterton, Kaifu Zhang, Donglin Li, Runhan Li, Jin Chen, Andrea Riva, Shuai Gao, Kuan Lu and Paolo Pennacchi. A Dynamic Simulation Tool for Multi-Spool Rotor Systems. (Submitted)
* Haopeng Zhang, Runhan Li, Kuan Lu, Xiaohui Gu, Ruijuan Sang, Donglin Li. Dynamic Behavior of Twin-Spool Rotor-Bearing System with Pedestal Looseness and Rub Impact. Applied Sciences. 2024; 14(3):1181. https://doi.org/10.3390/app14031181