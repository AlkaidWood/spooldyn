%% generateDynamicEquation - Generate dynamic equation file for rotor system simulation
%
% This function creates a MATLAB function file (dynamicEquation.m) that 
% implements the differential equations of motion for rotor dynamics 
% simulations based on system configuration parameters.
%
%% Syntax
%  generateDynamicEquation(Parameter)
%
%% Description
% |generateDynamicEquation| constructs a complete differential equation 
% solver function tailored to the specific rotor system configuration. 
% The generated function:
% * Computes rotational kinematics (position, speed, acceleration)
% * Assembles system matrices (mass, stiffness, damping, gyroscopic)
% * Incorporates various force components (unbalance, gravity, etc.)
% * Handles complex operating conditions (acceleration, deceleration)
% * Supports custom force models
%
%% Input Arguments
% * |Parameter| - System configuration structure containing:
%   * |Status|: [1×1 struct]              % Runtime status parameters
%   * |ComponentSwitch|: [1×1 struct]     % Component activation flags
%   * |Mesh|: [1×1 struct]                 % Discretization results
%   * |Matrix|: [1×1 struct]               % System matrices
%   * |Shaft|: [1×1 struct]                % Shaft parameters
%   * |Disk|: [1×1 struct]                 % Disk parameters
%   * |Bearing|: [1×1 struct]              % Bearing parameters
%   * |IntermediateBearing|: [1×1 struct]  % Intermediate bearing params
%   * |RubImpact|: [1×1 struct]           % Rub-impact parameters
%   * |LoosingBearing|: [1×1 struct]      % Loosening bearing parameters
%   * |CouplingMisalignment|: [1×1 struct] % Coupling parameters
%
%% Generated Function Features
% Function Signature:
%   ddyn = dynamicEquation(tn, yn, dyn, Parameter)
% * Inputs:
%   - |tn|: Current simulation time [s]
%   - |yn|: Displacement vector at time tn
%   - |dyn|: Velocity vector at time tn
%   - |Parameter|: System parameter structure
% * Output:
%   - |ddyn|: Acceleration vector at time tn
%
%% Key Components in Generated Code
% 1. Rotational Kinematics:
%   * Computes angular position, velocity, and acceleration
%   * Handles constant speed, acceleration, and deceleration phases
% 2. Matrix Assembly:
%   * Loads precomputed system matrices
%   * Adjusts matrices for current rotational state
% 3. Force Calculation:
%   * Unbalance forces (speed-dependent)
%   * Gravity forces
%   * Hertzian contact forces (if enabled)
%   * Rub-impact forces (if enabled)
%   * Bearing loosening effects (if enabled)
%   * Coupling misalignment forces (if enabled)
%   * Custom forces (if defined)
% 4. Equation Formulation:
%   * Solves M·ddyn = F - K·yn - C·dyn + G·dyn - N·yn
%
%% Implementation Notes
% * Automatic Code Generation:
%   * Creates optimized MATLAB code for specific system configuration
%   * Overwrites existing dynamicEquation.m file
% * Conditional Inclusion:
%   * Only includes enabled force components
%   * Optimizes computation by excluding unused features
% * Kinematics Handling:
%   * Supports both standard and custom speed profiles
%   * Manages acceleration/deceleration transitions
%
%% Example
% % Load system configuration (After modeling and data saving)
% load('modelParameter.mat', 'Parameter');
% % Generate dynamic equation function
% generateDynamicEquation(Parameter);
% % Use in simulation:
% [q, dq, t] = calculateResponse(Parameter, [0 10], 10000)
%
%% Dependencies
% * Requires component-specific force generation functions:
%   - |generateHertzianForce|, |generateRubImpactForce|, etc.
% * Relies on precomputed system matrices from |establishModel|
%
%% See Also
% dynamicEquation, establishModel, generateHertzianForce, generateRubImpactForce
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function generateDynamicEquation(Parameter)

% cheeck the running status parameters
if ~Parameter.Status.isUseCustomize
    if Parameter.Status.duration < 0 || Parameter.Status.acceleration < 0
        error('duration and acceleration must larger than 0 or equal 0');
    end
    
    if  Parameter.Status.vmax <= 0
        error('vmax must larger than 0');
    end
    
    if Parameter.Status.vmin > Parameter.Status.vmax
        error('vmin should smaller than vmax')
    end
end

%% 

% check the exist of dynamic equation and create .txt
if isfile('dynamicEquation.m')
    delete dynamicEquation.m
end

dq = fopen('dynamicEquation.txt','w');


%%

% write comments, firstly

comments = [...
"%% dynamicEquation";...
"% saving the differential equation in this function";...
"%% Syntax";...
"% ddyn = dynamicEquation(tn, yn, dyn, Parameter)";...
"%% Description";...
"% tn: is the n-th time (s)";...
"% ";...
"% yn: is the displacement at the n-th time";...
"% ";...
"% dyn: is the first derivative at the n-th time";...
"% ";...
"% Parameter: is a struct saving all information about the model";...
"% ";...
"% ddyn: is the second derivative at the n-th time";...
" ";...
" "...
];

% function start
functionStart = [...
"function ddyn = dynamicEquation(tn, yn, dyn, Parameter)";...
" "...
];

fprintf(dq,'%s\n',comments);
fprintf(dq,'%s\n',functionStart);

%%

if ~Parameter.Status.isUseCustomize
    % calculate running status
    Status   = Parameter.Status;
    shaftNum = Parameter.Shaft.amount;
    duration        = Status.duration;
    ratio           = Status.ratio;
    ratio           = [1; ratio]; % the first shaft is basic
    
    
    vmax = zeros(1,shaftNum);
    vmin = zeros(1,shaftNum);
    acceleration = zeros(1,shaftNum);
    for iShaft = 1:1:shaftNum
        vmax(iShaft)           = Status.vmax * ratio(iShaft);
        vmin(iShaft)           = Status.vmin * ratio(iShaft);
        acceleration(iShaft)   = Status.acceleration *ratio(iShaft);
    end % end for
end % end if

%%

if ~Parameter.Status.isUseCustomize
    % write the code to calculate the rotational speed
    % situation 1: a = 0
    if (Status.acceleration == 0) && (Status.isDeceleration == false)
        
        calculateOmega = {...
         ' ';...
         '% calculate phase, speed and acceleration';...
        ['domega = [',num2str(vmax),'];'];...
         'omega = domega .* tn;';...
         ' '...
        };
    
    elseif (Status.acceleration == 0) && (Status.isDeceleration == true)
        error('when acceleration equal 0, isDeceleration must be false')
    end
    
    
    % situation 2: a > 0 & no deceleration
    if (Status.acceleration > 0) && (Status.isDeceleration == false)
        t1 = abs(vmax(1) / acceleration(1)); % the first key point (a>0 -> a=0)
        phase1 = 0.5 * acceleration .* t1^2; % the phase at t1
        
        calculateOmega = {...
         ' ';...
         '% calculate phase, speed and acceleration';...
        ['if tn <= ', num2str(t1)];...
        ['   ddomega = [', num2str(acceleration), '];' ];...
         '   domega  = ddomega .* tn ;';...
         '   omega   = 0.5 * ddomega * tn^2;';...
         'else';...
        ['   ddomega = zeros(1,', num2str(shaftNum),');'];...
        ['   domega  = [', num2str(vmax), '];'];...
        ['   omega   = [', num2str(phase1), '] + domega .* (tn-', num2str(t1), ');'];...
         'end';...
         ' '...
    };
    
    end
    
    
    % situation 3: a > 0 & with deceleration
    if (Status.acceleration > 0) && (Status.isDeceleration == true)
        t1 = abs(vmax(1) / acceleration(1)); % the first key point (a>0 -> a=0)
        phase1 = 0.5 * acceleration * t1^2; % the phase at t1
        t2 = t1 + duration; % the end of uniform motion (a=0 -> a<0)
        phase2 = phase1 + vmax * (t2 - t1);
        t3 = t2 + (abs(vmax(1))-abs(vmin(1)))/abs(acceleration(1)); % (a<0 -> a=0)
        phase3 = phase2 + vmax(1)*(t3-t2) - 0.5*acceleration*(t3-t2)^2;
        
    calculateOmega = {...
     ' ';...
     '% calculate phase, speed and acceleration';...
    ['if tn <= ', num2str(t1)];...
    ['    ddomega = [', num2str(acceleration), '];'];...
    ['    domega  = [', num2str(acceleration),'] * tn;']
    ['    omega   = 0.5 * [', num2str(acceleration), '] * tn^2;'];...
    ['elseif tn <= ', num2str(t2)];...
    ['    ddomega = [', num2str(zeros(1,shaftNum)), '];'];...
    ['    domega  = [', num2str(vmax), '];'];...
    ['    omega   = [', num2str(phase1), '] + [', num2str(vmax), '] * (tn - ', num2str(t1), ' );'];...
    ['elseif tn <= ', num2str(t3)];...
    ['    ddomega = -[', num2str(acceleration), '];'];...
    ['    domega  = [', num2str(vmax), '] - [', num2str(acceleration), '] * (tn - ', num2str(t2), ' );'];...
    ['    omega   = [', num2str(phase2), '] + [', num2str(vmax), ']*(tn - ', num2str(t2), ' ) - 0.5*[', num2str(acceleration), ']*(tn - ', num2str(t2), ' )^2;'];...
    'else';...
    ['    ddomega = [', num2str(zeros(1,shaftNum)), '];'];...
    ['    domega  = [', num2str(vmin), '];'];...
    ['    omega   = [', num2str(phase3), '] + [', num2str(vmin), ']*(tn - ', num2str(t3), ' );'];...
    'end';...
    ' '
    };
    end % end if situation 3
else
    % use customized status function
    calculateOmega = {...
     ' ';...
     '% calculate phase, speed and acceleration';...
     '[ddomega, domega, omega] = Parameter.Status.customize(tn);';...
     ' '
    };
end % end if ~isUseCustomize


calculateOmega = cell2string(calculateOmega);
fprintf(dq,'%s\n', calculateOmega);

%%

% write load matrix part
isRemoveDdomega = ~Parameter.Status.isUseCustomize && (Status.acceleration == 0);
if isRemoveDdomega
loadMatrix1 = [...
" ";...
"% load matrix";...
"M = Parameter.Matrix.mass;";...
"G = Parameter.Matrix.gyroscopic_with_domega;";...
"Q = Parameter.Matrix.unblanceForce;";...
"EDisk = Parameter.Matrix.eccentricity;"
];
else
loadMatrix1 = [...
" ";...
"% load matrix";...
"M = Parameter.Matrix.mass;";...
"G = Parameter.Matrix.gyroscopic;";...
"N = Parameter.Matrix.matrixN;";...
"Q = Parameter.Matrix.unblanceForce;";...
"EDisk = Parameter.Matrix.eccentricity;"
];
end % end if
fprintf(dq,'%s\n', loadMatrix1);


% if there is loosing beaaring, the K and C would be created in bearingLoosingForce.m 
if ~Parameter.ComponentSwitch.hasLoosingBearing
    loadMatrix2 = [...
        "K = Parameter.Matrix.stiffness;";...
        "C = Parameter.Matrix.damping;";...
    ];
    fprintf(dq,'%s\n', loadMatrix2);
end


% if the gravity is taken into account
if Parameter.ComponentSwitch.hasGravity
    loadMatrix3 = "fGravity = Parameter.Matrix.gravity;";
    fprintf(dq,'%s\n', loadMatrix3);
    gravityForce = ' + fGravity';
else
    gravityForce = '';
end


%%

% calculate the dof range of shaft
Node = Parameter.Mesh.Node;
dofInterval = Parameter.Mesh.dofInterval;
shaftNum = Parameter.Shaft.amount;

shaftDof = zeros(shaftNum,2);
for iShaft = 1:1:shaftNum
    iShaftCell = repmat({iShaft}, 1, length(Node));
    isShaftHere = cellfun(@ismember, iShaftCell, {Node.onShaftNo});
    isBearingHere = [Node.isBearing] == false;
    IShaftNode = Node( isShaftHere & isBearingHere );
    startNode = min([IShaftNode.name]);
    endNode = max([IShaftNode.name]);
    shaftDof(iShaft,:) = [dofInterval(startNode,1), dofInterval(endNode,2)];
end

%%

% write the part of processing G N Q
if ~isRemoveDdomega
    for iShaft = 1:1:shaftNum
        processGN = {...
        ['G(', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ', ', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ')',...
        ' = domega(', num2str(iShaft), ')*G(', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ', ', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ');'];...
        ['N(', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ', ', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ')',...
        ' = ddomega(', num2str(iShaft), ')*N(', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ', ', num2str(shaftDof(iShaft,1)), ':', num2str(shaftDof(iShaft,2)), ');'];...
        };
        processGN = cell2string(processGN);
        fprintf(dq,'%s\n', processGN);
    end 
end

%%

% calculate the dof range of disk
Disk = Parameter.Disk;
diskDof = dofInterval(Disk.positionOnShaftNode,:)';
diskDof = diskDof(1,:);
diskInShaftNo = Disk.inShaftNo';
% diskMass = ((Disk.outerRadius.^2 - Disk.innerRadius.^2) .* pi .* Disk.thickness) .* Disk.density;
% diskMass = diskMass';
% eccenticity = Disk.eccentricity';
% coeffQ = (diskMass .* eccenticity);

%%

% write the part of unblance force
if isRemoveDdomega
processQ = {...
 ' ';...
 ' ';...
 '% calculate unblance force';...
['diskInShaftNo = [', num2str(diskInShaftNo), '];'];...
['Q([', num2str(diskDof), '])   = EDisk .* (domega(diskInShaftNo).^2 .* cos(omega(diskInShaftNo)));'];...
['Q([', num2str(diskDof+1), ']) = EDisk .* (domega(diskInShaftNo).^2 .* sin(omega(diskInShaftNo)));'];...
 ' '...
};
else
processQ = {...
 ' ';...
 ' ';...
 '% calculate unblance force';...
['diskInShaftNo = [', num2str(diskInShaftNo), '];'];...
['Q([', num2str(diskDof), '])   = EDisk .* ( ddomega(diskInShaftNo) .* sin(omega(diskInShaftNo)) + domega(diskInShaftNo).^2 .* cos(omega(diskInShaftNo)));'];...
['Q([', num2str(diskDof+1), ']) = EDisk .* (-ddomega(diskInShaftNo) .* cos(omega(diskInShaftNo)) + domega(diskInShaftNo).^2 .* sin(omega(diskInShaftNo)));'];...
 ' '...
};
end
processQ = cell2string(processQ);
fprintf(dq,'%s\n', processQ);


%%

% Hertzian contact force
if Parameter.ComponentSwitch.hasHertzianForce
    % generate hertzianForce.m
    if Parameter.ComponentSwitch.hasIntermediateBearing
        generateHertzianForce(Parameter.Mesh,Parameter.ComponentSwitch, Parameter.Shaft, Parameter.Bearing, Parameter.IntermediateBearing);
    else
        generateHertzianForce(Parameter.Mesh,Parameter.ComponentSwitch, Parameter.Shaft, Parameter.Bearing);
    end
    % write codes in dynamicEquation.m
    processHertzianForce = [...
" ";...
"% calculate Hertzian force";...
"fHertz = hertzianForce(yn, tn, domega);";...
" ";...
];
    fprintf(dq,'%s\n', processHertzianForce);
    hertzForce = ' + fHertz';
else
    hertzForce = '';
end

%%

% rub-impact force
if Parameter.ComponentSwitch.hasRubImpact
    generateRubImpactForce(Parameter.RubImpact, Parameter.Mesh)
    processRubImpactForce = [...
" ";...
"% calculate rub-impact force";...
"fRub = rubImpactForce(yn);";...
" ";...
];
    fprintf(dq,'%s\n', processRubImpactForce);
    rubForce = ' + fRub';
else
    rubForce = '';
end

%%

% loosing bearing

if Parameter.ComponentSwitch.hasLoosingBearing
    generateBearingLoosingForce(Parameter.LoosingBearing, Parameter.Mesh);
    proecessLoosingBearing = [...
" ";...
"% check the loosing bearing. If bearing loose, create the KLoose, CLoose.";...
"[K, C] = bearingLoosingForce(yn, Parameter.Matrix);";...
" ";...
];
    fprintf(dq,'%s\n',  proecessLoosingBearing);
end

%%

% coupling misalignment force
if Parameter.ComponentSwitch.hasCouplingMisalignment
    generateMisalignmentForce(Parameter.CouplingMisalignment, Parameter.Mesh)
    processMisalignment = [...
" ";...
"% calculate couplingment misalignment force";...
"fMisalignment = misalignmentForce(omega, domega);";...
" ";...
];
    fprintf(dq,'%s\n', processMisalignment);
    misForce = ' + fMisalignment';
else
    misForce = '';
end

%%

% customize force
if Parameter.ComponentSwitch.hasCustom
processCustomForce = [...
 " ";...
 "% calculate customize force";...
 "fCustom = Parameter.Custom.force(yn, dyn, tn, omega, domega, ddomega, Parameter);";...
 " ";...
];
    fprintf(dq,'%s\n', processCustomForce);
    customForce = ' + fCustom';
else
    customForce = '';
end

%%

% write total force
totalForce = {...
 ' ';...
 '% total force ';...
 ['F = Q', gravityForce, hertzForce, rubForce, misForce, customForce, ';'];...
 ' ';...
 };
totalForce = cell2string(totalForce);
fprintf(dq,'%s\n', totalForce);

%% 

% write dynamic equation
if isRemoveDdomega
dynamicEquationStr = [...
" ";...
"% dynamic equation";...
"term1 = K * yn;";...        
"term2 = C * dyn - G * dyn;";...
"rhs = F - term1 - term2;";...
"ddyn = M \ rhs;";...
" ";...
];
else
dynamicEquationStr = [...
" ";...
"% dynamic equation";...
"term1 = K * yn;";...        
"term1 = term1 - N * yn;";... 
"term2 = C * dyn - G * dyn;";...
"rhs = F - term1 - term2;";...
"ddyn = M \ rhs;";...
" ";...
];
end
fprintf(dq,'%s\n', dynamicEquationStr);

%%

% function end
functionEnd = [...
"end";...
" "...
];
fprintf(dq,'%s\n',functionEnd);

%%
% close .txt and transfer .txt -> .m
fclose(dq);
system('rename dynamicEquation.txt dynamicEquation.m');


end % end function