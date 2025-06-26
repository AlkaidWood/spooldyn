%% ESTABLISHMODEL Generate system matrices for rotor dynamics analysis
% Constructs mass, stiffness, damping, and gyroscopic matrices for rotor systems
% 
% Syntax:
%   Parameter = establishModel(InitialParameter)
%   Parameter = establishModel(InitialParameter, Name, Value)
%
% Input Arguments:
%   InitialParameter - System configuration structure with fields:
%       .Shaft: [1×1 struct]               % Shaft properties
%       .Disk: [1×1 struct]                % Disk components
%       .Bearing: [1×1 struct]             % Bearing components
%       .ComponentSwitch: [1×1 struct]     % Component activation flags
%       .IntermediateBearing: [1×1 struct]  % Intermediate bearing params (optional)
%       .RubImpact: [1×1 struct]            % Rub-impact properties (optional)
%       .LoosingBearing: [1×1 struct]       % Loosing bearing parameters (optional)
%       .CouplingMisalignment: [1×1 struct] % Coupling misalignment params (optional)
%
% Name-Value Pair Arguments:
%   gridFineness - Mesh resolution specification:
%       'low' (default) | 'middle' | 'high' | numeric vector
%   isPlotModel  - Display system schematic diagram (default: true)
%   isPlotMesh   - Visualize mesh discretization (default: true)
%
% Output:
%   Parameter - Enhanced system structure with FEM components:
%       .Shaft: [1×1 struct]               % Original shaft parameters
%       .Disk: [1×1 struct]                % Original disk parameters
%       .Bearing: [1×1 struct]             % Original bearing parameters
%       .ComponentSwitch: [1×1 struct]     % Component activation states
%       .IntermediateBearing: [1×1 struct] % Intermediate bearing params
%       .RubImpact: [1×1 struct]           % Rub-impact parameters
%       .LoosingBearing: [1×1 struct]      % Loosing bearing parameters
%       .CouplingMisalignment: [1×1 struct]% Coupling parameters
%       .Mesh: [1×1 struct]                % Discretization results (see
%                                            meshModel.m)
%       .Matrix: [1×1 struct]              % System matrices:
%           .mass: sparse matrix            % Mass matrix (n×n)
%           .stiffness: sparse matrix      % Stiffness matrix (n×n)
%           .damping: sparse matrix        % Damping matrix (n×n)
%           .gyroscopic: sparse matrix     % Gyroscopic matrix (n×n)
%           .matrixN: sparse matrix        % Nonlinear term matrix (n×n)
%           .unbalanceForce: double vector  % Unbalance force (n×1)
%           .gravity: double vector         % Gravity force (n×1)
%           .eccentricity: double vector    % Disk eccentricities (m×1)
%
% Example:
%   % Basic usage with default settings
%   config = establishModel(sysParams);
%
%   % High-resolution mesh without visualization
%   config = establishModel(sysParams, ...
%       'gridFineness', 'high', ...
%       'isPlotModel', false, ...
%       'isPlotMesh', false);
%
%   % Custom mesh specification
%   customMesh = linspace(0, 1.5, 50);
%   config = establishModel(sysParams, 'gridFineness', customMesh);
%
% See also FEMSHaft, FEMDisk, FEMBearing



function Parameter = establishModel(InitialParameter,NameValueArgs)

arguments % name value pair
    InitialParameter 
    NameValueArgs.isPlotModel = true;
    NameValueArgs.isPlotMesh = true;
    NameValueArgs.gridFineness = 'low';
end

%%

% plot the schematic diagram of the rotor
if NameValueArgs.isPlotModel
    plotModel(InitialParameter);
end

%%

% mesh
Parameter = meshModel(InitialParameter,NameValueArgs.gridFineness); % choose: low, middle, high 
if NameValueArgs.isPlotMesh
	plotMesh(Parameter); % plot mesh result
end

%%

% generate FEM matrices of shaft
[MShaft, KShaft, GShaft, NShaft, FgShaft] = femShaft( Parameter.Shaft,...
                                                      Parameter.Mesh.nodeDistance );

%%

% generate FEM matrices of disk 
[MDisk, GDisk, NDisk, QDisk, FgDisk, EDisk] = femDisk( Parameter.Disk,...
                                                [Parameter.Mesh.Node.dof] );
                             

%%

% generate FEM matrices of bearing
if ~InitialParameter.ComponentSwitch.hasLoosingBearing
    [MBearing, KBearing, CBearing, FgBearing] = femBearing( Parameter.Bearing,...
                                                            [Parameter.Mesh.Node.dof] );
else
    [MBearing, KBearing, CBearing, FgBearing, KLBearing, CLBearing] = femBearing( ...
                                            Parameter.Bearing,...
                                           [Parameter.Mesh.Node.dof],...
                                            Parameter.LoosingBearing);
end
                                         


%%

% generate FEM matrices of intermediate bearing
KInterBearing = zeros( length(MDisk) );
CInterBearing = zeros( length(MDisk) );
FgInterBearing = zeros(length(MDisk), 1);
if Parameter.ComponentSwitch.hasIntermediateBearing
    [MInterBearing, KInterBearing, CInterBearing, FgInterBearing] = femInterBearing( ...
            Parameter.IntermediateBearing, [Parameter.Mesh.Node.dof] );
end


%%

% expand the matrices about shaft
shaftDofNum = length(MShaft);
diskDofNum = length(MDisk);
if diskDofNum > shaftDofNum
    MShaft = blkdiag( MShaft,zeros(diskDofNum - shaftDofNum) );
    KShaft = blkdiag( KShaft,zeros(diskDofNum - shaftDofNum) );
    GShaft = blkdiag( GShaft,zeros(diskDofNum - shaftDofNum) );
    NShaft = blkdiag( NShaft,zeros(diskDofNum - shaftDofNum) );
    FgShaft = [FgShaft; zeros((diskDofNum-shaftDofNum),1)];
end
clear shaftDofNum diskDofNum;

%%

% rayleigh damping
rayleighCoeff = Parameter.Shaft.rayleighDamping;
CShaft = rayleighCoeff(1) * (MShaft+MDisk) + rayleighCoeff(2) * KShaft;


% assemble
M = MShaft + MDisk + MBearing + MInterBearing;
K = KShaft +         KBearing + KInterBearing;
G = GShaft + GDisk;
N = NShaft + NDisk;
C = CShaft +         CBearing + CInterBearing;
Q = QDisk;
Fg = FgShaft + FgDisk + FgBearing + FgInterBearing;

if InitialParameter.ComponentSwitch.hasLoosingBearing
    KLoosing = KShaft + KLBearing + KInterBearing;
    CLoosing = CShaft + CLBearing + CInterBearing;
    KLoosing = sparse(KLoosing);
    CLoosing = sparse(CLoosing);
end

% sparse format
M = sparse(M);
K = sparse(K);
C = sparse(C);
G = sparse(G);
N = sparse(N);
Q = sparse(Q);


% generate the struct Matrix
Matrix.mass = M; % n*n, n is dof number
Matrix.stiffness = K; % n*n
Matrix.gyroscopic = G; % n*n
Matrix.damping = C; % n*n
Matrix.matrixN = N; % n*n
Matrix.unblanceForce = Q; % n*1
Matrix.gravity = Fg; % n*1
Matrix.eccentricity = EDisk'; % 1*m, m is the number of disks


% pre-calculate G matrix to save time if accerleartion=0
if ~Parameter.Status.isUseCustomize
    if (Parameter.Status.acceleration == 0) && (Parameter.Status.isDeceleration == false)
        % initial
        G_with_domega = zeros(size(G));
        % calculate dof range of shafts
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
        % generate G with speed
        ratio = [1; Parameter.Status.ratio];
        for iShaft=1:1:shaftNum
            domega = Parameter.Status.vmax * ratio(iShaft);
            G_wiht_domega(shaftDof(iShaft,1):shaftDof(iShaft,2)) ...
                = domega * G(shaftDof(iShaft,1):shaftDof(iShaft,2));       
        end
        % sparse format
        G_with_domega = sparse(G_with_domega);
        % generate the struct Matrix
        Matrix.gyroscopic_with_domega = G_with_domega; % n*n
    elseif (Parameter.Status.acceleration == 0) && (Parameter.Status.isDeceleration == true)
        error('when acceleration equal 0, isDeceleration must be false')
    end % end if
end % end if


% for saving matrix of loosing bearings
if InitialParameter.ComponentSwitch.hasLoosingBearing
    Matrix.stiffnessLoosing = KLoosing;
    Matrix.dampingLoosing = CLoosing;
end

%%

% output

Parameter.Matrix = Matrix;

end