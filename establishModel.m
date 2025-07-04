%% establishModel - Generate system matrices for rotor dynamics analysis
%
% This function constructs the complete finite element model for rotor systems,
% assembling mass, stiffness, damping, and gyroscopic matrices from individual
% components. It serves as the core model assembly routine for rotor dynamics
% simulations.
%
%% Syntax
%  Parameter = establishModel(InitialParameter)
%  Parameter = establishModel(InitialParameter, NameValues)
%
%% Description
% |establishModel| performs the complete finite element assembly process for
% rotor systems by:
% * Generating mesh discretization
% * Creating component matrices (shaft, disk, bearing)
% * Assembling global system matrices
% * Applying numerical conditioning
% * Handling special configurations (loose bearings, custom rotation profiles)
%
%% Input Arguments
% * |InitialParameter| - System configuration structure with fields:
%   * |Shaft|: [1×1 struct]               % Shaft geometric/material properties
%   * |Disk|: [1×1 struct]                % Disk inertial properties
%   * |Bearing|: [1×1 struct]             % Bearing stiffness/damping properties
%   * |ComponentSwitch|: [1×1 struct]     % Component activation flags:
%       .hasIntermediateBearing
%       .hasLoosingBearing
%       .hasRubImpact
%       .hasCouplingMisalignment
%   * |IntermediateBearing|: [1×1 struct] % Intermediate bearing parameters
%   * |RubImpact|: [1×1 struct]           % Rub-impact properties
%   * |LoosingBearing|: [1×1 struct]      % Loosening bearing parameters
%   * |CouplingMisalignment|: [1×1 struct] % Coupling misalignment parameters
%   * |Status|: [1×1 struct]              % Operational status parameters
%
%% Name-Value Pair Arguments
% * |gridFineness| - Mesh resolution specification:
%   * |'low'|: Coarse mesh (default)
%   * |'middle'|: Medium mesh density
%   * |'high'|: Fine mesh resolution
%   * [numeric vector]: Custom node positions
% * |isPlotModel| - Display system schematic diagram (default: true)
% * |isPlotMesh| - Visualize mesh discretization (default: true)
% * |matrix_value_tol| - Threshold for matrix element truncation (default: 1e-12)
%
%% Output Structure
% * |Parameter| - Enhanced system structure with fields:
%   * Original input components (|Shaft|, |Disk|, etc.)
%   * |Mesh|: [1×1 struct]                % Discretization results:
%       .nodeDistance                     % Element lengths
%       .Node                             % Node properties array
%       .dofInterval                      % DOF index ranges
%   * |Matrix|: [1×1 struct]              % Assembled system matrices:
%       .mass: sparse matrix              % Global mass matrix (n×n)
%       .stiffness: sparse matrix         % Global stiffness matrix (n×n)
%       .damping: sparse matrix           % Global damping matrix (n×n)
%       .gyroscopic: sparse matrix        % Gyroscopic matrix (n×n)
%       .matrixN: sparse matrix           % Transient matrix (n×n)
%       .unbalanceForce: vector           % Unbalance force vector (n×1)
%       .gravity: vector                  % Gravity force vector (n×1)
%       .eccentricity: vector             % Disk eccentricity vector (m×1)
%       .gyroscopic_with_domega: matrix   % Precomputed gyroscopic matrix (when applicable)
%       .stiffnessLoosing: matrix         % Loosened bearing stiffness (if active)
%       .dampingLoosing: matrix            % Loosened bearing damping (if active)
%
%% Model Assembly Process
% 1. Visualization:
%    * Schematic diagram (|plotModel|)
%    * Mesh visualization (|plotMesh|)
% 2. Discretization:
%    * Mesh generation (|meshModel|)
% 3. Component Matrix Generation:
%    * Shaft FEM matrices (|femShaft|)
%    * Disk FEM matrices (|femDisk|)
%    * Bearing FEM matrices (|femBearing|)
%    * Intermediate bearing matrices (|femInterBearing|)
% 4. Matrix Assembly:
%    * Component matrix expansion
%    * Rayleigh damping application
%    * Global matrix summation
% 5. Numerical Conditioning:
%    * Small value truncation
%    * Sparse matrix conversion
% 6. Special Case Handling:
%    * Loosened bearing matrices
%    * Precomputed gyroscopic terms
%
%% Special Features
% * Rayleigh Damping (Just for rotating part):
%   $ C = \alpha M + \beta K $
% * Matrix Conditioning:
%   Elements < |matrix_value_tol| are zeroed
% * Gyroscopic Precomputation:
%   For constant-speed operation, computes $ G \cdot \omega $ in advance
% * Loosened Bearing Handling:
%   Maintains separate stiffness/damping matrices for fault conditions
%
%% Example
% % Basic model assembly (after generating initial parameters with input..() functions)
% sysModel = establishModel(rotorParams);
%
% % Custom mesh with suppressed visualization
% sysModel = establishModel(rotorParams, ...
%     'gridFineness', linspace(0, 2, 100), ...
%     'isPlotModel', false, ...
%     'isPlotMesh', false);
%
% % High-resolution mesh with strict conditioning
% sysModel = establishModel(rotorParams, ...
%     'gridFineness', 'high', ...
%     'matrix_value_tol', 1e-14);
%
%% Dependencies
% * |plotModel|, |plotMesh| - Visualization functions
% * |meshModel| - Mesh generation
% * |femShaft|, |femDisk|, |femBearing|, |femInterBearing| - Component matrix generators
%
%% See Also
% meshModel, femShaft, femDisk, femBearing, calculateResponse
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%



function Parameter = establishModel(InitialParameter,NameValueArgs)

arguments % name value pair
    InitialParameter 
    NameValueArgs.isPlotModel = true;
    NameValueArgs.isPlotMesh = true;
    NameValueArgs.gridFineness = 'low';
    NameValueArgs.matrix_value_tol = 1e-12;
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
MInterBearing = zeros( length(MDisk) );
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

% delete small values in matrix
M(abs(M) < NameValueArgs.matrix_value_tol) = 0;
K(abs(K) < NameValueArgs.matrix_value_tol) = 0;
G(abs(G) < NameValueArgs.matrix_value_tol) = 0;
N(abs(N) < NameValueArgs.matrix_value_tol) = 0;
C(abs(C) < NameValueArgs.matrix_value_tol) = 0;

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
            G_with_domega(shaftDof(iShaft,1):shaftDof(iShaft,2), shaftDof(iShaft,1):shaftDof(iShaft,2)) ...
                = domega * G(shaftDof(iShaft,1):shaftDof(iShaft,2), shaftDof(iShaft,1):shaftDof(iShaft,2));       
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