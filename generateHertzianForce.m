%GENERATEHERTZIANFORCE Create Hertzian contact force calculation function
%
% Syntax:
%   generateHertzianForce(Mesh, ComponentSwitch, Shaft, Bearing)
%   generateHertzianForce(Mesh, ComponentSwitch, Shaft, Bearing, InterBearing)
%
% Input Arguments:
%   Mesh - Discretization structure containing:
%       .dofNum: integer                Total number of DOFs
%       .dofInterval: [n×2 double]      DOF ranges for nodes
%   ComponentSwitch - Component activation structure with:
%       .hasIntermediateBearing: logical  Intermediate bearing flag
%   Shaft - Shaft configuration structure with:
%       .amount: integer                Number of shafts
%   Bearing - Main bearing parameters structure with:
%       .isHertzian: logical vector     Hertzian contact activation flags
%       .inShaftNo: integer vector      Connected shaft indices
%       .positionOnShaftNode: integer matrix  Node positions on shafts
%       .radiusInnerRace: double vector  Inner race radii
%       .radiusOuterRace: double vector  Outer race radii
%       .rollerNum: integer vector       Roller counts
%       .clearance: double vector        Initial clearances
%       .contactStiffness: double vector Hertz stiffness coefficients
%       .coefficient: double vector     Force exponents
%   InterBearing - (Optional) Intermediate bearing parameters with:
%       .isHertzian: logical vector     Hertzian contact flags
%       .innerShaftNo: integer vector   Inner shaft indices
%       .betweenShaftNo: integer matrix Connected shaft pairs
%       .positionNode: integer matrix   Node positions
%
% Description:
%   Generates hertzianForce.m file containing bearing contact force calculations:
%   - Processes both main and intermediate bearings
%   - Configures DOF mappings for shaft connections
%   - Implements speed-dependent contact force equations
%   - Handles ground-connected and shaft-connected bearing configurations
%
% Notes:
%   - Overwrites existing hertzianForce.m file
%   - Requires companion function hertzianForceEq for force calculations
%   - Automatic DOF adaptation for ground-connected bearings
%
% See also HERTZIANFORCEEQ, BEARINGLOOSINGFORCE, GENERATEDYNAMICEQUATION
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.

function generateHertzianForce(varargin)
%% input
if nargin<4
    error('Please input bearing information')
elseif nargin==4
    Mesh = varargin{1};
    ComponentSwitch = varargin{2};
    Shaft = varargin{3};
    Bearing = varargin{4};
elseif nargin==5
    Mesh = varargin{1};
    ComponentSwitch = varargin{2};
    Shaft = varargin{3};
    Bearing = varargin{4};
    InterBearing = varargin{5};
else
    error('Too much input parameter')
end % end if

% load some parameters
dofNum = Mesh.dofNum;
dofInterval = Mesh.dofInterval;
shaftNum = Shaft.amount;


%% calculate essential parameters for normal bearing
% the number of the hertzian force on the normal bearing
hertzianNumN = sum(Bearing.isHertzian); 

% initial
omegaiNoN = zeros(hertzianNumN, 1); % saving the inner shaft No for Herzian force
omegaoNoN = zeros(hertzianNumN, 1); % saving the outer shaft No for Herzian force

% hertzDof: is an index n*2 matrix. The first column saves the dof No. of
% the Inner shaft, the second column saves the dof No. of the Outer shaft.
% If this functioin is used to calculte the Hertzian force of a Bearing
% connecting the ground, the second column would expect a "dofNum+1". The
% i-th bearing info should be saved in the i-th row.
hertzDofN = zeros(hertzianNumN, 2); % hertzDof for normal bearing

riN = zeros(hertzianNumN, 1);
roN = zeros(hertzianNumN, 1);
nbN = zeros(hertzianNumN, 1);
delta0N = zeros(hertzianNumN, 1);
kHertzN = zeros(hertzianNumN, 1);
nN = zeros(hertzianNumN, 1);

% calculate
iHertz = 1;
for iBearing = 1:1:Bearing.amount
    if Bearing.isHertzian(iBearing)
        % for inner shaft
        shaftNo = Bearing.inShaftNo(iBearing);
        omegaiNoN(iHertz) = shaftNo;
        nodeNo = Bearing.positionOnShaftNode(iBearing);
        hertzDofN(iHertz, 1) = dofInterval(nodeNo, 1); % the x-direction dof of the node connecting this bearing
        
        % for outer shaft
        omegaoNoN(iHertz) = shaftNum + 1; % indicates the rotation speed of outer race equals to 0 (see hertzianForce.m)
        hasMass = sum(Bearing.mass(iBearing, :)) ~= 0;
        if hasMass
            massNodeNo = Bearing.positionNode(iBearing, 1);
            hertzDofN(iHertz, 2) = dofInterval(massNodeNo, 1);
        else
            hertzDofN(iHertz, 2) = dofNum + 1; % indicates the outer race connecting the ground (see hertzianForce.m)
        end % end if hasMass
        
        % for usual parameters
        riN(iHertz) = Bearing.radiusInnerRace(iBearing);
        roN(iHertz) = Bearing.radiusOuterRace(iBearing);
        nbN(iHertz) = Bearing.rollerNum(iBearing);
        delta0N(iHertz) = Bearing.clearance(iBearing);
        kHertzN(iHertz) = Bearing.contactStiffness(iBearing);
        nN(iHertz) = Bearing.coefficient(iBearing);
        
        % update
        iHertz = iHertz + 1;
    end % end if
end % end for


%% calcualte essential parameters for intermediate bearing
if ComponentSwitch.hasIntermediateBearing
    % the number of the hertzian force on the intermediate bearing
    hertzianNumI = sum(InterBearing.isHertzian); 
    
    % initial
    omegaiNoI = zeros(hertzianNumI, 1);
    omegaoNoI = zeros(hertzianNumI, 1);
    hertzDofI = zeros(hertzianNumI, 2); % hertzDof for intermediate bearing
    riI = zeros(hertzianNumI, 1);
    roI = zeros(hertzianNumI, 1);
    nbI = zeros(hertzianNumI, 1);
    delta0I = zeros(hertzianNumI, 1);
    kHertzI = zeros(hertzianNumI, 1);
    nI = zeros(hertzianNumI, 1);
    
    % calculate
    % generate outer shaft No based on the InterBearing.innershaftNo
    outerShaftNo = zeros(InterBearing.amount, 1); % initial
    for iBearing = 1:1:InterBearing.amount
        if InterBearing.innerShaftNo(iBearing)~=InterBearing.betweenShaftNo(iBearing, 1)
            outerShaftNo(iBearing) = InterBearing.betweenShaftNo(iBearing, 1);
        else
            outerShaftNo(iBearing) = InterBearing.betweenShaftNo(iBearing, 2);
        end % end if
    end % end for
    
    % calculate omegaiNoI, omegaoNoI, hertzDofI
    iHertz = 1;
    for iBearing = 1:1:InterBearing.amount
        if InterBearing.isHertzian(iBearing)
            % for inner shaft
            shaftNo = InterBearing.innerShaftNo(iBearing); % inner shaft No
            omegaiNoI(iHertz) = shaftNo;
            nodeNoIndex = find(InterBearing.betweenShaftNo(iBearing,:)==shaftNo);
            nodeNo = InterBearing.positionOnShaftNode(iBearing, nodeNoIndex);
            hertzDofI(iHertz, 1) = dofInterval(nodeNo, 1); % the x-direction dof of the node connecting this bearing
            
            % for outer shaft
            omegaoNoI(iHertz) = outerShaftNo(iBearing);
            hasMass = sum(InterBearing.mass(iBearing, :)) ~= 0;
            if hasMass
                if InterBearing.isHertzianTop(iBearing)
                    massNodeNo = InterBearing.positionNode(iBearing, 1);
                else
                    massNodeNoIndex = find(InterBearing.positionNode(iBearing, :),1,'last');
                    massNodeNo = InterBearing.positionNode(iBearing, massNodeNoIndex);
                end % end if top
                hertzDofI(iHertz, 2) = dofInterval(massNodeNo, 1);
            else
                nodeNoIndex = find(InterBearing.betweenShaftNo(iBearing,:)==outerShaftNo(iBearing));
                nodeNo = InterBearing.positionOnShaftNode(iBearing, nodeNoIndex);
                hertzDofI(iHertz, 2) = dofInterval(nodeNo, 1);
            end % end if hasMass
            
            % for usual parameters
            riI(iHertz) = InterBearing.radiusInnerRace(iBearing);
            roI(iHertz) = InterBearing.radiusOuterRace(iBearing);
            nbI(iHertz) = InterBearing.rollerNum(iBearing);
            delta0I(iHertz) = InterBearing.clearance(iBearing);
            kHertzI(iHertz) = InterBearing.contactStiffness(iBearing);
            nI(iHertz) = InterBearing.coefficient(iBearing);
            
            % update
            iHertz = iHertz + 1; 
        end % end if
    end % end for
end % end if (intermediate bearing)


%% Combining parameters in both Bearing and Intermediate Bearing 
if ComponentSwitch.hasIntermediateBearing
    nb = [nbN; nbI]';
    ri = [riN; riI]';
    ro = [roN; roI]';
    delta0 = [delta0N; delta0I]';
    kHertz = [kHertzN; kHertzI]';
    n = [nN; nI]';
    omegaiNo = [omegaiNoN; omegaiNoI]';
    omegaoNo = [omegaoNoN; omegaoNoI]';
    hertzDof = [hertzDofN; hertzDofI]';
    hertzianNum = hertzianNumN + hertzianNumI;
else
    nb = nbN';
    ri = riN';
    ro = roN';
    delta0 = delta0N';
    kHertz = kHertzN';
    n = nN';
    omegaiNo = omegaiNoN';
    omegaoNo = omegaoNoN';
    hertzDof = hertzDofN';
    hertzianNum = hertzianNumN;
end


%% check the exist of hertzianForce and create .txt
if isfile('hertzianForce.m')
    delete hertzianForce.m
end

rif = fopen('hertzianForce.txt','w');

% generate .m fucntion
% write comments
comments = [...
"%% hertzianForce";...
"% calculate the Herizan contact force in the bearing";...
"%% Syntax";...
"% f = herzianForce(qn, tn, domega)";...
"%% Description";...
"% hertzDof: is an index n*2 matrix. The first column saves the dof No. of";...
"% the Inner shaft, the second column saves the dof No. of the Outer shaft.";...
"% If this functioin is used to calculte the Hertzian force of a Bearing";...
"% connecting the ground, the second column would expect a dofNum+1. The";...
"% i-th bearing info should be saved in the i-th row.";...
"%";...
"% omegaiNo, omegaoNo: are index vector denoting the inner/outer shaft No.";...
"%";...
"% n: is a vector saving the exponential power of the bearing. ";...
"%";...
"% kHertz: is a vector saving the Hertzian contact stiffness.";...
"% ";...
"% delta0: is a vector saving the initial gap of bearings";...
"%";...
"% ro, ri: is a vector saving the outer/inner radius of bearings";...
"%";...
"% nb: is a vector saving the nubmer of rollers in the bearing";...
"%";...
"% domega: is a vector saving the rotational speed of shafts";...
"%";...
"% tn: is a scalar denoting the time (s)";...
"% ";...
"% qn: is a vector saving the displacement of dofs";...
"% ";...
"% fHertz: is a dofNum*1 vector denoting the Hertzian force";...
];

% function start
functionStart = [...
"function fHertz = hertzianForce(qn, tn, domega)";...
" "...
];

fprintf(rif,'%s\n',comments);
fprintf(rif,'%s\n',functionStart);

% function body
functionBody = {...
['nb = [', num2str(nb), '];'];...
['ri = [', num2str(ri), '];'];...
['ro = [', num2str(ro), '];'];...
['delta0 = [', num2str(delta0), '];'];...
['kHertz = [', num2str(kHertz), '];'];...
['n = [', num2str(n), '];'];...
['omegaiNo = [', num2str(omegaiNo), '];'];...
['omegaoNo = [', num2str(omegaoNo), '];'];...
['hertzDof = [', num2str(hertzDof(1,:)), ';...'];...
['            ', num2str(hertzDof(2,:)), ']'';'];...
' ';...
'domega = [domega, 0]; % add 1 row for adapting the bearing connecting the ground';...
'qn = [qn; 0; 0]; % add 2 rows for adapting the bearing connecting the ground';...
' ';...
['fHertz2 = zeros(', num2str(dofNum+2), ',1); % add 2 rows for adapting the bearing connecting the ground'];...
['for iHertz = 1:1:', num2str(hertzianNum)];...
'    x = qn(hertzDof(iHertz,1)) - qn(hertzDof(iHertz,2)); % displacement of the Inner shaft - that of the Outer shaft';...
'    y = qn(hertzDof(iHertz,1)+1) - qn(hertzDof(iHertz,2)+1);';...
'    f = hertzianForceEq(tn, x, y, domega(omegaiNo(iHertz)), domega(omegaoNo(iHertz)), nb(iHertz), ri(iHertz), ro(iHertz), delta0(iHertz), kHertz(iHertz), n(iHertz));';...
'    fHertz2(hertzDof(iHertz,1):hertzDof(iHertz,1)+1) = -f; % the relative displacement is (x_Inner - x_Outer), so the force on the Inner shaft should add a minus "-".';...
'    fHertz2(hertzDof(iHertz,2):hertzDof(iHertz,2)+1) = f; % the force on the outer shaft/ground';...
'end';...
'fHertz = fHertz2(1:end-2);';...
' ';...
};

functionBody = cell2string(functionBody);
fprintf(rif,'%s\n', functionBody);

% function end
functionEnd = [...
"end";...
" "...
];
fprintf(rif,'%s\n',functionEnd);

% close .txt and transfer .txt -> .m
fclose(rif);
system('rename hertzianForce.txt hertzianForce.m');



end