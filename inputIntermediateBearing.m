%% inputIntermediateBearingHertz
% Input parameters about intermediate bearing considering Hertzian contact
%% Syntax
% OutputParameter = inputIntermediateBearingHertz(InputParameter)
%% Description
% InputParameter is a struct data saving all initial parameters except
% intermediate bearing parameters considering Hertzian contact.
% 
% OutputParameter is a strct data recording all InputParameter and the data
% about intermediate bearing considering Hertzian contact.
%
% All of parameters about intermediate bearing considering Hertzian contact
% should be typed into this .m file.
%
% All vectors in this .m file should be in column


%%
function OutputParameter = inputIntermediateBearing(InputParameter)

% typing the parameter about intermediate bearing
IntermediateBearing.amount          = 1;
% shaft no. connected by same bearing in row; different bearings in column
IntermediateBearing.betweenShaftNo  =  [1, 2]; % n*2
% dof
IntermediateBearing.dofOfEachNodes =  [2];% if mass=0, dof must be 0 
% the same bearing in row; different bearings in column, n*2
IntermediateBearing.positionOnShaftDistance = 1e-3 *  [579.5, 382]; % from the left end of the shaft
IntermediateBearing.isHertzian      = [true]; % boolean
IntermediateBearing.isHertzianTop   = [true];
% IntermediateBearing.isHertzian      = [false]; % boolean
% IntermediateBearing.isHertzianTop   = [false];
% M K C, elements in the same row: the MKC at the same position of the
% shaft; mass(1,1) -> mass(1,n): 
% the mass near the betweenShaftNo(:,1) -》the mass near the betweenShaftNo(:,2)
% If isHertizian and no mass, the corresponding k c will be added in
% global matrix normally; the model:
% shaft1--Hertz+k1c1--shaft2;
% If is no Hertzian and with mass: there are n mass in a row, and n+1 k c 
% for a bearing; the model will be established as:
% shaft1--k1c1--m1--k2c2--m2--k3c3--m3--k4c4- ...-mn--k(n+1)c(n+1)--shaft2;
% If is no Hertzian and no mass, the k c will be added in global 
% matrix normally; the model:
% shaft1--k1c1--shaft2;
% If isHertzian and with mass, the hertzian force will be added at the mass
% in the first column (near the shaft); the model:
% shaft1--Hertz+k1c1--m1--k2c2--m2--k3c3--m3--k4c4--mn--k(n+1)c(n+1)--shaft2; (isHertzianTop=true)
% shaft1--k1c1--m1--k2c2--m2--k3c3--m3--k4c4--mn--Hertz+k(n+1)c(n+1)--shaft2; (isHertzianTop=false)
IntermediateBearing.stiffness           =  [100, 5e8]; % N/m, in column, n*1
IntermediateBearing.stiffnessVertical   =  [200, 3e8]; % N/m, in column, n*1
IntermediateBearing.damping             =  [800, 150]; % N/s^2, in column, n*1
IntermediateBearing.dampingVertical     =  [500, 300]; % N/s^2, in column, n*1
IntermediateBearing.mass                =  [0.0484]; % kg
% if there is no Hertizan contact force, please set n*1 zero vector for following parameters, where n is the number of the intermediate bearing                                 
IntermediateBearing.rollerNum        = [13];
IntermediateBearing.radiusInnerRace = [10e-3]; % m
IntermediateBearing.radiusOuterRace = [23.5e-3]; % m
IntermediateBearing.innerShaftNo = [1]; % indicates Inner shaft No. 
IntermediateBearing.clearance = [7e-6]; % m
IntermediateBearing.contactStiffness = [1.08e10]; % N*m^-3/2
IntermediateBearing.coefficient = [3/2]; % =3/2 in a ball bearing; = 10/9 in a roller bearing

%%

%check the input data
if size(IntermediateBearing.betweenShaftNo, 2) ~= 2
    error('IntermediateBearingH.betweenShaftNo must be a 2 column matrix')
end
               
if size(IntermediateBearing.positionOnShaftDistance, 2) ~= 2
    error('IntermediateBearingH.positionOnShaftDistance must be a 2 column matrix')
end

checkInputData(IntermediateBearing);

%%

% sort the betweenShaftNo and PositionOnShaftDistance (sort column)
shaftNo = IntermediateBearing.betweenShaftNo; % short the variable
position = IntermediateBearing.positionOnShaftDistance;
for iBearing = 1:1:IntermediateBearing.amount
    if shaftNo(iBearing,1)>shaftNo(iBearing,2)
       % exchange the column 1 and column 2 in iBearing row 
       temporary = shaftNo(iBearing,1);
       shaftNo(iBearing,1) = shaftNo(iBearing,2);
       shaftNo(iBearing,2) = temporary;
       temporary = position(iBearing,1);
       position(iBearing,1) = position(iBearing,2);
       position(iBearing,2) = temporary;
       IntermediateBearing.betweenShaftNo = shaftNo;
       IntermediateBearing.positionOnShaftDistance = position;
       % isHertzianTop, stiffness, damping, mass, dofOfEachNodes should be 
       % updated, if iBearing has mass.
       if sum(IntermediateBearing.mass(iBearing,:))
            % adjust isHertzianTop
            IntermediateBearing.isHertzianTop(iBearing) = ~IntermediateBearing.isHertzianTop(iBearing);
            % adjust mass and dofOfEachNodes
            colNum = size(IntermediateBearing.mass(iBearing,:),2); 
            massHere = IntermediateBearing.mass(iBearing,:);
            dofHere = IntermediateBearing.dofOfEachNodes(iBearing,:);
            massHereNum = sum(massHere~=0);
            massHere = massHere(1:massHereNum);
            dofHere = dofHere(1:massHereNum);
            massInv = flip(massHere);
            dofInv = flip(dofHere);
            IntermediateBearing.mass(iBearing,:) = [massInv, zeros(1,colNum-massHereNum)];
            IntermediateBearing.dofOfEachNodes(iBearing,:) = [dofInv, zeros(1,colNum-massHereNum)];
            % adjust stiffness and damping
            colNum = size(IntermediateBearing.stiffness(iBearing,:),2);
            kHere = IntermediateBearing.stiffness(iBearing,:);
            kHereVertical = IntermediateBearing.stiffnessVertical(iBearing,:);
            cHere = IntermediateBearing.damping(iBearing,:);
            cHereVertical = IntermediateBearing.dampingVertical(iBearing,:);
            kHereNum = massHereNum + 1;
            kHere = kHere(1:kHereNum);
            kHereVertical = kHereVertical(1:kHereNum);
            cHere = cHere(1:kHereNum);
            cHereVertical = cHereVertical(1:kHereNum);
            kInv = flip(kHere);
            kInvVertical = flip(kHereVertical);
            cInv = flip(cHere);
            cInvVertical = flip(cHereVertical);
            IntermediateBearing.stiffness(iBearing,:) = [kInv, zeros(1,colNum-kHereNum)];
            IntermediateBearing.stiffnessVertical(iBearing,:) = [kInvVertical, zeros(1,colNum-kHereNum)];
            IntermediateBearing.damping(iBearing,:) = [cInv, zeros(1,colNum-kHereNum)];
            IntermediateBearing.dampingVertical(iBearing,:) = [cInvVertical, zeros(1,colNum-kHereNum)];
       end
    end
end
% sort columns in struct
IntermediateBearing = sortRowsWithShaftDis(IntermediateBearing);

%%

OutputParameter = InputParameter;
OutputParameter.IntermediateBearing = IntermediateBearing;
OutputParameter.ComponentSwitch.hasIntermediateBearing = true;
if sum(IntermediateBearing.isHertzian)~=0
    OutputParameter.ComponentSwitch.hasHertzianForce = true;
end % end if
end % for function inputIntermediateBearingHertz()




