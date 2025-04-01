%% generateMisalignmentForce
% generate a 'misalignmentForce.m' file
%% Syntax
% generateMisalignmentForce(CouplingMisalignment, Mesh)
%% Description
% CouplingMisalignment: is a struct saving the relevant data
%
% Mesh: is a struct saving the information about meshing
%
% generate a misalignmentForce.m file in the root directory
%% Symbols
% $\Delta Y$: parallel offset
%
% $\Delta L$: distance between two shafts
%
% $\Delta \alpha$: angle offset
% 
% $m$: mass of the coupling 
%
% $\Delta E$: equivalent misalignment
% 
% $$\Delta E = \Delta Y / 2 + \Delta L /2 \tan \left( \Delta\alpha / 2 \right)$$
% 


function generateMisalignmentForce(CouplingMisalignment, Mesh)

% check the exist of rubImpactForce and create .txt
if isfile('misalignmentForce.m')
    delete misalignmentForce.m
end

cmf = fopen('misalignmentForce.txt','w');

%%

% write comments, firstly

comments = [...
"%% misalignmentForce";...
"% saving the equation of coupling misalignment force in this function";...
"%% Syntax";...
"% fMisalignment = misalignmentForce(qn, omega, domega)";...
"%% Description";...
"% omega, domega: are vector denoting the phase and speed of the shaft ";...
"% ";...
"% fMisalignment: is misalignment force (vector)";...
" ";...
" "...
];

%%

% function start
functionStart = [...
"function fMisalignment = misalignmentForce(omega, domega)";...
" "...
];

fprintf(cmf,'%s\n',comments);
fprintf(cmf,'%s\n',functionStart);

%%

% calculate the constants in the misalignment force

deltaY     = CouplingMisalignment.parallelOffset;
deltaL     = CouplingMisalignment.distance;
deltaAlpha = CouplingMisalignment.angleOffset;
m          = CouplingMisalignment.mass;
deltaE     = deltaY ./ 2 + deltaL ./ 2 .* tan(deltaAlpha ./ 2); 
coeff      = -2 * m .* deltaE;
misDof = Mesh.dofInterval(CouplingMisalignment.positionOnShaftNode,:)';
misDofX = misDof(1,:);
misDofY = misDof(1,:) +1;

%%
functionBody = {...
 ' ';...
['fMisalignment = zeros(', num2str(Mesh.dofNum), ',1);'];...
['for iMis = 1:1:', num2str(CouplingMisalignment.amount)];...
['    inShaftNo = [', num2str(CouplingMisalignment.inShaftNo'), '];'];...
['    constants = [', num2str(coeff), '] .* domega(inShaftNo).^2 ;'];...
['    fMisalignment([', num2str(misDofX), ']) = constants .* sin(2*omega(inShaftNo));'];...
['    fMisalignment([', num2str(misDofY), ']) = constants .* cos(2*omega(inShaftNo));'];...
 'end';...
 ' ';...
};

functionBody = cell2string(functionBody);

fprintf(cmf,'%s\n', functionBody);

%%

% function end
functionEnd = [...
"end";...
" "...
];
fprintf(cmf,'%s\n',functionEnd);

%%

% close .txt and transfer .txt -> .m
fclose(cmf);
system('rename misalignmentForce.txt misalignmentForce.m');
end