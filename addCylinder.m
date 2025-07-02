%% addCylinder
% Add a cylinder in figure along x-axis (default).
%% Syntax
%  addCylinder(position,outerRadius,innerRadius,length)
%  addCylinder(position,outerRadius,innerRadius,length,NODE_IN_CIRCLE)
%  addCylinder(position,outerRadius,innerRadius,length,NODE_IN_CIRCLE,axisName)
%% Description
% position = [xPosition; yPosition; zPosition]: the distance from oringin 
% point to the center of the cylinder with unit m. 
% 
% outerRadius: outer radius of this cylinder with unit m.
%
% innerRadius: inner radius of this cylinder with unit m.
%
% length: the length along x-axis (default).
%
% NODE_IN_CIRCLE: the number of interpolation nodes along a circle.
%
% axisName: the axis length dirction being in. Defaul value is 'x'. Optional
% value is 'x', 'y', 'z'.


%%
function addCylinder(position,outerRadius,innerRadius,length,NODE_IN_CIRCLE,axisName)

% default value of NODE_IN_CIRCLE and axisName
if nargin<6
    axisName = 'x';
end

if nargin<5
    NODE_IN_CIRCLE = 10;
end


%  generate a cylinder with unit height, radius = outerRadius
[x,y,z] = cylinder(outerRadius,NODE_IN_CIRCLE);
z = z*length;


% for innerRadius
RATIO_INNER_OUTER = innerRadius / outerRadius; 

%%

%  transformation of coordinates
switch axisName
    case 'x'
        temporary = x;
        x = z;
        z = temporary; % exchange x and z = exange the axle
        x = x - length/2; % transiate the center of cylinder to origin point
        x = x([1 2 2 1 1],:); % for plot, generate 5 curve
        y = [y;y*RATIO_INNER_OUTER;y(1,:)];
        z = [z;z*RATIO_INNER_OUTER;z(1,:)];
    case 'y'
        temporary =  y;
        y = z;
        z = temporary; % exchange y and z
        y = y - length/2;
        y = y([1 2 2 1 1],:);
        x = [x;x*RATIO_INNER_OUTER;x(1,:)];
        z = [z;z*RATIO_INNER_OUTER;z(1,:)];
    case 'z'
        z = z - length/2;
        z = z([1 2 2 1 1],:);
        x = [x;x*RATIO_INNER_OUTER;x(1,:)];
        y = [y;y*RATIO_INNER_OUTER;y(1,:)];
    otherwise
        error('axisName in addCylinder() must be x, y or z')
end


% transiate the center
x = x + position(1); 
y = y + position(2);
z = z + position(3);

%%

% plot
%surf(x([1 2 2 1 1],:),[y;y*RATIO_INNER_OUTER;y(1,:)],[z;z*RATIO_INNER_OUTER;z(1,:)])
% define color
C = zeros(size(z));
C(:,:,1) = 160/255;
C(:,:,2) = 175/255;
C(:,:,3) = 190/255;
surf(x,y,z, C, 'EdgeColor', 'none', 'FaceAlph', 0.7)
hold on
axis equal

end