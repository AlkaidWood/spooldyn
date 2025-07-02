%% addTriangularBlock
% Add a triangular block with a simicircle gap in figure along x-axis (default).
%% Syntax
%  addTriangularBlock(position,radius,height,width,thickness,NODE_IN_CIRCLE,axisName,RotateInfo)
%% Description
% position = [xPosition; yPosition; zPosition]: the distance from oringin 
% point to the center of the circle with unit m. 
% 
% radius: radius of the circle at the top of the triangle with unit m.
%
% height: the height of this block
%
% width: the width of the bottom line
%
% thickness: the thickness along x-axis (default).
%
% NODE_IN_CIRCLE: the number of interpolation nodes along a simicircle.
%
% axisName: the axis length dirction being in. Defaul value is 'x'. Optional
% value is 'x', 'y', 'z'.
%
% RotateInfo: is a struct with field isRotate(boolean), orgingin(3*1
% vector), angle(scalar), dirction(3*1 vector)

function addTriangularBlock(ax, position,radius,height,width,thickness,NODE_IN_CIRCLE,axisName,RotateInfo)

% default value of NODE_IN_CIRCLE and axisName
if nargin<9
    RotateInfo.isRotate = false;
end

if nargin<8
    axisName = 'x';
end

if nargin<7
    NODE_IN_CIRCLE = 10;
end

if nargin<6
    thickness = 1;
end

%%

% generate two simicircles
omega = linspace(pi,2*pi,NODE_IN_CIRCLE);
xCircle = [radius * cos(omega);...
           radius * cos(omega)];
yCircle = [radius * sin(omega);...
           radius * sin(omega)];
zCircle = [0 * ones(1,NODE_IN_CIRCLE);...
           thickness * ones(1,NODE_IN_CIRCLE)];
 
 
% generate bottom line
if width < 2*radius
    error('the width must greater than 2*radius')
end

if height < 2*radius
    error('the height must greater than 2*radius')
end

xBottom = [linspace(-width/2,width/2,NODE_IN_CIRCLE);...
           linspace(-width/2,width/2,NODE_IN_CIRCLE)];
yBottom = -height * ones(2,NODE_IN_CIRCLE);
zBottom = [thickness * ones(1,NODE_IN_CIRCLE);...
           0 * ones(1,NODE_IN_CIRCLE)];


% generate side line
xSide = [xCircle(1,1), xCircle(2,1);...
         xBottom(2,1), xBottom(1,1);...
         xCircle(1,end), xCircle(2,end);...
         xBottom(2,end), xBottom(1,end)];
ySide = [yCircle(1,1), yCircle(2,1);...
         yBottom(2,1), yBottom(1,1);...
         yCircle(1,end), yCircle(2,end);...
         yBottom(2,end), yBottom(1,end)];
zSide = [zCircle(1,1), zCircle(2,1);...
         zBottom(2,1), zBottom(1,1);...
         zCircle(1,end), zCircle(2,end);...
         zBottom(2,end), zBottom(1,end)];
x = [xCircle; xBottom; xCircle(1,:)];
y = [yCircle; yBottom; yCircle(1,:)];
z = [zCircle; zBottom; zCircle(1,:)];

%%

%  transformation of coordinates
switch axisName
    case 'x'
        [x,z]           = exchangeTwoValue(x,z);
        [xSide,zSide]   = exchangeTwoValue(xSide,zSide);
        x               = x - thickness/2;
        xSide           = xSide - thickness/2;
    case 'y'
        [y,z]           = exchangeTwoValue(y,z);
        [ySide,zSide]   = exchangeTwoValue(ySide,zSide);
        y               = y - thickness/2;
        ySide           = ySide - thickness/2;
    case 'z'
        z       = z - thickness/2;
        zSide   = zSide - thickness/2;
    otherwise
        error('axisName in addCylinder() must be x, y or z')
end
        
%%

% transiate the center
[x,y,z] = transiateCenter(x,y,z,position);
[xSide,ySide,zSide] = transiateCenter(xSide,ySide,zSide,position);

%%

% plot
% define color
C = zeros(size(z));
C(:,:,1) = 160/255;
C(:,:,2) = 175/255;
C(:,:,3) = 190/255;
s(1)=surf(ax, x,y,z, C, 'EdgeColor', 'none'); hold on
C = zeros(size(zSide([1,2],:)));
C(:,:,1) = 160/255;
C(:,:,2) = 175/255;
C(:,:,3) = 190/255;
s(2)=surf(ax, xSide([1,2],:),ySide([1,2],:),zSide([1,2],:), C, 'EdgeColor', 'none'); hold on
s(3)=surf(ax, xSide([3,4],:),ySide([3,4],:),zSide([3,4],:), C, 'EdgeColor', 'none'); hold on


% rotate the plot
if RotateInfo.isRotate
    for ii = 1:1:size(s,2)
        rotate(s(ii),RotateInfo.direction,RotateInfo.angle,RotateInfo.oringin);
    end % end for
end % end if
%%

%subfunction
function [x,y,z] = transiateCenter(x,y,z,position)
    x = x + position(1); 
    y = y + position(2);
    z = z + position(3);
end
  
function [x,y] = exchangeTwoValue(x,y)
    temporary = x;
    x = y;
    y = temporary;
end  


end