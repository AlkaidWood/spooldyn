%% addTriangularBlock - Add 3D triangular block with semicircular gap visualization
%
% This function creates and displays a 3D triangular block with a semicircular 
% gap at the top, with customizable dimensions, position, and orientation.
%
%% Syntax
%  addTriangularBlock(ax, position, radius, height, width, thickness)
%  addTriangularBlock(ax, position, radius, height, width, thickness, NODE_IN_CIRCLE)
%  addTriangularBlock(ax, position, radius, height, width, thickness, NODE_IN_CIRCLE, axisName)
%  addTriangularBlock(ax, position, radius, height, width, thickness, NODE_IN_CIRCLE, axisName, RotateInfo)
%
%% Description
% |addTriangularBlock| generates a 3D triangular block model with a semicircular 
% top gap and adds it to the specified axes. The block can be oriented along 
% any principal axis (x, y, or z) and supports optional rotation transformations.
%
%% Input Parameters
% * |ax| - Target axes handle for visualization
% * |position| - [x; y; z] coordinates of block center [m]
% * |radius| - Radius of semicircular gap at block top [m]
% * |height| - Vertical height of block [m]
% * |width| - Bottom width of triangular block [m]
% * |thickness| - Depth along orientation axis [m] (default=1)
% * |NODE_IN_CIRCLE| - Number of circumferential interpolation nodes (default=10)
% * |axisName| - Orientation axis ('x', 'y', or 'z'; default='x')
% * |RotateInfo| - Rotation configuration structure with fields:
%   * |isRotate| - Rotation activation flag (logical)
%   * |oringin| - [x; y; z] rotation origin point
%   * |angle| - Rotation angle [degrees]
%   * |direction| - [x; y; z] rotation axis vector
%
%% Geometry Requirements
% * |width| must be greater than 2*|radius|
% * |height| must be greater than 2*|radius|
%
%% Visualization Features
% * Creates triangular block with semicircular top gap
% * Applies semi-transparent metallic color scheme (RGB: [160,175,190]/255)
% * Supports orientation along any principal axis
% * Optional rotation transformation around arbitrary axis
% * Edge lines disabled for cleaner visualization
% * Face alpha set to 0.7 for semi-transparent effect
%
%% Implementation Details
% 1. Generates semicircular gap geometry
% 2. Creates triangular block base
% 3. Performs coordinate transformation based on axis orientation
% 4. Translates to target position
% 5. Applies optional rotation transformation
% 6. Renders with specified color and transparency
%
%% Example
%   % Create figure and axes
%   fig = figure;
%   ax = axes(fig);
%   % Add triangular block at [0,0,0] with rotation
%   RotateInfo = struct('isRotate', true, 'oringin', [0;0;0], ...
%                       'angle', 45, 'direction', [0;1;0]);
%   addTriangularBlock(ax, [0;0;0], 0.2, 1, 0.8, 0.3, 20, 'z', RotateInfo);
%   view(3); axis equal; grid on;
%
%% See Also
%  addCylinder, surf, rotate
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

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