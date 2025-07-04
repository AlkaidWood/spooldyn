%% addCylinder - Add 3D cylinder visualization to axes
%
% This function creates and displays a 3D cylinder model in specified axes, 
% with customizable dimensions, position, and orientation.
%
%% Syntax
%  addCylinder(ax, position, outerRadius, innerRadius, length)
%  addCylinder(ax, position, outerRadius, innerRadius, length, NODE_IN_CIRCLE)
%  addCylinder(ax, position, outerRadius, innerRadius, length, NODE_IN_CIRCLE, axisName)
%
%% Description
% |addCylinder| generates a 3D cylinder visualization with specified 
% dimensions and adds it to the given axes. The cylinder can be oriented 
% along any principal axis (x, y, or z) and supports hollow cylinder 
% visualization through inner/outer radius parameters.
%
%% Input Parameters
% * |ax| - Target axes handle for cylinder display
% * |position| - [x; y; z] coordinates of cylinder center [m]
% * |outerRadius| - Outer radius of cylinder [m]
% * |innerRadius| - Inner radius of cylinder [m] (0 for solid cylinder)
% * |length| - Length of cylinder along specified axis [m]
% * |NODE_IN_CIRCLE| - Number of circumferential interpolation nodes (default=10)
% * |axisName| - Orientation axis ('x', 'y', or 'z'; default='x')
%
%% Visualization Features
% * Creates hollow or solid cylinders based on radius parameters
% * Applies semi-transparent metallic color scheme (RGB: [160,175,190]/255)
% * Maintains axis equal scaling for proportional display
% * Edge lines disabled for cleaner visualization
% * Face alpha set to 0.7 for semi-transparent effect
%
%% Implementation Details
% 1. Generates base cylinder geometry with unit height
% 2. Scales geometry to specified dimensions
% 3. Performs coordinate transformation based on axis orientation
% 4. Adjusts for inner radius using radial scaling
% 5. Translates to target position
% 6. Applies color and transparency settings
%
%% Example
%   % Create figure and axes
%   fig = figure;
%   ax = axes(fig);
%   % Add cylinder along y-axis at position [0,0,0]
%   addCylinder(ax, [0;0;0], 0.5, 0.3, 2, 20, 'y');
%   view(3); axis equal; grid on;
%
%% See Also
%  cylinder, surf, patch
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


%%
function addCylinder(ax, position,outerRadius,innerRadius,length,NODE_IN_CIRCLE,axisName)

% default value of NODE_IN_CIRCLE and axisName
if nargin<7
    axisName = 'x';
end

if nargin<6
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

% define color
C = zeros(size(z));
C(:,:,1) = 160/255;
C(:,:,2) = 175/255;
C(:,:,3) = 190/255;
% plot
surf(ax, x, y, z, C, 'EdgeColor', 'none', 'FaceAlph', 0.7)
hold on
axis equal

end