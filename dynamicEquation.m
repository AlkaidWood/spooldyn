%% dynamicEquation
% saving the differential equation in this function
%% Syntax
% ddyn = dynamicEquation(tn, yn, dyn, Parameter)
%% Description
% tn: is the n-th time (s)
% 
% yn: is the displacement at the n-th time
% 
% dyn: is the first derivative at the n-th time
% 
% Parameter: is a struct saving all information about the model
% 
% ddyn: is the second derivative at the n-th time
 
 
function ddyn = dynamicEquation(tn, yn, dyn, Parameter)
 
 
% calculate phase, speed and acceleration
[ddomega, domega, omega] = Parameter.Status.customize(tn);
 
 
% load matrix
M = Parameter.Matrix.mass;
G = Parameter.Matrix.gyroscopic;
N = Parameter.Matrix.matrixN;
Q = Parameter.Matrix.unblanceForce;
EDisk = Parameter.Matrix.eccentricity;
K = Parameter.Matrix.stiffness;
C = Parameter.Matrix.damping;
fGravity = Parameter.Matrix.gravity;
G(1:32, 1:32) = domega(1)*G(1:32, 1:32);
N(1:32, 1:32) = ddomega(1)*N(1:32, 1:32);
G(33:52, 33:52) = domega(2)*G(33:52, 33:52);
N(33:52, 33:52) = ddomega(2)*N(33:52, 33:52);
 
 
% calculate unblance force
diskInShaftNo = [1  1  2  2];
Q([13  21  41  45])   = EDisk .* ( ddomega(diskInShaftNo) .* sin(omega(diskInShaftNo)) + domega(diskInShaftNo).^2 .* cos(omega(diskInShaftNo)));
Q([14  22  42  46]) = EDisk .* (-ddomega(diskInShaftNo) .* cos(omega(diskInShaftNo)) + domega(diskInShaftNo).^2 .* sin(omega(diskInShaftNo)));
 
 
% total force 
F = Q + fGravity;
 
 
% dynamic equation 
ddyn = M \ ( F -  (K - N)*yn - (C - G)*dyn );
 
end
 
