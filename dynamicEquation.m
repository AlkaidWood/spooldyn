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
if tn <= 23.8095
    ddomega = [26.3894      31.6673];
    domega  = [26.3894      31.6673] * tn;
    omega   = 0.5 * [26.3894      31.6673] * tn^2;
elseif tn <= 33.8095
    ddomega = [0  0];
    domega  = [628.3185      753.9822];
    omega   = [7479.9825       8975.979] + [628.3185      753.9822] * (tn - 23.8095 );
elseif tn <= 57.619
    ddomega = -[26.3894      31.6673];
    domega  = [628.3185      753.9822] - [26.3894      31.6673] * (tn - 33.8095 );
    omega   = [13763.1678      16515.8014] + [628.3185      753.9822]*(tn - 33.8095 ) - 0.5*[26.3894      31.6673]*(tn - 33.8095 )^2;
else
    ddomega = [0  0];
    domega  = [0  0];
    omega   = [21243.1503      22499.7874] + [0  0]*(tn - 57.619 );
end
 
 
% load matrix
M = Parameter.Matrix.mass;
G = Parameter.Matrix.gyroscopic;
N = Parameter.Matrix.matrixN;
Q = Parameter.Matrix.unblanceForce;
K = Parameter.Matrix.stiffness;
C = Parameter.Matrix.damping;
G(1:28, 1:28) = domega(1)*G(1:28, 1:28);
N(1:28, 1:28) = ddomega(1)*N(1:28, 1:28);
G(29:48, 29:48) = domega(2)*G(29:48, 29:48);
N(29:48, 29:48) = ddomega(2)*N(29:48, 29:48);
 
 
% calculate unblance force
diskInShaftNo = [1  1  2  2];
Q([9  17  37  41])   = [0.00056224  0.00056224  0.00052761  0.00052761] .* ( ddomega(diskInShaftNo) .* sin(omega(diskInShaftNo)) + domega(diskInShaftNo).^2 .* cos(omega(diskInShaftNo)));
Q([10  18  38  42]) = [0.00056224  0.00056224  0.00052761  0.00052761] .* (-ddomega(diskInShaftNo) .* cos(omega(diskInShaftNo)) + domega(diskInShaftNo).^2 .* sin(omega(diskInShaftNo)));
 
 
% total force 
F = Q;
 
 
% dynamic equation 
ddyn = M \ ( F -  (K - N)*yn - (C - G)*dyn );
 
end
 
