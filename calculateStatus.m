%% calculateStatus
% This is a customized function to calculate the acceleration, speed,
% angular at time point "tn"
%% Syntax
% [ddomega, domega, omega] = calculateStatus(tn)
%% Description
% tn: is the n-th time (s)
% 
% ddomega: is the acceleration at the n-th time
% 
% domega: is the speed at the n-th time
% 
% omega: is the angular at the n-th time


function [ddomega, domega, omega] = calculateStatus(tn)

if tn <= 10
   ddomega = [21         25.2];
   domega  = ddomega .* tn ;
   omega   = 0.5 * ddomega * tn^2;
else
   ddomega = zeros(1,2);
   domega  = [210  252];
   omega   = [1050  1260] + domega .* (tn-10);
end


end