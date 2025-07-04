%% triangular2symmetric - Convert triangular matrix to symmetric form
%
% This function transforms an upper or lower triangular matrix into a 
% symmetric matrix while preserving the original diagonal elements.
%
%% Syntax
%   B = triangular2symmetric(A)
%
%% Description
% |triangular2symmetric| creates a symmetric matrix from a triangular 
% input matrix by:
% * Preserving the original diagonal elements
% * Mirroring the off-diagonal elements across the diagonal
%
%% Input Arguments
% * |A| - Input triangular matrix [square matrix]:
%   * Must be either upper or lower triangular
%   * Diagonal elements are preserved in the output
%
%% Output Arguments
% * |B| - Symmetric matrix [square matrix]:
%   * Has same dimensions as input matrix
%   * Satisfies B = B'
%   * Diagonal identical to input diagonal
%
%% Algorithm
% The conversion follows:
%   1. Extract diagonal elements: D = diag(A)
%   2. Remove diagonal: A' = A - diag(D)
%   3. Create symmetric matrix: B = A' + (A')' + diag(D)
%   Simplified to: B = (A + A') - diag(diag(A))
%
%% Implementation Note
% * For upper triangular inputs:
%   * Upper triangle → symmetric
%   * Lower triangle mirrored from upper
% * For lower triangular inputs:
%   * Lower triangle → symmetric
%   * Upper triangle mirrored from lower
%
%% Examples
% % Upper triangular input
% A = [1 2 3; 
%      0 4 5; 
%      0 0 6];
% B = triangular2symmetric(A)
% % Returns: [1 2 3; 
% %          2 4 5; 
% %          3 5 6]
%
% % Lower triangular input
% C = [1 0 0;
%      2 3 0;
%      4 5 6];
% D = triangular2symmetric(C)
% % Returns: [1 2 4; 
% %          2 3 5; 
% %          4 5 6]
%
%% Applications
% * Finite element stiffness matrix assembly
% * Covariance matrix reconstruction
% * Sparse matrix storage conversion
% * Linear system formulation
%
%% See Also
% issymmetric, istriu, istril, diag, transpose
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function B = triangular2symmetric(A)

% Determine whether the matrix is upper triangular or lower triangular
isUpper = istriu(A);
isLower = istril(A);
if ~isUpper && ~isLower
    error('please input a triangular matrix')
end

%%

% transfer
diagElement = diag(A);
B = A - diag(diagElement) +A';

end