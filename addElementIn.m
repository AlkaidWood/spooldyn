%% addElementIn - Add matrix elements into specified position of larger matrix
%
% This function inserts a smaller matrix into a larger matrix at a specified 
% position, performing element-wise addition at the overlapping region.
%
%% Syntax
%  C = addElementIn(A, B, position)
%
%% Description
% |addElementIn| adds matrix |B| into matrix |A| at the specified |position|, 
% performing element-wise addition. The function:
% * Validates matrix dimensions and position indices
% * Adds corresponding elements in overlapping regions
% * Returns the combined matrix |C|
%
%% Input Arguments
% * |A| - Base matrix (larger dimensions)
% * |B| - Matrix to be added (smaller dimensions)
% * |position| - [row, column] index pair specifying where the top-left 
%   element of |B| should be placed in |A|
%
%% Output Arguments
% * |C| - Resulting matrix after adding |B| into |A| at specified position
%   (same dimensions as |A|)
%
%% Dimension Requirements
% * |B| must be smaller than or equal to |A| in both dimensions
% * |position| must be a 2-element vector [row, column]
% * |B| must fit within |A| boundaries starting from |position|
%
%% Error Checking
% * Throws error if |B| exceeds |A| boundaries
% * Validates |position| is a 2-element vector
%
%% Algorithm
% 1. Calculate overlapping region indices
% 2. Extract corresponding submatrix from |A|
% 3. Perform element-wise addition with |B|
% 4. Insert result back into |A|
%
%% Example
%   A = zeros(5,5);
%   B = [1 2; 3 4];
%   % Add B into A starting at row 2, column 3
%   C = addElementIn(A, B, [2,3])
%   % Result:
%   % [0 0 0 0 0
%   %  0 0 1 2 0
%   %  0 0 3 4 0
%   %  0 0 0 0 0
%   %  0 0 0 0 0]
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function C = addElementIn(A, B, position)

% check input
[rowNumA, columnNumA] = size(A);
[rowNumB, columnNumB] = size(B);
isOutRow = rowNumB+position(1)-1 > rowNumA;
isOutColumn = columnNumB+position(2)-1 > columnNumA;
if isOutRow || isOutColumn
   error('the smaller matrix exceed the boundary of lagger matrix'); 
end
if (length(position) ~= 2) || (length(unique( size(position) )) ~= 2)
    error('the position should be a 1*2 or 2*1 array')
end

%%
% add element of B into A
rowIndexInB = 1:1:rowNumB;
columnIndexInB = 1:1:columnNumB;
rowIndexInA = rowIndexInB + position(1) - 1;
columnIndexInA = columnIndexInB + position(2) -1;


C = A;
C(rowIndexInA, columnIndexInA) = A(rowIndexInA, columnIndexInA)...
                                 + B(rowIndexInB, columnIndexInB);
                             
end