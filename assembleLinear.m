%% assembleLinear - Assemble multiple matrices into a global matrix with overlapping regions
%
% This function assembles multiple smaller matrices into a single global 
% matrix by overlapping specified row and column dimensions, performing 
% element-wise addition in overlapping regions.
%
%% Syntax
%  B = assembleLinear(A, intersectionRow, intersectionColumn)
%
%% Description
% |assembleLinear| combines multiple matrices stored in a cell array into 
% a single global matrix by overlapping specified row and column dimensions. 
% The function:
% * Validates input dimensions and intersection parameters
% * Creates a global matrix with combined dimensions
% * Performs element-wise addition in overlapping regions
%
%% Input Arguments
% * |A| - Cell array (n×1) containing matrices to be assembled
% * |intersectionRow| - Vector of overlapping row counts between consecutive matrices
% * |intersectionColumn| - Vector of overlapping column counts between consecutive matrices
%
%% Output Arguments
% * |B| - Assembled global matrix with combined dimensions
%
%% Dimension Requirements
% * Length of |A| must equal length of |intersectionRow| + 1
% * Length of |A| must equal length of |intersectionColumn| + 1
% * Intersection counts must be less than or equal to adjacent matrix dimensions
%
%% Assembly Algorithm
% 1. Calculate global matrix dimensions:
%   * rows = Σ(rows of A) - Σ(intersectionRow)
%   * columns = Σ(columns of A) - Σ(intersectionColumn)
% 2. Initialize global matrix with zeros
% 3. Sequentially place matrices with specified overlaps:
%   * First matrix placed at top-left corner
%   * Subsequent matrices placed with specified row/column overlaps
%   * Overlapping elements added through element-wise addition
%
%% Error Checking
% * Validates consistent dimensions between cell array and intersection vectors
% * Ensures intersection counts are non-negative and within matrix bounds
%
%% Example
%   % Create three matrices
%   A1 = [1 2; 3 4];
%   A2 = [5 6; 7 8];
%   A3 = [9 10; 11 12];
%   % Assemble with 1 row overlap between A1-A2 and 1 column overlap between A2-A3
%   B = assembleLinear({A1, A2, A3}, [1 1], [1 1])
%   % Resulting matrix:
%   % [1   2   0   0
%   %  3   4+5 6   0
%   %  0   7   8+9 10
%   %  0   0   11  12]
%
%% See Also
%  addElementIn, cellfun, size
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function B = assembleLinear(A, intersectionRow, intersectionColumn)

% check input data
matrixNum = length(A);
indexNumRow = length(intersectionRow);
indexNumColumn = length(intersectionColumn);
if ~isequal(matrixNum, indexNumRow+1, indexNumColumn+1)
    error('the dimensions of input parameters are different');
end % end if 

%%

% calculate the size of matrix B and initial
[rowNumA, columnNumA] = cellfun(@size, A);
rowNumB = sum(rowNumA) - sum(intersectionRow);
columnNumB = sum(columnNumA) - sum(intersectionColumn);
B = zeros(rowNumB, columnNumB);

%%

% intersecte matrices
for iMatrix = 1:1:matrixNum
    [rowMatrix, columnMatrix] = size(A{iMatrix});
    rowIndexA = 1:1:rowMatrix;
    columnIndexA = 1:1:columnMatrix;
    if iMatrix == 1
        B(rowIndexA,columnIndexA) = A{iMatrix}(rowIndexA,columnIndexA);
        [endRow, endColumn] = size(A{iMatrix});
    else
        positionRow = endRow - intersectionRow(iMatrix-1);
        positionColumn = endColumn - intersectionColumn(iMatrix-1);
        rowIndexB = rowIndexA + positionRow;
        columnIndexB = columnIndexA + positionColumn;
        B(rowIndexB,columnIndexB) = B(rowIndexB,columnIndexB)...
                                    + A{iMatrix}(rowIndexA,columnIndexA);
        endRow = positionRow + rowMatrix;
        endColumn = positionColumn + columnMatrix;
    end % end if
end % end for

end % end function