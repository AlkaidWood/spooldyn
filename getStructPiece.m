%% getStructPiece - Extract subset of data from structured arrays
%
% This function extracts specified rows and/or columns from all fields of a 
% structure containing matrix data, creating a new structure with the same 
% fields but reduced data sets.
%
%% Syntax
%  StructPiece = getStructPiece(Target, rowIndex, columnIndex)
%  StructPiece = getStructPiece(Target, rowIndex, columnIndex, isCheck)
%
%% Description
% |getStructPiece| creates a new structure by extracting specified elements 
% from each field of the input structure. The function:
% * Maintains field names and organization
% * Supports row/column indexing for matrix fields
% * Includes optional dimension consistency checking
%
%% Input Arguments
% * |Target| - Input structure with matrix fields [struct]:
%   * Fields must contain numeric/logical matrices
%   * All fields should have same row count (if |isCheck=true|)
% * |rowIndex| - Row indices to extract [integer vector]:
%   * Use |[]| to select all rows
% * |columnIndex| - Column indices to extract [integer vector]:
%   * Use |[]| to select all columns
% * |isCheck| - (Optional) Dimension validation flag [logical]:
%   * |true|: Verify consistent row dimensions (default)
%   * |false|: Skip dimension check
%
%% Output Arguments
% * |StructPiece| - Output structure with extracted data [struct]:
%   * Same fields as |Target|
%   * Each field contains only specified rows/columns
%   * Maintains original data types
%
%% Dimension Requirements
% When |isCheck=true| (default):
% * All fields must have same number of rows
% * Column counts may vary
% * Returns error if row dimensions are inconsistent
%
%% Indexing Rules
% 1. Both |rowIndex| and |columnIndex| cannot be empty
% 2. Extraction cases:
%   * |rowIndex=[]| → Extract all rows from specified columns
%   * |columnIndex=[]| → Extract all columns from specified rows
%   * Both specified → Extract submatrix at row/column indices
%
%% Implementation Details
% 1. Validation:
%   * Checks row dimension consistency (if enabled)
%   * Verifies at least one index is non-empty
% 2. Extraction:
%   * Processes each field independently
%   * Applies identical indexing to all fields
% 3. Output:
%   * Creates new structure with extracted data
%
%% Example
% % Create test structure
% data.a = [1 2 3; 4 5 6];
% data.b = [7 8; 9 10];
% 
% % Extract first row from all fields
% piece1 = getStructPiece(data, 1, [])
% % piece1.a = [1 2 3]
% % piece1.b = [7 8]
%
% % Extract second column from all fields
% piece2 = getStructPiece(data, [], 2)
% % piece2.a = [2; 5]
% % piece2.b = [8; 10]
%
% % Extract element at (2,1)
% piece3 = getStructPiece(data, 2, 1)
% % piece3.a = 4
% % piece3.b = 9
%
%% Application Notes
% * Ideal for processing parameter sets in rotor dynamics
% * Useful for extracting specific components from FEM results
% * Enables batch processing of structured configuration data
%
%% See Also
% struct
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function StructPiece = getStructPiece(Target,rowIndex,columnIndex, isCheck)

if nargin < 4
    isCheck = true;
end

%%

% check the input
checkData = struct2cell(Target);
fieldNum = length(checkData);
dimension = zeros(fieldNum,1);

for iData =1:1:fieldNum
    dimension(iData) = size( checkData{iData}, 1);
end

isDimensionEqual = length(unique(dimension)) == 1;
if ~isDimensionEqual && isCheck
    error('the dimension of data in every field must be same')
end

isNullRow = isempty(rowIndex);
isNullColumn = isempty(columnIndex);

if isNullRow  && isNullColumn
    error('index is empty')
end



%%

fieldName = fieldnames(Target);

for iField = 1:1:fieldNum
    fullData = Target.(fieldName{iField});
    if isNullRow
        StructPiece.(fieldName{iField}) = fullData(:,columnIndex);
    elseif isNullColumn
        StructPiece.(fieldName{iField}) = fullData(rowIndex,:);
    else
        StructPiece.(fieldName{iField}) = fullData(rowIndex,columnIndex);
    end % end if 
end % end for

end % end function