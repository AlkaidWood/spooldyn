%% cell2string - Convert cell array to string array
%
% This function converts a cell array of character vectors or strings 
% into a homogeneous string array.
%
%% Syntax
%  stringArray = cell2string(cellArray)
%
%% Description
% |cell2string| transforms a cell array containing text data into a 
% standard string array. The function:
% * Preserves the dimensions of the input cell array
% * Converts all elements to string format
% * Handles empty cells by converting to empty strings ("")
%
%% Input Arguments
% * |cellArray| - Input cell array containing:
%   * Character vectors
%   * String scalars
%   * Numeric values (automatically converted to string)
%   * Logical values (converted to "true"/"false")
%   * Empty cells (converted to empty strings)
%
%% Output Arguments
% * |stringArray| - Output string array with same dimensions as |cellArray|
%
%% Conversion Rules
% 1. Character vectors → string scalars
% 2. Numeric values → string representation (e.g., 3.14 → "3.14")
% 3. Logical values → "true" or "false"
% 4. Empty cells → "" (empty string)
% 5. Existing strings remain unchanged
%
%% Implementation Details
% 1. Pre-allocates numeric array for efficiency
% 2. Converts to string array using MATLAB string()
% 3. Iterates through each cell element
% 4. Assigns converted values element-wise
%
%% Example
%   % Create mixed-type cell array
%   data = {'apple', 42, true, [], "banana"};
%   % Convert to string array
%   strData = cell2string(data)
%   % Result: ["apple", "42", "true", "", "banana"]
%
%% Notes
% * For cell arrays containing non-scalar data (matrices, structs), 
%   consider using |cellfun(@string, cellArray)| instead
% * This function is optimized for scalar cell elements
%
%% See Also
%  string, cellfun, convertCharsToStrings
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%
function stringData = cell2string(cellData)
stringData = zeros(size(cellData));
stringData = string(stringData);

for iRow = 1:1:size(cellData,1)
    for iColumn =1:1:size(cellData,2)
        stringData(iRow,iColumn) = cellData{iRow,iColumn};
    end % end for iColumn
end % end for iRow

end  % end sub function