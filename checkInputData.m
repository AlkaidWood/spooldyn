%% checkInputData - Validate structural input data dimensions
%
% This function verifies that all fields in a structure have the specified 
% number of rows, ensuring data consistency for rotor dynamics modeling.
%
%% Syntax
%  checkInputData(checkData)
%
%% Description
% |checkInputData| performs dimensional validation on structure fields to 
% ensure they match the required row count specified in the |amount| field. 
% The function:
% * Validates that all non-|amount| fields have exactly |amount| rows
% * Generates detailed error messages for dimension mismatches
% * Supports all matrix types (numeric, logical, cell, etc.)
%
%% Input Arguments
% * |checkData| - Validation structure containing:
%   * |amount| - Required row count for all other fields [scalar integer]
%   * Other fields - Data matrices to validate (must have |amount| rows)
%
%% Validation Rules
% 1. Mandatory field:
%    * |amount| must exist and be a scalar integer
% 2. Field requirements:
%    * All other fields must have exactly |amount| rows
%    * Column dimension is unrestricted
%    * Field data types are not restricted
%
%% Error Handling
% When validation fails, the function:
% 1. Identifies the calling function using stack trace
% 2. Captures the input variable name
% 3. Generates detailed error message containing:
%    * Calling function name
%    * Input variable name
%    * Required row count
%
%% Implementation
% 1. Field processing:
%    * Removes |amount| field from validation
%    * Converts structure to cell array for iteration
% 2. Dimension check:
%    * Compares row count of each field to |amount|
%    * Throws error on first mismatch
%
%% Example
%   % Create bearing parameter structure
%   bearing.amount = 3;
%   bearing.stiffness = [1e8; 1.2e8; 0.9e8]; % 3 rows - valid
%   bearing.damping = [500; 600]; % 2 rows - invalid!
%   % Validate dimensions
%   checkInputData(bearing)
%   % Error: In function inputBearing(), the input data in <bearing> 
%   %        must be a 3 row matrix.
%
%% Application Notes
% * Essential for verifying FEM model parameter structures
% * Used in all parameter input functions (shaft, disk, bearing)
% * Ensures consistent data dimensions before matrix assembly
%
%% See Also
%  inputEssentialParameter
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function checkInputData(checkData)

rowNum = checkData.amount;
checkData = rmfield(checkData,'amount'); % amount is scalar, no check
checkData = struct2cell(checkData);

for iData = 1:1:size(checkData,1)
    if size(checkData{iData}, 1) ~= rowNum 
        % get first input variable name of this function
        variableName = inputname(1); 
        stackName = dbstack(1); % get stack information
        % get the function name calling this function
        fileName = stackName(1).name; 
        error([ 'In function ', fileName  '(), ', 'the input data in <', variableName, '> must be a ', num2str(rowNum), ' row matrix.'])
    end % for if
end % for loop

end % for subfunction checkInputData()