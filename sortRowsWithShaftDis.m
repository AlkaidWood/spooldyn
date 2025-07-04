%% sortRowsWithShaftDis - Sort bearing elements by shaft position
%
% This function orders bearing and intermediate bearing elements 
% according to their position along their respective shafts.
%
%% Syntax
%   orderedElement = sortRowsWithShaftDis(Element)
%
%% Description
% |sortRowsWithShaftDis| rearranges bearing elements by:
%  1. Grouping elements by shaft number
%  2. Sorting each group by position along the shaft
%  3. Reconstructing the element structure in the new order
%
%% Input Arguments
% * |Element| - Input element structure:
%   * |Bearing| or |IntermediateBearing| structure from parameters
%   * Must contain:
%     - |inShaftNo| or |betweenShaftNo|: Shaft identifier vector
%     - |positionOnShaftDistance|: Position vector along shaft
%     - |amount|: Total element count
%
%% Output Arguments
% * |orderedElement| - Sorted element structure:
%   * Elements sorted first by |inShaftNo|/|betweenShaftNo|
%   * Elements with same shaft number sorted by |positionOnShaftDistance|
%   * Retains all original fields with updated |amount|
%
%% Algorithm
% 1. Data Extraction:
%    * Removes |amount| field for processing
%    * Identifies shaft identifier field (|inShaftNo| or |betweenShaftNo|)
%    * Gets unique shaft numbers
% 2. Group Processing:
%    * For each shaft group:
%      a) Extracts all elements on the shaft
%      b) Sorts elements by |positionOnShaftDistance|
% 3. Data Reconstruction:
%    * Concatenates sorted shaft groups
%    * Restores |amount| field
%
%% See Also
% sortrows, unique, struct2cell, cell2struct
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function orderedElement = sortRowsWithShaftDis(Element)
Temporary = rmfield(Element,'amount');

% divide Element into pieces respect to shaft No.
if isfield(Temporary, 'inShaftNo')
    inShaftNo = Temporary.inShaftNo(:,1);
else
    inShaftNo = Temporary.betweenShaftNo(:,1);
end
shaftNoSet = unique(inShaftNo);
shaftNum = length(shaftNoSet);
% convert strct to cell
fieldName = fieldnames(Temporary); % save the field names
cellData = struct2cell(Temporary); % save the data
% find the index of certain field name in the cell data
if isfield(Temporary, 'inShaftNo')
    inShaftNoIndex = ismember(fieldName, 'inShaftNo');
else
    inShaftNoIndex = ismember(fieldName, 'betweenShaftNo');
end
distanceIndex = ismember(fieldName, 'positionOnShaftDistance');
orderCellData = cell(length(cellData),1);
for iShaft = 1:1:shaftNum
    % find columns with same shaft no
    index = cellData{inShaftNoIndex}(:,1)==iShaft;
    % extract the data
    cellDataShaft = cell(length(cellData),1);
    for iRow = 1:1:length(cellData)
        cellDataShaft{iRow} = cellData{iRow}(index,:);
    end
    % order the data respect to distance
    [~,indexDis] = sort(cellDataShaft{distanceIndex}(:,1));
    for iRow = 1:1:length(cellData)
        cellDataShaft{iRow} = cellDataShaft{iRow}(indexDis,:);
        % save the ordered data
        orderCellData{iRow} = [orderCellData{iRow};cellDataShaft{iRow}];
    end % end for iRow
end % end for iShaft

% convert cell to struct
Temporary = cell2struct(orderCellData, fieldName, 1);

% output
orderedElement = Temporary;
orderedElement.amount = Element.amount;

end