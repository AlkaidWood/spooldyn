%% findIndex - Calculate global matrix indices for element assembly
%
% This function computes the global matrix indices for finite element 
% assembly based on node positions and system DOF configuration.
%
%% Syntax
%  elementIndex = findIndex(positionNode, nodeDof)
%
%% Description
% |findIndex| determines the global matrix indices where element matrices 
% should be placed during finite element assembly. The function:
% * Maps node positions to global DOF indices
% * Supports both single-node and dual-node elements
% * Generates index matrices for efficient matrix assembly
%
%% Input Arguments
% * |positionNode| - Node position specification:
%   * Column vector: [n×1] - Node numbers for single-node elements
%   * Matrix: [n×2] - Node pairs for dual-node elements
%     - Column 1: First node numbers
%     - Column 2: Second node numbers
%
% * |nodeDof| - [m×1] DOF count per system node, m = total nodes
%
%% Output Arguments
% * |elementIndex| - Global matrix indices:
%   * For single-node elements: [n×2] matrix where each row contains 
%     [start_index, end_index] for the element
%   * For dual-node elements: [2n×4] matrix organized as:
%     Row 1 (element i): [start_node1, end_node1, start_node2, end_node2]
%     Row 2 (element i): [start_node2, end_node1, start_node2, end_node2]
%     ... continues for each element
%
%% Index Calculation Algorithm
% 1. Single-node elements:
%    index = Σ(nodeDof(1:k-1)) + 1, where k = node number
% 2. Dual-node elements:
%    * For each node pair (k1, k2):
%        index1 = Σ(nodeDof(1:k1-1)) + 1
%        index2 = Σ(nodeDof(1:k2-1)) + 1
%    * Generates four index pairs per element
%
%% Implementation Notes
% * Index calculation accounts for cumulative DOF
% * Output format optimized for |addElementIn| function
% * Supports efficient sparse matrix assembly
%
%% Example
% % Single-node elements (e.g., concentrated masses)
% nodes = [1; 3];
% dofPerNode = [2; 2; 2; 2];
% idx = findIndex(nodes, dofPerNode)
% % Returns: [1,1; 5,5]
%
% % Dual-node elements (e.g., shaft elements)
% nodePairs = [1,2; 2,3];
% dofPerNode = [2;2;2];
% idx = findIndex(nodePairs, dofPerNode)
% % Returns: 
% %   [1,1,1,3; 
% %    3,1,3,3;
% %    3,3,3,5;
% %    5,3,5,5]
%
%% Application
% Essential for:
% * Finite element matrix assembly
% * Connecting local element matrices to global system
% * Handling complex node-element relationships
%
%% See Also
% addElementIn, assembleLinear, femShaft, femDisk
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%

function elementIndex = findIndex(positionNode,nodeDof)

% check input
if size(positionNode,2)>2
    error('incorrect dimension of first input parameter');
end

%%
% calculate the position of element in global matrix
elementNum = size(positionNode,1);

if size(positionNode,2) == 1
    
    node1No = positionNode;
    elementIndex = zeros(elementNum, 2);
    for iElement = 1:1:elementNum
        num1 = sum(nodeDof(1:node1No(iElement)-1)) + 1;
        % position of the first disk element located; 
        elementIndex(iElement,1) = num1;
        elementIndex(iElement,2) = num1;
    end % end for
    
elseif size(positionNode,2) == 2
    
    node1No = positionNode(:,1);
    node2No = positionNode(:,2);
    elementIndex = zeros(2*elementNum,4);
    for iElement = 1:1:elementNum
        num1 = sum(nodeDof(1:node1No(iElement)-1)) + 1;
        num2 = sum(nodeDof(1:node2No(iElement)-1)) + 1;
        % Index11
        elementIndex(2*iElement-1,1) = num1;
        elementIndex(2*iElement-1,2) = num1;
        % Index12
        elementIndex(2*iElement-1,3) = num1;
        elementIndex(2*iElement-1,4) = num2;
        % Index21
        elementIndex(2*iElement,1) = num2;
        elementIndex(2*iElement,2) = num1;
        % Index22
        elementIndex(2*iElement,3) = num2;
        elementIndex(2*iElement,4) = num2;
    end % end for  
    
else
    
    error('incorrect dimension of first input parameter');
    
end % end if


end % end funcion

