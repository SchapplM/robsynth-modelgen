% Convert inverse dynamics regressor matrix to a vector for
% %RN%
% Use sparsity of the regressor matrix: %robot_NTAUJFIXBREGNN%/(%NQJ%*%NMPVFIXB%) elements are non-zero
%
% Input:
% RM [%NQJ%x%NMPVFIXB%]
%   minimal parameter regressor of inverse dynamics joint torque vector
%
% Output:
% RV [%robot_NTAUJFIXBREGNN%x1]
%   vector of non-Null entries of the input matrix. (columns, then rows).

% %VERSIONINFO%

function RV = %FN%(RM)

