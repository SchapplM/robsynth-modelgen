% Calculate vector of inverse dynamics joint torques for
% %RN%
% The function exploits the sparsity of the regressor matrix
% 
% Input:
% RV [%robot_NTAUJFIXBREGNN%x1]
%   vector of non-Null entries of the regressor matrix. (columns, then rows).
%   see %RN%_invdynJ_fixb_regmin2vec.m
% %INPUT_MDPFIXB%
% 
% Output:
% tauJ [%NQJ%x1]
%   joint torques of inverse dynamics (contains inertial, gravitational Coriolis and centrifugal forces)

% %VERSIONINFO%

function tauJ = %FN%(RV, MDP)
