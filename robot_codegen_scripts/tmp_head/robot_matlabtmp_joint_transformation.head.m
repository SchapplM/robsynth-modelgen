% Calculate homogenous joint transformation matrices for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% T_mdh [4x4x%NJ%]
%   homogenous transformation matrices for joint transformation (MDH)
%   Transformation matrices from one joint to the next (not: from base to joints)

% %VERSIONINFO%

function T_mdh = %FN%(q, ...
  pkin)
