% Calculate homogenous joint transformation matrices for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PKIN%
% 
% Output:
% T_mdh [4x4x%NJ%]
%   homogenous transformation matrices for joint transformation (MDH)
%   Transformation matrices from one joint to the next (not: from base to joints)

% %VERSIONINFO%

function T_mdh = %FN%(q, ...
  pkin)
