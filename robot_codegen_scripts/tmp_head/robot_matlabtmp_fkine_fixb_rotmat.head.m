% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% T_c_mdh [4x4x%NL%]
%   homogenious transformation matrices for each body frame (MDH)
%   1:  mdh base (link 0) -> mdh link 1
%   ...
%   %NL%:  mdh base (link 0) -> mdh link %NL%

% %VERSIONINFO%

function T_c_mdh = %FN%(q, ...
  pkin)
