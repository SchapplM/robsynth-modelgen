% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% r_base [3x1]
%   Base position in world frame
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% T_c_mdh [4x4x%NL%]
%   homogenious transformation matrices for each body frame (MDH)
%   1:  world -> mdh base (link 1)
%   2:  world -> mdh link 2 (first articulated link)
%   ...
%   %NL%:  world -> mdh link %NL% (last articulated link)

% %VERSIONINFO%

function T_c_mdh = %FN%(q, r_base, phi_base, ...
  pkin)
