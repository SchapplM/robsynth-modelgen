% Calculate forward kinematics (homogenous transformation matrices) for floating-base
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% T_c_mdh [4x4x%NL%]
%   homogenous transformation matrices for each body frame (MDH)
%   1:  world -> mdh base (link 1)
%   2:  world -> mdh link 2 (first articulated link)
%   ...
%   %NL%:  world -> mdh link (%NL%-1) (last articulated link)

% %VERSIONINFO%

function T_c_mdh = %FN%(q, r_base, phi_base, ...
  pkin)
