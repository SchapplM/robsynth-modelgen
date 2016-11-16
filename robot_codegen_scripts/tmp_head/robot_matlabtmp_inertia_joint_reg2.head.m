% Calculate inertial parameters regressor of joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% MM_reg [%NQJ%*%NQJ%x(%NQJ%*10)]
%   minimal parameter regressor of joint inertia matrix

% %VERSIONINFO%

function MM_reg = %FN%(q, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%)