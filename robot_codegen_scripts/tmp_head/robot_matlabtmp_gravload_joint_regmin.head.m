% Calculate minimal parameter regressor of gravitation load for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% taug_reg [%NJ%x%NMPV%]
%   minimal parameter regressor of gravitation joint torque vector

% %VERSIONINFO%

function taug_reg = %FN%(q, g, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%)
