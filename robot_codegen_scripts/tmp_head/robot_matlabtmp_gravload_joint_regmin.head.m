% Calculate minimal parameter regressor of gravitation load for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint Angles [rad]
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% taug_reg [%NJ%x%NMPV%]
%   minimal parameter regressor of gravitation joint torque vector

function taug_reg = %RN%_gravload_joint_regmin_sym_lag_varpar(q, g_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh)
