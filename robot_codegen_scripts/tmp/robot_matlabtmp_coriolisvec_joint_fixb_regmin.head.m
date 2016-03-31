% Calculate minimal parameter regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint Angles [rad]
% qD [%NJ%x1]
%   Joint Velocities [rad/s]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% taug_reg [%NJ%x%NMPV%]
%   minimal parameter regressor of coriolis joint torque vector

function tauc_reg = %RN%_coriolisvec_joint_fixb_regmin_sym_lag_varpar(q, qD, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh)
