% Calculate minimal parameter regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint angles [rad]
% qD [%NJ%x1]
%   Joint velocities [rad/s]
% qDD [%NJ%x1]
%   Joint accelerations [rad/s]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% tau_reg [%NJ%x%NMPV%]
%   minimal parameter regressor of inverse dynamics joint torque vector

function tau_reg = %RN%_invdyn_joint_fixb_regmin_sym_lag_varpar(q, qD, qDD, g, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh)
