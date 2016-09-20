% Calculate minimal parameter regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Joint angles [rad]
% qD [%NQJ%x1]
%   Joint velocities [rad/s]
% qD [%NQJ%x1]
%   Joint accelerations [rad/s]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% tau_reg [%NJ%x%NMPV%]
%   minimal parameter regressor of inverse dynamics joint torque vector

function tau_reg = %FN%(q, qD, qDD, g, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh)
