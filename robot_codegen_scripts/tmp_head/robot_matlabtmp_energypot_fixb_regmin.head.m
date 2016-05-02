% Calculate minimal parameter regressor of potential energy for
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
% U_reg [1x%NMPV%]
%   minimal parameter regressor of Potential energy

function U_reg = %RN%_energypot_fixb_regmin_sym_lag_varpar(q, g_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh)