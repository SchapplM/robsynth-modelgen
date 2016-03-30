% Calculate potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint Angles [rad]
% r_base [3x1]
%   Position of the base link in world frame, rotated into mdh base frame
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh [%NL%x1]
%   dynamic parameters
% 
% Output:
% U [1x1]
%   Potential energy

function U = %RN%_energypot_sym_lag_varpar_par2(q, r_base, g_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, mrSges_num_mdh)
