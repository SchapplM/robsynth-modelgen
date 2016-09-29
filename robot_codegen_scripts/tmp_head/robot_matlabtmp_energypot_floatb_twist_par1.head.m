% Calculate potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% r_base [3x1]
%   Position of the base link in world frame, rotated into mdh base frame
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh [%NL%x1]
%   dynamic parameters
% 
% Output:
% U [1x1]
%   Potential energy

% %VERSIONINFO%

function U = %FN%(q, r_base, g, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh%KCPARG%)
