% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh [%NL%x1]
%   dynamic parameters
% 
% Output:
% taug [%NJ%x1]
%   joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(q, g, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh%KCPARG%)
