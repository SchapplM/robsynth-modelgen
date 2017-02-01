% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh [%NL%x1]
%   dynamic parameters
% 
% Output:
% taug [%NQJ%x1]
%   joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(q, g, ...
  pkin, m_num, mrSges_num_mdh)
