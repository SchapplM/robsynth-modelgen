% Calculate potential energy for
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
% m_num_mdh, rSges_num_mdh [%NL%x1]
%   dynamic parameters
% 
% Output:
% U [1x1]
%   Potential energy

% %VERSIONINFO%

function U = %FN%(q, g, ...
  pkin, m_num, rSges_num_mdh)
