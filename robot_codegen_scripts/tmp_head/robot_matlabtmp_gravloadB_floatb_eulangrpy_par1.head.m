% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% 
% Output:
% Fg [6x1]
%   base forces required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(qJ, phi_base, g, ...
  pkin, m_num, rSges_num_mdh)
