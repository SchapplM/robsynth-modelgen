% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% 
% Output:
% taug [6x1]
%   base forces required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(q, phi_base, g, ...
  pkin, m_num, mrSges_num_mdh)
