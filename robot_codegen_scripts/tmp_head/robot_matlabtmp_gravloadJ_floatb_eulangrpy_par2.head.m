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
% %INPUT_MR%
% 
% Output:
% taug [%NQJ%x1]
%   joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(qJ, phi_base, g, ...
  pkin, m_num, mrSges_num_mdh)
