% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% 
% Output:
% taug [%NQJ%x1]
%   joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(qJ, g, ...
  pkin, m_num, mrSges_num_mdh)
