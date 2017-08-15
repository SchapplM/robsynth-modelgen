% Calculate potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_RB%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% 
% Output:
% U [1x1]
%   Potential energy

% %VERSIONINFO%

function U = %FN%(qJ, r_base, g, ...
  pkin, m_num, mrSges_num_mdh)
