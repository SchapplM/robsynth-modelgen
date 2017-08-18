% Calculate full inertia matrix (base and joints) for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% M [(6+%NQJ%)x(6+%NQJ%)]
%   full inertia matrix (for base and joint dynamics)

% %VERSIONINFO%

function M = %FN%(qJ, phi_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
