% Calculate floating base base inertia matrix for
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
% Mqb [6x6]
%   base inertia matrix (gives inertial forces on the base from base acceleration)

% %VERSIONINFO%

function Mqb = %FN%(qJ, phi_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
