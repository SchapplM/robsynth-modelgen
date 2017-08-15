% Calculate joint-base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% Mqb [%NQJ%x6]
%   joint-base inertia matrix (gives inertial forces on the joins from base acceleration)

% %VERSIONINFO%

function Mqb = %FN%(qJ, phi_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
