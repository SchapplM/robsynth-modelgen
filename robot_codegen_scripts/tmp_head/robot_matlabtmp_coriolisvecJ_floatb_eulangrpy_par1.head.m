% Calculate vector of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_XDB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% tauc [%NQJ%x1]
%   joint torques required to compensate coriolis and centrifugal load

% %VERSIONINFO%

function tauc = %FN%(q, qD, phi_base, xD_base, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
