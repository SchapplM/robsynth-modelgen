% Calculate matrix of centrifugal and coriolis load on the joints for
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
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% Cq [%NQJ%x(6+%NQJ%)]
%   matrix of coriolis and centrifugal joint torques
%   Gives coriolis joint torques when multiplied with base and joint velocities

% %VERSIONINFO%

function Cq = %FN%(q, qD, phi_base, xD_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
