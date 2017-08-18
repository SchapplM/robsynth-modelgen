% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_XDB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% Cq [%NQJ%x(6+%NQJ%)]
%   matrix of coriolis and centrifugal joint torques
%   Gives coriolis joint torques when multiplied with base and joint velocities

% %VERSIONINFO%

function Cq = %FN%(qJ, qJD, phi_base, xD_base, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
