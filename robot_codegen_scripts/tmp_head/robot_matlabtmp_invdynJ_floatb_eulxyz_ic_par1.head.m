% Calculate vector of inverse dynamics with ic joint torques for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_XDB%
% %INPUT_XDDB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% tau [%NAJ%x1]
%   joint torques of inverse dynamics  with ic (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function tau = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, ...
  pkin, m, rSges, Icges)
