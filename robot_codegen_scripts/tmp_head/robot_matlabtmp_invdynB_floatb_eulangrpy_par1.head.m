% Calculate vector of inverse dynamics generalized base forces for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_QDD%
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
% tauB [6x1]
%   generalized base forces of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% %VERSIONINFO%

function tauB = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
