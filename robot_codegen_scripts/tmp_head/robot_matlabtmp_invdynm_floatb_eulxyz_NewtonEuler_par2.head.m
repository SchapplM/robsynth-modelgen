% Calculate vector of cutting torques for
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
% %INPUT_MR%
% %INPUT_IF%
%
% Output:
% m_new [3x%NL%]
%   vector of cutting torques (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function m_new = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, ...
  pkin, m, mrSges, Ifges)
