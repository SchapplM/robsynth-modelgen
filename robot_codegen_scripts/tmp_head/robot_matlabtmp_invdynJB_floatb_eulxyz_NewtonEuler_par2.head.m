% Calculate vector of inverse dynamics joint torques and base forces Newton-Euler for
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
% tauJB [(6+%NQJ%)x1]
%   joint torques and base forces of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function tauJB = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, ...
  pkin, m, mrSges, Ifges)
