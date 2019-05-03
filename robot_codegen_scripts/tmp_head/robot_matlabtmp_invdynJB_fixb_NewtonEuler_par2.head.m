% Calculate vector of inverse dynamics joint torques and base forces with Newton-Euler
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% tauJB [(6+%NQJ%)x1]
%   joint torques and base forces of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function tauJB = %FN%(qJ, qJD, qJDD, g, ...
  pkin, m, mrSges, Ifges)
