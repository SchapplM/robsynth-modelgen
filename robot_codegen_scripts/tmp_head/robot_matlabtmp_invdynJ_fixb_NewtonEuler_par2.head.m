% Calculate vector of inverse dynamics joint torques for with Newton-Euler
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
% tauJ [%NQJ%x1]
%   joint torques of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function tauJ = %FN%(qJ, qJD, qJDD, g, ...
  pkin, m, mrSges, Ifges)
