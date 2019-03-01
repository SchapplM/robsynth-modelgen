% Calculate vector of cutting torques with Newton-Euler for
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
% m [3x%NL%]
%   vector of cutting torques (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function m = %FN%(qJ, qJD, qJDD, g, ...
  pkin, m, mrSges, Ifges)
