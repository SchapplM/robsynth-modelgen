% Calculate vector of cutting forces with Newton-Euler
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
% f_new [3x%NL%]
%   vector of cutting forces (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function f_new = %FN%(qJ, qJD, qJDD, g, ...
  pkin, m, mrSges, Ifges)
