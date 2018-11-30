% Calculate vector of centrifugal and Coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% tauc [%NQJ%x1]
%   joint torques required to compensate Coriolis and centrifugal load

% %VERSIONINFO%

function tauc = %FN%(qJ, qJD, ...
  pkin, m, mrSges, Ifges)
