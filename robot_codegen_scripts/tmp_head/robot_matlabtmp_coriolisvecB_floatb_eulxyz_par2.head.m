% Calculate vector of centrifugal and coriolis load on the base for
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
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% Fc [6x1]
%   base forces and torques required to compensate coriolis and centrifugal load

% %VERSIONINFO%

function Fc = %FN%(qJ, qJD, phi_base, xD_base, ...
  pkin, m, mrSges, Ifges)
