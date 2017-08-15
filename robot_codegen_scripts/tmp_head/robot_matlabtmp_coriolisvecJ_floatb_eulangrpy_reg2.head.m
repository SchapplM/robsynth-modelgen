% Calculate inertial parameter regressor for vector of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_XDB%
% %INPUT_PKIN%
% 
% Output:
% tauJc_reg [%NQJ%x(10*%NL%)]
%   inertial parameter regressor for joint torques required to compensate coriolis and centrifugal load

% %VERSIONINFO%

function tauJc_reg = %FN%(qJ, qJD, phi_base, xD_base, pkin)
