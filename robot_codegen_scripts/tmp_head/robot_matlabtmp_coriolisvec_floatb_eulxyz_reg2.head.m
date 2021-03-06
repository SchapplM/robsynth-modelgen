% Calculate inertial parameter regressor of vector of centrifugal and coriolis load for
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
% 
% Output:
% tauc_reg [(6+%NQJ%)x(%NL%*10)]
%   inertial parameter regressor for generalized forces and torques 
%   required to compensate coriolis and centrifugal load (floating base)

% %VERSIONINFO%

function tauc_reg = %FN%(qJ, qJD, phi_base, xD_base, pkin)
