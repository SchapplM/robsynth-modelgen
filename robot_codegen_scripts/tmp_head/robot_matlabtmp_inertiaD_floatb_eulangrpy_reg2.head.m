% Calculate inertial parameter regressor for inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_XDB%
% %INPUT_PKIN%
% 
% Output:
% MD_reg [(((6+%NQJ%)+1)*(6+%NQJ%)/2)x(%NL%*10)]
%   inertial parameter regressor for full time derivative of inertia matrix (for base and joint dynamics)
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MD_reg = %FN%(qJ, qJD, phi_base, xD_base, pkin)
