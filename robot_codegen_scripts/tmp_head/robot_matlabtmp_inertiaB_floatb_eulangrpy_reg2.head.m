% Calculate inertial parameters regressor of base floating base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [(6*7/2)x(%NL%*10)]
%   inertial parameter regressor of base floating base inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MM_reg = %FN%(qJ, phi_base, pkin)
