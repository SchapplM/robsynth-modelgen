% Calculate inertial parameters regressor of floating base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [((%NQ%+1)*%NQ%/2)x(%NL%*10)]
%   inertial parameter regressor of floating base inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MM_reg = %FN%(q, phi_base, pkin)
