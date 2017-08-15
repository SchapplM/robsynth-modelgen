% Calculate base parameter regressor for inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_RB%
% %INPUT_PKIN%
% 
% Output:
% MD_reg [(((6+%NQJ%)+1)*(6+%NQJ%)/2)x%NMPVFLOATB%]
%   base parameter regressor for full time derivative of inertia matrix (for base and joint dynamics)
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MD_reg = %FN%(q, qD, phi_base, xD_base, pkin)
