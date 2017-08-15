% Calculate minimal parameter regressor of base floating base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [(6*7/2)x%NMPVFIXB%]
%   minimal parameter regressor of base floating base inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MM_reg = %FN%(q, phi_base, pkin)
