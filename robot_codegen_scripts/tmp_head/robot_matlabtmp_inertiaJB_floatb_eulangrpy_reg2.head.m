% Calculate inertial parameters regressor of joint-base floating base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [(6*%NQJ%)x(%NL%*10)]
%   inertial parameter regressor of joint-base floating base inertia matrix

% %VERSIONINFO%

function MM_reg = %FN%(qJ, phi_base, pkin)
