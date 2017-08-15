% Calculate minimal parameter regressor of joint-base floating base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [(6*%NQJ%)x%NMPVFIXB%]
%   minimal parameter regressor of joint-base floating base inertia matrix

% %VERSIONINFO%

function MM_reg = %FN%(q, phi_base, pkin)
