% Calculate minimal parameter regressor of joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [((%NQJ%+1)*%NQJ%/2)x%NMPVFIXB%]
%   minimal parameter regressor of joint inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MM_reg = %FN%(q, pkin)
