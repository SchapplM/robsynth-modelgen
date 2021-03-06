% Calculate inertial parameters regressor of joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
% 
% Output:
% MM_reg [((%NQJ%+1)*%NQJ%/2)x(%NQJ%*10)]
%   inertial parameter regressor of joint inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MM_reg = %FN%(qJ, pkin)
