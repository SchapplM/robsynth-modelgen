% Calculate inertial parameters regressor of joint inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% MMD_reg [((%NQJ%+1)*%NQJ%/2)x(%NQJ%*10)]
%   inertial parameter regressor of inerta matrix time derivative
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MMD_reg = %FN%(qJ, qJD, pkin)
