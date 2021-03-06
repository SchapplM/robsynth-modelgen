% Calculate minimal parameter regressor of joint inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
% 
% Output:
% MMD_reg [((%NQJ%+1)*%NQJ%/2)x%NMPVFIXB%]
%   minimal parameter regressor of inertia matrix time derivative
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MMD_reg = %FN%(qJ, qJD, ...
  pkin)
